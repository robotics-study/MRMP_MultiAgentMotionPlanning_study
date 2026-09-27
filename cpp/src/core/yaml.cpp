#include "mrmp/core/yaml.hpp"

#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace mrmp::core {
namespace {

std::string ltrim(const std::string& s) {
  size_t i = 0;
  while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
  return s.substr(i);
}

std::string rtrim(const std::string& s) {
  size_t i = s.size();
  while (i > 0 && std::isspace(static_cast<unsigned char>(s[i - 1]))) --i;
  return s.substr(0, i);
}

std::string trim(const std::string& s) { return rtrim(ltrim(s)); }

int indent_of(const std::string& s) {
  int n = 0;
  while (n < static_cast<int>(s.size()) && s[n] == ' ') ++n;
  return n;
}

bool is_blank(const std::string& s) {
  for (char c : s) {
    if (!std::isspace(static_cast<unsigned char>(c))) return false;
  }
  return true;
}

// Strip an inline "# ..." comment (space-preceded, outside quotes).
std::string strip_comment(const std::string& s) {
  bool in_s = false, in_d = false;
  for (size_t i = 0; i < s.size(); ++i) {
    char c = s[i];
    if (c == '\'' && !in_d) {
      in_s = !in_s;
    } else if (c == '"' && !in_s) {
      in_d = !in_d;
    } else if (c == '#' && !in_s && !in_d && (i == 0 || s[i - 1] == ' ' || s[i - 1] == '\t')) {
      return s.substr(0, i);
    }
  }
  return s;
}

// Content of a line with comment removed and edges trimmed. Empty => skip line.
std::string body_of(const std::string& raw) { return trim(strip_comment(raw)); }

bool skippable(const std::string& raw) { return body_of(raw).empty(); }

std::string unquote(const std::string& s) {
  if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                        (s.front() == '\'' && s.back() == '\''))) {
    std::string inner = s.substr(1, s.size() - 2);
    if (s.front() == '"') {
      std::string out;
      for (size_t i = 0; i < inner.size(); ++i) {
        if (inner[i] == '\\' && i + 1 < inner.size()) {
          char n = inner[++i];
          out += (n == 'n') ? '\n' : (n == 't') ? '\t' : n;
        } else {
          out += inner[i];
        }
      }
      return out;
    }
    return inner;
  }
  return s;
}

// Index of the mapping ':' (top level, outside quotes/brackets, followed by
// space or end). npos if the line is not a "key: value" entry.
size_t find_key_colon(const std::string& b) {
  bool in_s = false, in_d = false;
  int depth = 0;
  for (size_t i = 0; i < b.size(); ++i) {
    char c = b[i];
    if (c == '\'' && !in_d) {
      in_s = !in_s;
    } else if (c == '"' && !in_s) {
      in_d = !in_d;
    } else if (!in_s && !in_d) {
      if (c == '[' || c == '{') {
        ++depth;
      } else if (c == ']' || c == '}') {
        --depth;
      } else if (c == ':' && depth == 0 && (i + 1 >= b.size() || b[i + 1] == ' ')) {
        return i;
      }
    }
  }
  return std::string::npos;
}

// Split on top-level commas, respecting nested brackets and quotes.
std::vector<std::string> split_top(const std::string& s) {
  std::vector<std::string> out;
  bool in_s = false, in_d = false;
  int depth = 0;
  size_t start = 0;
  for (size_t i = 0; i < s.size(); ++i) {
    char c = s[i];
    if (c == '\'' && !in_d) {
      in_s = !in_s;
    } else if (c == '"' && !in_s) {
      in_d = !in_d;
    } else if (!in_s && !in_d) {
      if (c == '[' || c == '{') {
        ++depth;
      } else if (c == ']' || c == '}') {
        --depth;
      } else if (c == ',' && depth == 0) {
        out.push_back(s.substr(start, i - start));
        start = i + 1;
      }
    }
  }
  out.push_back(s.substr(start));
  return out;
}

YamlNode make_scalar(const std::string& raw) {
  YamlNode n;
  n.type = YamlNode::Type::Scalar;
  n.scalar = unquote(raw);
  return n;
}

YamlNode parse_flow(const std::string& raw) {
  std::string t = trim(raw);
  if (t.size() >= 2 && t.front() == '[') {
    YamlNode node;
    node.type = YamlNode::Type::Sequence;
    for (const std::string& part : split_top(t.substr(1, t.size() - 2))) {
      std::string e = trim(part);
      if (e.empty()) continue;
      node.seq.push_back((e[0] == '[' || e[0] == '{') ? parse_flow(e) : make_scalar(e));
    }
    return node;
  }
  if (t.size() >= 2 && t.front() == '{') {
    YamlNode node;
    node.type = YamlNode::Type::Map;
    for (const std::string& part : split_top(t.substr(1, t.size() - 2))) {
      std::string e = trim(part);
      if (e.empty()) continue;
      size_t c = find_key_colon(e);
      if (c == std::string::npos) throw std::runtime_error("yaml: bad flow entry: " + e);
      std::string key = unquote(trim(e.substr(0, c)));
      std::string val = trim(e.substr(c + 1));
      node.map[key] = (!val.empty() && (val[0] == '[' || val[0] == '{')) ? parse_flow(val)
                                                                         : make_scalar(val);
    }
    return node;
  }
  return make_scalar(t);
}

struct Parser {
  std::vector<std::string> lines;

  YamlNode parse_block(size_t& i, int min_indent);
  YamlNode parse_block_scalar(size_t& i, int key_indent, bool folded);
};

YamlNode Parser::parse_block_scalar(size_t& i, int key_indent, bool folded) {
  std::vector<std::string> parts;
  while (i < lines.size()) {
    const std::string& raw = lines[i];
    if (is_blank(raw)) {
      parts.emplace_back();
      ++i;
      continue;
    }
    if (indent_of(raw) <= key_indent) break;
    parts.push_back(trim(raw));
    ++i;
  }
  while (!parts.empty() && parts.back().empty()) parts.pop_back();
  YamlNode v;
  v.type = YamlNode::Type::Scalar;
  for (size_t k = 0; k < parts.size(); ++k) {
    if (k) v.scalar += folded ? ' ' : '\n';
    v.scalar += parts[k];
  }
  return v;
}

YamlNode Parser::parse_block(size_t& i, int min_indent) {
  while (i < lines.size() && skippable(lines[i])) ++i;
  if (i >= lines.size() || indent_of(lines[i]) < min_indent) return YamlNode();

  const int indent = indent_of(lines[i]);
  std::string first = body_of(lines[i]);

  // A block whose content is a single flow collection (e.g. an inline "- {..}"
  // sequence item rewritten to "{..}") is one flow node on this line.
  if (!first.empty() && (first[0] == '{' || first[0] == '[')) {
    ++i;
    return parse_flow(first);
  }

  bool is_seq = first == "-" || (first.size() >= 2 && first[0] == '-' && first[1] == ' ');

  YamlNode node;
  if (is_seq) {
    node.type = YamlNode::Type::Sequence;
    while (i < lines.size()) {
      while (i < lines.size() && skippable(lines[i])) ++i;
      if (i >= lines.size()) break;
      int ci = indent_of(lines[i]);
      if (ci < indent) break;
      if (ci > indent) throw std::runtime_error("yaml: unexpected indent in sequence");
      std::string b = body_of(lines[i]);
      bool item = b == "-" || (b.size() >= 2 && b[0] == '-' && b[1] == ' ');
      if (!item) break;
      if (b == "-") {
        ++i;
        node.seq.push_back(parse_block(i, indent + 1));
      } else {
        // An inline "- <content>" item: the content column becomes the block
        // indent for that item, so following aligned lines join the same node.
        const std::string& raw = lines[i];
        int k = indent + 1;
        while (k < static_cast<int>(raw.size()) && raw[k] == ' ') ++k;
        int item_indent = k;
        lines[i] = std::string(static_cast<size_t>(item_indent), ' ') + raw.substr(k);
        node.seq.push_back(parse_block(i, item_indent));
      }
    }
    return node;
  }

  node.type = YamlNode::Type::Map;
  while (i < lines.size()) {
    while (i < lines.size() && skippable(lines[i])) ++i;
    if (i >= lines.size()) break;
    int ci = indent_of(lines[i]);
    if (ci < indent) break;
    if (ci > indent) throw std::runtime_error("yaml: unexpected indent in map");
    std::string b = body_of(lines[i]);
    if (b == "-" || (b.size() >= 2 && b[0] == '-' && b[1] == ' ')) break;
    size_t colon = find_key_colon(b);
    if (colon == std::string::npos) throw std::runtime_error("yaml: expected 'key: value': " + b);
    std::string key = unquote(trim(b.substr(0, colon)));
    std::string rest = trim(b.substr(colon + 1));
    ++i;
    YamlNode val;
    if (rest.empty()) {
      size_t save = i;
      while (save < lines.size() && skippable(lines[save])) ++save;
      if (save < lines.size() && indent_of(lines[save]) > indent) {
        val = parse_block(i, indent + 1);
      }
    } else if (rest == ">" || rest == ">-" || rest == "|" || rest == "|-") {
      val = parse_block_scalar(i, indent, rest[0] == '>');
    } else if (rest[0] == '[' || rest[0] == '{') {
      val = parse_flow(rest);
    } else {
      val = make_scalar(rest);
    }
    node.map[key] = std::move(val);
  }
  return node;
}

}  // namespace

bool YamlNode::has(const std::string& key) const {
  return type == Type::Map && map.count(key) > 0;
}

const YamlNode& YamlNode::at(const std::string& key) const {
  auto it = map.find(key);
  if (type != Type::Map || it == map.end()) {
    throw std::runtime_error("yaml: missing key '" + key + "'");
  }
  return it->second;
}

const std::string& YamlNode::as_string() const {
  if (type != Type::Scalar) throw std::runtime_error("yaml: value is not a scalar");
  return scalar;
}

double YamlNode::as_double() const {
  try {
    return std::stod(as_string());
  } catch (const std::exception&) {
    throw std::runtime_error("yaml: not a number: '" + scalar + "'");
  }
}

long long YamlNode::as_int() const {
  try {
    return std::stoll(as_string());
  } catch (const std::exception&) {
    throw std::runtime_error("yaml: not an integer: '" + scalar + "'");
  }
}

bool YamlNode::as_bool() const {
  const std::string& s = as_string();
  if (s == "true") return true;
  if (s == "false") return false;
  throw std::runtime_error("yaml: not a bool: '" + s + "'");
}

YamlNode parse_yaml_string(const std::string& text) {
  Parser p;
  std::istringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    p.lines.push_back(line);
  }
  size_t i = 0;
  return p.parse_block(i, 0);
}

YamlNode parse_yaml_file(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("yaml: cannot open '" + path + "'");
  std::stringstream buf;
  buf << f.rdbuf();
  return parse_yaml_string(buf.str());
}

}  // namespace mrmp::core
