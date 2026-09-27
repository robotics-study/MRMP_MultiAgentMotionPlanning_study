#pragma once

#include <map>
#include <string>
#include <vector>

namespace mrmp::core {

// Minimal YAML reader covering the subset our map/scenario/config files use:
// top-level "key: value", 2-space nested maps, block sequences ("- scalar",
// "- {inline}"), inline flow maps/sequences, quoted/plain scalars, "#" comments,
// and folded/literal block scalars (">-", "|"). Not a general YAML implementation.
class YamlNode {
 public:
  enum class Type { Null, Scalar, Sequence, Map };

  Type type = Type::Null;
  std::string scalar;
  std::vector<YamlNode> seq;
  std::map<std::string, YamlNode> map;

  bool is_null() const { return type == Type::Null; }
  bool is_scalar() const { return type == Type::Scalar; }
  bool is_seq() const { return type == Type::Sequence; }
  bool is_map() const { return type == Type::Map; }
  bool has(const std::string& key) const;

  // Throw std::runtime_error on shape/type mismatch (load/validate stage only).
  const YamlNode& at(const std::string& key) const;
  const std::string& as_string() const;
  double as_double() const;
  long long as_int() const;
  bool as_bool() const;
};

YamlNode parse_yaml_file(const std::string& path);
YamlNode parse_yaml_string(const std::string& text);

}  // namespace mrmp::core
