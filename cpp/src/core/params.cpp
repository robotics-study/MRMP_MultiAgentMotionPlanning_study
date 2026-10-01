#include "mrmp/core/params.hpp"

#include <stdexcept>

#include "mrmp/core/yaml.hpp"

namespace mrmp::core {
namespace {

[[noreturn]] void fail(const std::string& msg) { throw std::runtime_error("param error: " + msg); }

bool is_integer_literal(const std::string& s) {
  if (s.empty()) return false;
  size_t i = (s[0] == '-' || s[0] == '+') ? 1 : 0;
  if (i >= s.size()) return false;
  for (; i < s.size(); ++i) {
    if (s[i] < '0' || s[i] > '9') return false;
  }
  return true;
}

void check_range(const std::string& name, double value, const std::optional<double>& lo,
                 const std::optional<double>& hi) {
  if (lo && value < *lo) fail("'" + name + "' below min");
  if (hi && value > *hi) fail("'" + name + "' above max");
}

}  // namespace

ParamSet ParamSet::from_yaml(const std::string& path) {
  YamlNode root = parse_yaml_file(path);
  if (!root.is_map()) fail("config root must be a mapping: " + path);

  ParamSet set;
  set.algorithm_ = root.at("algorithm").as_string();
  set.section_ = root.at("section").as_string();
  // A config declares which family (site section) its algorithm belongs to — a
  // config declaring anything else is stale. Mirrored in Python.
  if (set.section_ != "search" && set.section_ != "sampling" &&
      set.section_ != "decentralized" && set.section_ != "kinodynamic") {
    fail("unknown section '" + set.section_ + "'");
  }
  // Scenario slugs this algorithm runs on — required list of scalars (possibly
  // empty). The matrix runner and web exporter route per config. Mirrored in Python.
  const YamlNode& scenarios = root.at("scenarios");
  if (!scenarios.is_seq() && !scenarios.is_null()) fail("'scenarios' must be a sequence");
  for (const YamlNode& s : scenarios.seq) set.scenarios_.push_back(s.as_string());

  const YamlNode& params = root.at("params");
  if (!params.is_seq() && !params.is_null()) fail("'params' must be a sequence");

  for (const YamlNode& decl : params.seq) {
    if (!decl.is_map()) fail("each param must be a mapping");
    std::string name = decl.at("name").as_string();
    std::string type = decl.at("type").as_string();
    const YamlNode& def = decl.at("default");
    decl.at("description");  // required by schema

    std::optional<double> lo, hi;
    if (decl.has("min")) lo = decl.at("min").as_double();
    if (decl.has("max")) hi = decl.at("max").as_double();

    if (type == "int") {
      if (!is_integer_literal(def.as_string())) fail("'" + name + "' default is not an integer");
      long long v = def.as_int();
      check_range(name, static_cast<double>(v), lo, hi);
      set.values_[name] = static_cast<int>(v);
    } else if (type == "float") {
      double v = def.as_double();
      check_range(name, v, lo, hi);
      set.values_[name] = v;
    } else if (type == "bool") {
      set.values_[name] = def.as_bool();
    } else if (type == "string") {
      set.values_[name] = def.as_string();
    } else if (type == "enum") {
      if (!decl.has("choices")) fail("'" + name + "' enum needs choices");
      std::string v = def.as_string();
      bool ok = false;
      for (const YamlNode& c : decl.at("choices").seq) {
        if (c.as_string() == v) ok = true;
      }
      if (!ok) fail("'" + name + "' default '" + v + "' not in choices");
      set.values_[name] = v;
    } else {
      fail("unknown param type '" + type + "' for '" + name + "'");
    }
  }
  return set;
}

int ParamSet::get_int(const std::string& name) const {
  auto it = values_.find(name);
  if (it == values_.end()) fail("unknown param '" + name + "'");
  if (!std::holds_alternative<int>(it->second)) fail("'" + name + "' is not int");
  return std::get<int>(it->second);
}

double ParamSet::get_float(const std::string& name) const {
  auto it = values_.find(name);
  if (it == values_.end()) fail("unknown param '" + name + "'");
  if (!std::holds_alternative<double>(it->second)) fail("'" + name + "' is not float");
  return std::get<double>(it->second);
}

bool ParamSet::get_bool(const std::string& name) const {
  auto it = values_.find(name);
  if (it == values_.end()) fail("unknown param '" + name + "'");
  if (!std::holds_alternative<bool>(it->second)) fail("'" + name + "' is not bool");
  return std::get<bool>(it->second);
}

std::string ParamSet::get_string(const std::string& name) const {
  auto it = values_.find(name);
  if (it == values_.end()) fail("unknown param '" + name + "'");
  if (!std::holds_alternative<std::string>(it->second)) fail("'" + name + "' is not string");
  return std::get<std::string>(it->second);
}

bool ParamSet::has(const std::string& name) const { return values_.count(name) > 0; }

}  // namespace mrmp::core
