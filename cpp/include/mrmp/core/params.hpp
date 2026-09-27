#pragma once

#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace mrmp::core {

using ParamValue = std::variant<int, double, bool, std::string>;

// Loads and validates an algorithm's parameter set from its configs yaml
// (spec/param_schema.json). Validation (type match, [min,max] range, enum
// choices) runs at load time and throws std::runtime_error ("param error: ...")
// on failure — this is the "param validation failure" contract.
class ParamSet {
 public:
  static ParamSet from_yaml(const std::string& path);

  int get_int(const std::string& name) const;
  double get_float(const std::string& name) const;
  bool get_bool(const std::string& name) const;
  std::string get_string(const std::string& name) const;
  bool has(const std::string& name) const;

  const std::map<std::string, ParamValue>& values() const { return values_; }
  const std::string& algorithm() const { return algorithm_; }
  const std::string& category() const { return category_; }

 private:
  std::string algorithm_;
  std::string category_;
  std::map<std::string, ParamValue> values_;
};

}  // namespace mrmp::core
