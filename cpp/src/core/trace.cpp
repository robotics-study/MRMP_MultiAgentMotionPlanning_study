#include "mrmp/core/trace.hpp"

#include <cmath>
#include <cstdio>

namespace mrmp::core {
namespace {

// JSON numbers: emit integral values without a decimal point (so cells read as
// [row, col] ints) and others with enough precision to round-trip coordinates.
void write_num(std::ostream& os, double v) {
  if (std::isfinite(v) && std::abs(v) < 9e15 && v == std::floor(v)) {
    os << static_cast<long long>(v);
  } else {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.10g", v);
    os << buf;
  }
}

void write_str(std::ostream& os, const std::string& s) {
  os << '"';
  for (char c : s) {
    switch (c) {
      case '"': os << "\\\""; break;
      case '\\': os << "\\\\"; break;
      case '\n': os << "\\n"; break;
      case '\t': os << "\\t"; break;
      case '\r': os << "\\r"; break;
      default: os << c;
    }
  }
  os << '"';
}

void write_array(std::ostream& os, const std::vector<double>& a) {
  os << '[';
  for (size_t i = 0; i < a.size(); ++i) {
    if (i) os << ',';
    write_num(os, a[i]);
  }
  os << ']';
}

void write_param(std::ostream& os, const ParamValue& v) {
  std::visit(
      [&](const auto& v) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, bool>) {
          os << (v ? "true" : "false");
        } else if constexpr (std::is_same_v<T, int>) {
          os << v;
        } else if constexpr (std::is_same_v<T, double>) {
          write_num(os, v);
        } else {
          write_str(os, v);
        }
      },
      v);
}

}  // namespace

TraceRecorder::TraceRecorder(std::ostream& os) : os_(os) {}

void TraceRecorder::begin_event(const char* event) {
  // No wall-clock field: an MRMP trace is ordered by seq alone (see header).
  os_ << "{\"seq\":" << seq_++ << ",\"event\":\"" << event << '"';
}

void TraceRecorder::end_event() { os_ << "}\n"; }

void TraceRecorder::planning_started(const std::string& algorithm, const std::string& map_path,
                                    const std::map<std::string, ParamValue>& params) {
  begin_event("planning_started");
  os_ << ",\"algorithm\":";
  write_str(os_, algorithm);
  os_ << ",\"map\":";
  write_str(os_, map_path);
  // std::map iterates sorted by key — the Python recorder sorts explicitly to match.
  os_ << ",\"params\":{";
  bool first = true;
  for (const auto& [k, v] : params) {
    if (!first) os_ << ',';
    first = false;
    write_str(os_, k);
    os_ << ':';
    write_param(os_, v);
  }
  os_ << '}';
  end_event();
}

void TraceRecorder::planning_finished(bool success, const std::map<std::string, double>& metrics) {
  begin_event("planning_finished");
  os_ << ",\"success\":" << (success ? "true" : "false") << ",\"metrics\":{";
  bool first = true;
  for (const auto& [k, v] : metrics) {
    if (!first) os_ << ',';
    first = false;
    write_str(os_, k);
    os_ << ':';
    write_num(os_, v);
  }
  os_ << '}';
  end_event();
}

void TraceRecorder::node_expanded(const std::vector<double>& state, const std::optional<double>& cost,
                                 const std::optional<int>& agent, const std::optional<int>& t) {
  begin_event("node_expanded");
  os_ << ",\"state\":";
  write_array(os_, state);
  if (cost) {
    os_ << ",\"cost\":";
    write_num(os_, *cost);
  }
  if (agent) os_ << ",\"agent\":" << *agent;
  if (t) os_ << ",\"t\":" << *t;
  end_event();
}

void TraceRecorder::node_expanded(const Cell& c, const std::optional<double>& cost,
                                 const std::optional<int>& agent, const std::optional<int>& t) {
  node_expanded(to_trace(c), cost, agent, t);
}

void TraceRecorder::path_found(const std::vector<Cell>& path, int agent) {
  begin_event("path_found");
  os_ << ",\"path\":[";
  for (size_t i = 0; i < path.size(); ++i) {
    if (i) os_ << ',';
    write_array(os_, to_trace(path[i]));
  }
  os_ << "],\"agent\":" << agent;
  end_event();
}

void TraceRecorder::conflict_found(ConflictKind kind, const Cell& cell, int t,
                                  const std::pair<int, int>& agents,
                                  const std::optional<Cell>& to) {
  begin_event("conflict_found");
  os_ << ",\"kind\":";
  write_str(os_, to_string(kind));
  os_ << ",\"cell\":";
  write_array(os_, to_trace(cell));
  os_ << ",\"t\":" << t;
  os_ << ",\"agents\":[" << agents.first << ',' << agents.second << ']';
  if (to) {
    os_ << ",\"to\":";
    write_array(os_, to_trace(*to));
  }
  end_event();
}

void TraceRecorder::constraint_added(int agent, ConflictKind kind, const Cell& cell, int t,
                                     const std::optional<Cell>& to) {
  begin_event("constraint_added");
  os_ << ",\"agent\":" << agent;
  os_ << ",\"kind\":";
  write_str(os_, to_string(kind));
  os_ << ",\"cell\":";
  write_array(os_, to_trace(cell));
  os_ << ",\"t\":" << t;
  if (to) {
    os_ << ",\"to\":";
    write_array(os_, to_trace(*to));
  }
  end_event();
}

}  // namespace mrmp::core
