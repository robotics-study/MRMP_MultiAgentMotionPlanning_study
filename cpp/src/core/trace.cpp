#include "mrmp/core/trace.hpp"

#include <cmath>
#include <format>

namespace mrmp::core {
namespace {

// JSON numbers: emit integral values without a decimal point (so cells read as
// [row, col] ints) and others in the shortest round-trip form — std::format's
// default float format is the shortest decimal that re-reads to the same double,
// exactly what Python's json emits via repr. Parsed values are equal on both
// sides; integers stay integer bytes (Python 5.0 vs C++ 5 parse identically).
void write_num(std::ostream& os, double v) {
  if (std::isfinite(v) && std::abs(v) < 9e15 && v == std::floor(v)) {
    os << static_cast<long long>(v);
  } else {
    os << std::format("{}", v);
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
                                    const std::map<std::string, ParamValue>& params,
                                    const std::optional<Coords>& coords,
                                    const std::optional<std::vector<double>>& radius,
                                    const std::optional<std::vector<double>>& vmax) {
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
  // Omitted for discrete traces (the default reading); continuous traces declare
  // their state pairs are world points and carry each agent's disc radius; timed
  // traces add the per-agent velocity limits (cells per time unit).
  if (coords) {
    os_ << ",\"coords\":";
    write_str(os_, to_string(*coords));
  }
  if (radius) {
    os_ << ",\"radius\":";
    write_array(os_, *radius);
  }
  if (vmax) {
    os_ << ",\"vmax\":";
    write_array(os_, *vmax);
  }
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

void TraceRecorder::path_found(const std::vector<Point>& path, int agent) {
  begin_event("path_found");
  os_ << ",\"path\":[";
  for (size_t i = 0; i < path.size(); ++i) {
    if (i) os_ << ',';
    write_array(os_, to_trace(path[i]));
  }
  os_ << "],\"agent\":" << agent;
  end_event();
}

// One timed route per agent (kinodynamic branch only): `cells` is the route with
// wait actions removed (consecutive cells are adjacent), times[i] the earliest
// arrival time at cells[i] — times[0] is 0 for every agent. Field order matches
// the Python recorder: agent, then cells, then times.
void TraceRecorder::schedule_found(int agent, const std::vector<Cell>& cells,
                                  const std::vector<double>& times) {
  begin_event("schedule_found");
  os_ << ",\"agent\":" << agent;
  os_ << ",\"cells\":[";
  for (size_t i = 0; i < cells.size(); ++i) {
    if (i) os_ << ',';
    write_array(os_, to_trace(cells[i]));
  }
  os_ << "],\"times\":";
  write_array(os_, times);
  end_event();
}

void TraceRecorder::roadmap_built(int agent, const std::vector<Point>& vertices,
                                  const std::vector<std::pair<int, int>>& edges) {
  begin_event("roadmap_built");
  os_ << ",\"agent\":" << agent;
  os_ << ",\"vertices\":[";
  for (size_t i = 0; i < vertices.size(); ++i) {
    if (i) os_ << ',';
    write_array(os_, to_trace(vertices[i]));
  }
  os_ << "],\"edges\":[";
  for (size_t i = 0; i < edges.size(); ++i) {
    if (i) os_ << ',';
    os_ << '[' << edges[i].first << ',' << edges[i].second << ']';
  }
  os_ << ']';
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
