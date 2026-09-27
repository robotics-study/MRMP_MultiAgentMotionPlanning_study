#pragma once

#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "mrmp/core/params.hpp"
#include "mrmp/core/types.hpp"

namespace mrmp::core {

// Conflict / constraint kind: "vertex" = two agents on one cell at one step,
// "edge" = two agents swapping the ends of one edge (a swap is its own event —
// `cell` is one end and `to` the other; direction is not encoded).
enum class ConflictKind { Vertex, Edge };

inline const char* to_string(ConflictKind k) { return k == ConflictKind::Vertex ? "vertex" : "edge"; }

// Emits step-by-step trace events as JSON Lines (spec/trace_schema.json). seq
// starts at 0 and increments per event. Unlike single-robot nav traces, an MRMP
// trace carries NO wall-clock time: replay is driven purely by seq order, and the
// only time an event carries is the discrete timestep `t` of space-time events.
// Both languages serialize each event's fields in exactly the documented order so
// parsed traces match field-for-field. A null TraceRecorder* is never dereferenced
// by planners (hot-path guard at the call site), so tracing is zero-cost when off.
class TraceRecorder {
 public:
  explicit TraceRecorder(std::ostream& os);

  void planning_started(const std::string& algorithm, const std::string& map_path,
                        const std::map<std::string, ParamValue>& params);
  // metrics keys are sorted (std::map) so both languages emit identical order.
  void planning_finished(bool success, const std::map<std::string, double>& metrics);

  // One search-node expansion. `state` is one cell ([row, col]) for per-agent
  // searches (prioritized planning, CBS low level) or the flattened joint state
  // [r0,c0,r1,c1,...] (joint-space A*). `agent` names the owning agent's
  // sub-search when the search is per-agent; `t` is the space-time step of the
  // expanded node (omitted for joint-state expansions, whose state already carries
  // every position at that instant); `cost` is the g-value.
  void node_expanded(const std::vector<double>& state, const std::optional<double>& cost = {},
                     const std::optional<int>& agent = {}, const std::optional<int>& t = {});
  void node_expanded(const Cell& c, const std::optional<double>& cost = {},
                     const std::optional<int>& agent = {}, const std::optional<int>& t = {});

  // One space-time path per agent: path[t] is the cell occupied at step t.
  // `agent` is required — every MRMP result belongs to a named agent.
  void path_found(const std::vector<Cell>& path, int agent);

  // CBS high level (Sharon et al. 2015): two agents' current paths collide.
  void conflict_found(ConflictKind kind, const Cell& cell, int t,
                      const std::pair<int, int>& agents, const std::optional<Cell>& to = {});

  // A branch of the CBS constraint tree: `agent` may not occupy `cell` at step t
  // (vertex) or traverse between `cell` and `to` across step t (edge).
  void constraint_added(int agent, ConflictKind kind, const Cell& cell, int t,
                        const std::optional<Cell>& to = {});

 private:
  void begin_event(const char* event);
  void end_event();

  std::ostream& os_;
  long long seq_ = 0;
};

}  // namespace mrmp::core
