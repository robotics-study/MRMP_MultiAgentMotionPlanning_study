#pragma once

#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "mrmp/core/capabilities.hpp"
#include "mrmp/core/planner.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"

namespace mrmp::search {

// Prioritized planning — plan agents one at a time, later agents treat earlier
// paths as moving obstacles.
//
// Erdmann & Lozano-Pérez (1987), "On Multiple Moving Objects": assign priorities
// to the objects, then plan motions one object at a time; each planner searches a
// configuration space-time that represents the time-varying constraints imposed by
// the already-planned (time-parameterized) paths. Here the single-agent search is
// plain A* over states (cell, t) — unit-cost 4-connected moves plus wait,
// Manhattan heuristic, goal tested at pop.
//
// Priority order IS the agent index: agent 0 plans first against the static map
// only; every later agent reserves all earlier paths in space-time. Occupancy is
// stay-at-goal: a finished path keeps occupying its final cell forever, so a later
// agent may only enter a cell an earlier path visits after it leaves — and may
// never finish on a cell an earlier path will still occupy later (the goal guard).
//
// Not optimal (each agent is individually optimal against fixed reservations; the
// joint plan need not be), not complete (an early path can box a later agent in —
// that returns success=false, honestly). The time axis is made finite exactly:
// after every reservation has frozen, only simple static paths matter, so states
// beyond frozen + |reachable cells| cannot belong to any minimal feasible plan.
//
// Determinism contract: neighbor order is the map's fixed up/down/left/right/wait
// order and equal-f ties pop in push order (lexicographic (f, seq) heap key), so
// C++/Python/TS runs produce identical expansions and paths. Mirrors
// python/mrmp/search/prioritized_astar.py bit-for-bit.
class PrioritizedAStar final : public core::MultiAgentPlanner {
 public:
  explicit PrioritizedAStar(core::ParamSet params) : MultiAgentPlanner(std::move(params)) {}

  std::string name() const override { return "prioritized_astar"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }

  core::MultiPlanResult plan(const core::DiscreteSpace& space,
                             const std::vector<core::AgentTask>& tasks,
                             core::TraceRecorder* recorder) override;

 private:
  // Space-time A* for one agent against the reserved paths in `planned`. Returns
  // (path, expanded): path is empty when the goal is statically unreachable,
  // already occupied at t=0 by an earlier path, or boxed in past the finite
  // horizon. expanded counts every pop of this sub-search (every state is pushed
  // exactly once, so a pop IS an expansion) — mirrors Python's tuple return.
  std::pair<std::optional<std::vector<core::Cell>>, int> plan_one(
      const core::DiscreteSpace& space, const core::AgentTask& task,
      const std::vector<std::vector<core::Cell>>& planned, int agent,
      core::TraceRecorder* recorder) const;
};

}  // namespace mrmp::search
