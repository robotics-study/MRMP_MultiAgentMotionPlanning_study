#pragma once

#include <string>
#include <vector>

#include "mrmp/core/capabilities.hpp"
#include "mrmp/core/planner.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"

namespace mrmp::search {

// Joint-space A* — the coupled baseline of the MAPF genealogy.
//
// The earlier page's single-robot A*, applied to the whole team at once: the
// search state is the TUPLE of all agent positions at one instant, and ONE
// search (not one per agent) finds a jointly optimal plan. This is the honest
// baseline — complete and sum-of-costs optimal by construction — paid for with
// a state space of |V|^k; it exists to show exactly what the decoupled and
// hybrid poles trade away.
//
// Semantics fixed here (identical in Python/TS):
//
// * A joint state is (c_0, ..., c_{k-1}). An agent whose position equals its
//   goal is ARRIVED: it pins in place (stay-at-goal semantics made structural —
//   its only action is the self-loop) and pays nothing further. The step at
//   which an agent first reaches its goal is that agent's cost, so one joint
//   step costs 1 per still-unarrived agent — exactly len(path) - 1 of the
//   prioritized representation.
// * A joint transition is legal only if the new tuple has no repeated cell
//   (vertex conflict) and no pair swaps cells across the step (edge conflict).
//   Duplicate starts fail the instance outright — no joint state can separate
//   them at t=0.
// * Plain lazy-deletion A*: heap keyed by (f, seq) with a push counter
//   (starting at 1) breaking equal-f ties in insertion order; a state popped
//   for the first time is expanded once and never again (the heuristic — sum of
//   per-agent Manhattan distances, admissible and consistent — means the first
//   pop already carries optimal g). Goal tested at pop.
// * Successor enumeration order is fixed: the cartesian product of the
//   per-agent action lists in agent-index order with the LAST agent varying
//   fastest, each list in the map's fixed up/down/left/right/wait order. Same
//   tie-breaks, same expansions as python/mrmp/search/joint_astar.py bit-for-bit.
//
// The joint state carries no time — arrival times live in the reconstructed
// paths: agent k's path is its position sequence up to its FIRST arrival (after
// which it pinned anyway), so both algorithms hand back identical space-time
// paths for an identical solution.
class JointAStar final : public core::MultiAgentPlanner {
 public:
  explicit JointAStar(core::ParamSet params) : MultiAgentPlanner(std::move(params)) {}

  std::string name() const override { return "joint_astar"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }

  core::MultiPlanResult plan(const core::DiscreteSpace& space,
                             const std::vector<core::AgentTask>& tasks,
                             core::TraceRecorder* recorder) override;
};

}  // namespace mrmp::search
