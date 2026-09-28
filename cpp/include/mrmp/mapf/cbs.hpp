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

namespace mrmp::mapf {

// CBS — Conflict-Based Search, the hybrid pole of the MAPF genealogy.
//
// Sharon, Stern, Felner & Sturtevant (2015), "Conflict-Based Search for Optimal
// Multi-Agent Pathfinding": stop searching the joint state AND stop hoping a fixed
// priority order works out. Instead plan every agent alone against explicit
// constraints (the low level — wave 1's space-time A*, reservations replaced by
// constraints), keep each single-agent solution as a node of a CONSTRAINT TREE, and
// when the cheapest node's joint solution still collides, branch: one child forbids
// the first agent from that collision, the other forbids the second. Optimal like
// joint-space search without ever touching |V|^k — agents are coupled only at the
// exact cell and step where they actually collide.
//
// One honest caveat about termination, which is why this planner has a parameter
// where its two siblings have none: CBS is OPTIMAL and COMPLETE but only
// SEMI-decidable. On an unsolvable instance the tree never dies (every branch just
// pushes the conflict to a later step, forever), so no run can ever WAIT for an
// "unsolvable" verdict; `max_ct_expansions` stops the search instead. A queue that
// empties on its own IS a verdict; hitting the budget is only "no solution found
// within budget".
//
// Semantics fixed here (identical in Python/TS): occupancy is stay-at-goal
// everywhere; a VERTEX conflict at step t means occ_i(t) == occ_j(t); an EDGE
// conflict at step t means a pair swapped cells across the step (t-1 -> t). Both
// kinds are keyed by the ARRIVAL step. Edge conflicts/constraints are canonicalized:
// `cell` is the lexicographic min of the two cells, `to` the max; direction is not
// encoded. Conflict selection: earliest step; ties by (cell row, col), then pair
// i < j. A constraint (vertex c@t) forbids occupying c at t; (edge {c,d}@tau)
// forbids the swap across that step in EITHER direction. The low level accepts a
// goal pop only when no vertex constraint on the goal cell binds at any t' >= t.
// High level: best-first over CT nodes keyed by (sum-of-costs, creation seq); the
// root expands at construction (every agent unconstrained, index order; any
// sub-search failing fails the instance). A child whose low level fails dies and is
// never pushed. expanded_nodes counts every low-level pop across ALL sub-searches.
//
// Determinism contract: identical heap tie-breaks ((cost, seq) / (f, seq), counters
// from 0), identical neighbor order and conflict ordering — C++/Python/TS runs
// produce byte-identical traces. Mirrors python/mrmp/mapf/cbs.py bit-for-bit.
class Cbs final : public core::MultiAgentPlanner {
 public:
  explicit Cbs(core::ParamSet params) : MultiAgentPlanner(std::move(params)) {}

  std::string name() const override { return "cbs"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }

  core::MultiPlanResult plan(const core::DiscreteSpace& space,
                             const std::vector<core::AgentTask>& tasks,
                             core::TraceRecorder* recorder) override;

 private:
  // One CT constraint. Vertex: occupy `cell` at step t is forbidden. Edge: traverse
  // between `cell` and `to` across step t (canonicalized, both ways).
  struct Constraint {
    core::ConflictKind kind;
    core::Cell cell;
    int t;
    std::optional<core::Cell> to;  // edge only
  };

  // One CT node: per-agent constraint lists + their individually-optimal paths.
  struct CtNode {
    std::vector<std::vector<Constraint>> constraints;
    std::vector<std::vector<core::Cell>> paths;
  };

  // Space-time A* for one agent against `constraints` — wave 1's sub-search with
  // constraints replacing reservations. Returns (path, expanded); empty path when
  // the start is constrained at t=0 or the goal is statically unreachable.
  std::pair<std::optional<std::vector<core::Cell>>, int> plan_one(
      const core::DiscreteSpace& space, const core::AgentTask& task,
      const std::vector<Constraint>& constraints, int agent, core::TraceRecorder* recorder) const;

  // Earliest conflict over all agent pairs; ties by (cell row, col), then pair
  // i < j. Returns (kind, cell, t, (i, j), to).
  static std::optional<std::tuple<core::ConflictKind, core::Cell, int, std::pair<int, int>,
                                  std::optional<core::Cell>>>
  first_conflict(const std::vector<std::vector<core::Cell>>& paths);

  // An edge constraint forbids traversing between its two cells in EITHER
  // direction across its step — the pair is canonicalized, not directed.
  static bool matches_edge(const Constraint& c, const core::Cell& from_cell,
                           const core::Cell& to_cell);
};

}  // namespace mrmp::mapf
