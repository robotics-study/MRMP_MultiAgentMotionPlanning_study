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

// RHCR — Rolling-Horizon Collision Resolution, the search branch's hybrid axis folded
// into time itself.
//
// Li, Tinka, Kiesel, Durham, Kumar & Koenig (AAAI-21), "Lifelong Multi-Agent Path
// Finding in Large-Scale Warehouses" (arXiv:2005.07371): stop resolving collisions over
// the whole horizon at once. Decompose the problem into a sequence of WINDOWED MAPF
// instances — each one replanned from the actual positions every h steps, resolving
// collisions only inside the window and ignoring everything beyond it. The batch
// adaptation: every agent carries exactly one real task (start -> goal), and once an
// agent stands on its goal its remaining task sequence is empty — the paper's own
// degenerate case for finite tasks (stay put).
//
// The windowed solver inside is CBS itself, verbatim: the same space-time A* low level,
// the same canonicalized vertex/edge constraints, the same best-first constraint tree
// with the same tie-breaks. The only difference is what counts as a conflict: an
// episode starting at step T resolves collisions whose ARRIVAL step falls in (T, T+w]
// and ignores every collision beyond. Execution commits the first h steps (h <= w —
// every executed step must fall inside some resolved window), and those executed
// positions become the next episode's starts. That is the whole algorithm.
//
// What the window buys and costs (identical in Python/C++/TS by construction):
// PLIABLE, never frozen — executed history carries forward only through each agent's
// CURRENT position; no old path segment stays reserved. NOT optimal, NOT complete by
// design: a collision beyond w is invisible forever, and every windowed instance is
// satisfiable (parking past the horizon always fits), so the tree NEVER empties and the
// only failure mode is the honest budget stop. When w covers an instance's whole
// horizon the single episode IS plain CBS with identical tie-breaks. Execution stops at
// the first step where every agent simultaneously stands on its final goal; cost sums
// each agent's FIRST arrival (flowtime) and paths are the executed trajectories padded
// to that makespan step. Well-formedness (distinct starts, distinct goals, passable
// cells) is checked up front: a t=0 collision is history, not a conflict a window could
// resolve. Fixed degrees of freedom identical across languages: neighbor order
// up/down/left/right-then-wait; A* frontier tie-break f = absolute step + Manhattan,
// ties by insertion order; conflict selection earliest arrival step, ties by (cell row,
// col) then pair i < j; CT queue keyed by (sum of absolute arrivals, creation seq);
// root expansions in agent-index order. Mirrors python/mrmp/search/rhcr.py field for
// field.
class Rhcr final : public core::MultiAgentPlanner {
 public:
  explicit Rhcr(core::ParamSet params) : MultiAgentPlanner(std::move(params)) {}

  std::string name() const override { return "rhcr"; }
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

  // One CT node of one episode: per-agent constraint lists + their individually-
  // optimal absolute-indexed paths.
  struct CtNode {
    std::vector<std::vector<Constraint>> constraints;
    std::vector<std::vector<core::Cell>> paths;
  };

  // One Windowed MAPF episode: plain CBS over absolute space-time, but a conflict only
  // counts when its arrival step falls in (t_now, t_now + window]. Returns each agent's
  // full path (absolute-indexed) or nullopt if some sub-search dies on its constraints.
  std::optional<std::vector<std::vector<core::Cell>>> episode(
      const core::DiscreteSpace& space, const std::vector<core::Cell>& goals,
      const std::vector<std::vector<core::Cell>>& executed, int t_now, int window,
      core::TraceRecorder* recorder, int* expanded) const;

  // A* over absolute space-time (cell, t), t >= t_now — the same sub-search as cbs.cpp
  // with the clock shifted to the episode's start. Returns (path, expanded); empty on
  // an unavoidable start constraint or a statically unreachable goal.
  std::pair<std::optional<std::vector<core::Cell>>, int> plan_one(
      const core::DiscreteSpace& space, const core::Cell& start, const core::Cell& goal,
      const std::vector<Constraint>& constraints, int t_now, core::TraceRecorder* recorder,
      int agent) const;

  // Earliest conflict whose ARRIVAL step falls inside the window, (t_now, t_now +
  // window]; ties by (cell row, col), then pair i < j. Beyond the window nothing is a
  // conflict: that is what the window IS.
  static std::optional<std::tuple<core::ConflictKind, core::Cell, int, std::pair<int, int>,
                                  std::optional<core::Cell>>>
  first_conflict(const std::vector<std::vector<core::Cell>>& paths, int t_now, int window);

  // An edge constraint forbids traversing between its two cells in EITHER direction
  // across its step — the pair is canonicalized, not directed.
  static bool matches_edge(const Constraint& c, const core::Cell& from_cell,
                           const core::Cell& to_cell);

  // Success: truncate every trajectory at the co-presence step and report (cost = each
  // agent's FIRST arrival summed — the paper's flowtime).
  core::MultiPlanResult finish(const std::vector<std::vector<core::Cell>>& executed,
                               const std::vector<core::Cell>& goals, int done_at, int calls,
                               core::TraceRecorder* recorder) const;

  // Honest budget exhaustion (or a sub-search that died on its constraints).
  core::MultiPlanResult fail(core::TraceRecorder* recorder, int calls) const;
};

}  // namespace mrmp::search
