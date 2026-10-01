#pragma once

#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mrmp/core/capabilities.hpp"
#include "mrmp/core/planner.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"

namespace mrmp::decentralized {

// PIBT — Okumura, Machida, Défago & Tamura (IJCAI 2019 / Artificial Intelligence 310,
// 2022). The decentralized branch's representative: everything before this branch
// planned the WHOLE plan offline and only then executed it. PIBT has no plan until it
// exists one timestep at a time. Every timestep t: (1) every agent updates its
// priority — standing on the goal resets p_i to ε_i, still travelling increments p_i
// by 1, so any active agent always outranks any parked one and within each group the
// pinned index order rules; (2) agents decide in decreasing priority order, each
// picking its next cell from Neigh(current) ∪ {current} sorted by BFS distance to its
// own goal (ties: unoccupied first — an occupied candidate would force inheritance
// nobody needs yet — then fixed row-major cell order); (3) a cell another agent still
// occupies is not taken but CLAIMED: the occupant inherits the claimant's priority and
// must vacate into one of its own candidates, and if it cannot, the claim backtracks.
// A swap is structurally impossible — an inheriting agent may never move into its
// claimant's current cell — and a vertex conflict is impossible because every claimed
// cell is excluded from every later choice. The top-level call can never fail: whoever
// decides with nobody above them always keeps their own cell, so the group never
// deadlocks by construction; what kills PIBT instead is topology — an edge on no cycle
// (a width-1 corridor) gives the blocked agent nowhere to vacate to, and the run
// deadlocks honestly there.
//
// Completeness (paper Theorem 1): if every pair of adjacent free cells lies on a simple
// cycle of length ≥ 3 (the same sufficient condition Push and Swap needed, for the same
// reason), every agent reaches its goal within diam(G)·|A| steps regardless of
// priorities. Where the graph violates that condition there is no guarantee — head-on
// swaps in width-1 corridors deadlock at the budget and fail honestly, exactly like the
// priority branch's honest failures on tree-shaped maps.
//
// Pinned degrees of freedom (the paper leaves all of these free or random; identical
// here in Python, C++ and TS by construction): ε_i = (k-1-i)/k — distinct values in
// [0,1) as required, agent 0 highest (the repo's index-order convention); priorities
// stay distinct forever because every step adds the same +1 to active agents and resets
// parked ones below all active ones. Candidate order: BFS distance to goal ascending;
// ties unoccupied-before-occupied; final tie-break row-major cell order. Distance
// tables are static BFS from each goal over free cells, computed once at setup (the
// paper's own suggestion against the on-demand-A* bottleneck); cells unreachable from a
// goal sort after every reachable one, still in fixed order. The run stops when every
// agent simultaneously stands on its goal (makespan = that step) or at the max_steps
// budget — an honest "no solution found within budget", never a proof of unsolvability.
//
// Paths are full-horizon like the priority branch: paths[k][t] is agent k's cell at
// step t for every agent up to the makespan, because a parked agent CAN be pushed off
// its goal by an active one — that is what the priority reset encodes. sum_of_costs
// counts actual moves (waits cost nothing); expanded_nodes counts decision-procedure
// invocations (top-level + inherited) — this algorithm has no search frontier at all.
// Mirrors python/mrmp/decentralized/pibt.py bit-for-bit.
class Pibt final : public core::MultiAgentPlanner {
 public:
  explicit Pibt(core::ParamSet params) : MultiAgentPlanner(std::move(params)) {}

  std::string name() const override { return "pibt"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }

  core::MultiPlanResult plan(const core::DiscreteSpace& space,
                             const std::vector<core::AgentTask>& tasks,
                             core::TraceRecorder* recorder) override;

 private:
  // Mutable state of the timestep loop (mirror of Python's _Sim): pos is π[t] (current
  // cells), nxt the partially decided π[t+1] (nullopt = not yet assigned — an agent
  // still undecided can be pulled into an inheritance chain); dists[k][c] is the static
  // BFS distance from cell c to agent k's goal (absent = unreachable, sorts last);
  // calls counts every decision-procedure invocation (the expanded_nodes metric).
  struct Sim {
    std::set<core::Cell> free_cells;
    std::vector<core::Cell> goals;
    std::vector<std::map<core::Cell, int>> dists;
    std::vector<core::Cell> pos;
    std::vector<std::optional<core::Cell>> nxt;
    int calls = 0;
  };

  // Free 4-neighbors of c in the fixed order (out-of-bounds cells are not in the free
  // set by construction, so membership alone bounds the grid).
  static std::vector<core::Cell> nbrs(const std::set<core::Cell>& free_cells, const core::Cell& c);

  // Static BFS distance table from the goal over free cells (fixed neighbor order).
  // Reachability only — distances ignore all other agents by design.
  static std::map<core::Cell, int> bfs(const std::set<core::Cell>& free_cells, core::Cell goal);

  // The paper's recursive procedure. Agent i picks its next cell; a candidate still
  // occupied by an UNDECIDED agent k triggers inheritance (decide(k, i): k inherits i's
  // claim and must vacate or make this claim fail), and the parent's current cell is
  // excluded from i's candidates so a swap can never form. A candidate already claimed
  // by anyone is skipped outright — vertex conflicts are structurally impossible.
  static bool decide(Sim& sim, int i, std::optional<int> from_agent);

  core::MultiPlanResult fail(core::TraceRecorder* recorder, int calls) const;
};

}  // namespace mrmp::decentralized
