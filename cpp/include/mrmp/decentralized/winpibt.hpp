#pragma once

#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "mrmp/core/capabilities.hpp"
#include "mrmp/core/planner.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"

namespace mrmp::decentralized {

// winPIBT — Okumura, Tamura & Défago (IJCAI 2019). The decentralized branch's second
// member: PIBT generalized along the TIME WINDOW. PIBT negotiates one cell per
// timestep; winPIBT lets every agent hold a whole provisional space-time path Π_i and
// extend it w steps ahead at a time, securing the steps one by one in priority order.
// The paper's Algorithm 1/2, mirrored here line for line (hard mode — §4.2.2's
// iterative-use relaxation is NOT applied: goals stay fixed, an agent can be pushed off
// its goal and re-travels to it; termination is still every agent on its goal at once):
//
// * Priorities are PIBT's exactly: standing on the goal resets p_i to ε_i, travelling
//   increments it by one per round; agents extend their paths in decreasing priority.
// * Each round t (Algorithm 2): agent i extends only while ℓ_i ≤ t. The highest-priority
//   such agent gets α = t + w; every later one is capped at κ, the running minimum of
//   what higher priorities already secured — lower priorities may never reserve beyond
//   what higher ones covered. That cap is why a big w behaves like prioritized planning.
// * A call winpibt(i, α) (Algorithm 1): β = max(α, every registered length) fixed at
//   entry; compute an ideal path from the agent's secured end to its goal — static
//   parent-chain route padded with waits at the goal, else a pinned space-time search —
//   valid against every VISIBLE reservation: entering cell v at step τ is invalid iff
//   some other agent's path occupies v at any step in [τ, min(β, ℓ_j)] (a secured cell
//   stays poison until its reservation visibly ends) or swaps with a fully-secured move.
//   No valid walk → copeStuck: pin waits to α and report invalid. Register the whole
//   extension at once but SECURE it step by step; securing step t forces anyone parked
//   on that cell (retroactively, one step per inheritance call) out of it first. A failed
//   inheritance retracts the unsecured suffix and replans from the last secured step.
// * Visibility is by secured length ℓ_j, not registration: a candidate step at τ only
//   sees what agents j secured up to their own ℓ_j. That is the disentangled condition
//   (paper Def 3.4) doing its job — beyond ℓ_j nothing is decided yet, and whoever
//   still occupies a cell on the incoming path gets pulled forward by inheritance before
//   that step is ever secured. Rotations need no special case: an agent whose own path
//   already covers the contested step simply is not an inheritance target (ℓ_j ≥ τ).
// * w = 1 collapses to PIBT's per-cell negotiation; large w approaches prioritized
//   planning. The window is this branch's coupling axis as a single parameter.
//
// Pinned degrees of freedom (the paper leaves these free or random; identical here in
// Python, C++ and TS by construction): ε_i = (k-1-i)/k with agent 0 highest; the static
// route's BFS parent chain over fixed neighbor order up/down/left/right, padded with
// waits at the goal / truncated to the horizon; the fallback space-time search — A* over
// (cell, step) states, f = step + static BFS distance to the goal, ties broken by later
// step first then insertion order, expansion in fixed neighbor order with wait last, and
// at every popped state a fresh try at the padded static route (success = surviving to β);
// inheritance targets picked in agent-index order. The paper's random candidate shuffle
// has no analogue here — nothing is random.
//
// The run stops when every agent simultaneously stands on its goal (makespan = that
// step) or at the max_steps budget — an honest "no solution found within budget", never
// a proof of unsolvability. Reachability (paper Theorem 4.3): on dodgeable graphs
// (every adjacent pair of free cells on a simple cycle) and finite window, every agent
// reaches its goal in finite time — the same condition PIBT needed, for the same reason:
// an edge on no cycle gives an agent with a reserved cell nowhere to retroactively leave.
//
// Paths are full-horizon like the PIBT page: paths[k][t] is agent k's cell at step t up
// to the makespan; sum_of_costs counts actual moves (waits cost nothing); expanded_nodes
// counts winpibt() decision-procedure invocations (top-level + inherited) — this
// algorithm has no search frontier of its own, only these calls.
// Mirrors python/mrmp/decentralized/winpibt.py field-for-field.
class Winpibt final : public core::MultiAgentPlanner {
 public:
  explicit Winpibt(core::ParamSet params) : MultiAgentPlanner(std::move(params)) {}

  std::string name() const override { return "winpibt"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }

  core::MultiPlanResult plan(const core::DiscreteSpace& space,
                             const std::vector<core::AgentTask>& tasks,
                             core::TraceRecorder* recorder) override;

 private:
  // Mutable state of one run (mirror of Python's _Sim): paths[i][t] is agent i's
  // provisional path — cells registered up to ℓ_i (sim.ell[i]); positions at time t are
  // paths[i][t]. eps carries PIBT's base priorities; calls counts every winpibt()
  // invocation (the expanded_nodes metric); dists[k] is the static BFS distance table
  // from goal k (absent = unreachable, sorts last).
  struct Sim {
    std::set<core::Cell> free_cells;
    std::vector<core::Cell> goals;
    std::vector<std::map<core::Cell, int>> dists;
    std::vector<std::vector<core::Cell>> paths;
    std::vector<int> ell;
    std::vector<double> eps;
    int calls = 0;
  };

  // Free 4-neighbors of c in the fixed order (out-of-bounds cells are not in the free
  // set by construction, so membership alone bounds the grid).
  static std::vector<core::Cell> nbrs(const std::set<core::Cell>& free_cells, const core::Cell& c);

  // Static BFS distance table from the goal over free cells (fixed neighbor order).
  // Reachability only — distances ignore all other agents by design.
  static std::map<core::Cell, int> bfs(const std::set<core::Cell>& free_cells, core::Cell goal);

  // The paper's Algorithm 1. Extend agent i's provisional path to step alpha; secure the
  // new steps one at a time, forcing anyone parked on a secured cell out of it
  // (retroactive inheritance). Invalid → copeStuck (pin waits to alpha) and report
  // invalid so the claimant backtracks.
  bool winpibt(Sim& sim, int i, int alpha) const;

  // First agent (index order) whose path ENDS on v strictly before t — the one that
  // must vacate a cell being secured at step t. An agent whose registered path covers t
  // is not a target: their reservation already governs validity.
  static std::optional<int> target(const Sim& sim, const core::Cell& v, int t);

  // validPath + registerPath as one search (the paper leaves "compute the ideal path"
  // free; this is what we pin for it): a walk from v_i(t1) at step t1 that stays
  // disentangled from every VISIBLE reservation up to the horizon beta. A* over
  // (cell, step) states: f = step + static BFS distance to the goal, ties by later step
  // first then insertion order; expansion in fixed neighbor order with wait last. At
  // EVERY popped state the pinned static route is tried first — if valid it IS the ideal
  // path from there; otherwise the search fans out one more step. Success = surviving
  // to a state at step ≥ beta (chain backtracked); failure returns nullopt → copeStuck.
  std::optional<std::vector<core::Cell>> ideal_path(const Sim& sim, int i, int t1,
                                                   int beta) const;

  // Backtrack the space-time chain to its start; index 0 is the start step.
  static std::vector<core::Cell> chain(
      const std::map<std::pair<core::Cell, int>, std::optional<std::pair<core::Cell, int>>>& parent,
      const std::pair<core::Cell, int>& state);

  // The static BFS parent-chain from start to goal (fixed neighbor order), padded with
  // waits at the goal or truncated so it spans exactly steps t1..beta. nullopt when the
  // goal is unreachable on the free graph.
  std::optional<std::vector<core::Cell>> static_chain(const Sim& sim, int i, const core::Cell& start,
                                                     int t1, int beta) const;

  // Every step of the segment (occupying seg[j] at step t1+j) must stay disentangled.
  static bool valid(const Sim& sim, int i, const std::vector<core::Cell>& seg, int t1, int beta);

  // A step v1 → v2 landing at step tau is invalid iff some other agent whose path is
  // SECURED up to at least tau (ℓ_j ≥ tau) occupies v2 at any step in [tau, min(beta,
  // ℓ_j)] — a secured reservation stays poison until it visibly ends (the paper's
  // isolation condition Def 3.4: rule 1 at the exact step plus rule 3 over the later
  // secured steps) — or iff it swaps with a fully-secured move. Beyond ℓ_j nothing is
  // decided for agent j: no constraint applies, and whoever still stands on v2 gets
  // pulled forward by inheritance when the step securing it is reached.
  static bool valid_step(const Sim& sim, int i, const core::Cell& v1, const core::Cell& v2, int tau,
                         int beta);

  core::MultiPlanResult fail(core::TraceRecorder* recorder, int calls) const;
};

}  // namespace mrmp::decentralized
