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

namespace mrmp::kinodynamic {

// db-CBS — CBS whose low level has momentum. Moldagalieva, Ortiz-Haro, Toussaint &
// Hönig (arXiv:2309.16445), "db-CBS: Discontinuity-Bounded Conflict-Based Search for
// Multi-Robot Kinodynamic Motion Planning": three levels — a single-robot search over
// motion primitives that may be chained with bounded discontinuities (db-A*), the CBS
// high level resolving inter-robot collisions by constraining states, and a joint-space
// trajectory optimization seeded with the result, iterated under a shrinking bound. The
// third level is out of scope for this repo exactly as MAPF-POST's LP variants were;
// what IS discretizable — and what this module implements literally — is the first two,
// plus the iteration over the discontinuity bound itself.
//
// The lattice translation (pinned here, mirrored in every language):
//
// * STATE = (cell, velocity). Velocity is an integer pair, per-axis bounded by the
//   agent's vmax (the same AgentTask field MAPF-POST reads; db-CBS requires integer
//   values — the lattice only quantizes integers). Acceleration is bounded by 1
//   cell/step^2 per axis: from state (x, v) the ideal landings are exactly (x + u, u)
//   for actions u with |u - v| <= 1 per axis and |u| <= vmax per axis — at vmax = 1
//   that bound is vacuous (every 8-neighbor move plus wait is legal from every state:
//   this branch's "plain" motion still differs from the search branch's 4-connected
//   model, and swaps are NOT conflicts here — sampled point robots pass through each
//   other).
// * The discontinuity bound d(x_{k+1}, step(x_k, u_k)) <= delta becomes lattice
//   arithmetic on the state distance max(Manhattan(position), |velocity delta|): a
//   transition is legal iff SOME action u has |u - v| <= 1 per axis, |u| <= vmax, and
//   lands within Manhattan floor(delta) of x' with velocity within floor(delta) of v'.
//   At delta < 1 (floor = 0) both slack terms vanish: exact double integrator —
//   momentum is law. At delta >= 1 the chaining gap covers an extra cell of position
//   AND a jump of velocity: physics becomes negotiable, which is exactly what
//   "discontinuity bound" means. The ladder runs [delta_start, delta_end] = [1.5, 0.5]
//   by default: loose first (feasibility), exact last (the honest model); the LAST rung
//   that succeeds supplies the answer — the paper's anytime structure, honestly labeled
//   in metrics. Looser is NOT simply easier: a looser bound fattens the constraint
//   volume too, so the rungs are independent attempts, never monotone refinements.
// * CONSTRAINTS are volumes: a constraint (cell c at step t) binds agent i iff its
//   position at step t is within Manhattan floor(delta) of c — the paper's d(x_c, x_k)
//   > δ read on a lattice. At delta < 1 that is exactly CBS's vertex constraint; the
//   volume only fattens as the bound loosens. Conflicts are sampled-state co-presence
//   (same cell at the same step) — point robots have no shape to overlap, so edge/swap
//   conflicts do not exist in this branch at ANY delta.
// * The high level is CBS verbatim: best-first over CT nodes keyed by
//   sum-of-arrival-steps then creation order; the earliest conflict (ties by cell row,
//   col, then pair i < j) branches into both agents' children; a node whose sub-search
//   dies never enters the queue; an empty queue is the unsolvability verdict and budget
//   exhaustion is not. Each rung starts a FRESH tree from an unconstrained root (the
//   paper re-initializes per iteration — constraints never cross a delta boundary).
// * Low level: A* over states (cell, velocity) in absolute time; g == t, f = t +
//   ceil(Chebyshev / (vmax + floor(delta))) — admissible and consistent because every
//   legal move shifts each axis by at most that much; every state is pushed exactly
//   once. The goal test reads POSITION only — arrival is arrival, velocity unobserved
//   (a deliberate narrowing of the paper's state-valued x_f: our goal set is every
//   state on the goal cell). A pop counts as the goal only when no constraint binds
//   that cell at any step >= t.
// * The paper's third level (joint-space DDP) and its asymptotic-optimality limit are
//   OUT OF SCOPE: the lattice never refines, so this repo's honest claim ends at
//   "complete on the discretized model at delta < 1" — the ladder tightens dynamics
//   and constraint volume, not resolution.
//
// Determinism across languages: successor sets are enumerated in canonical sorted order
// (row, col, v_row, v_col); heap ties break by creation order exactly like the search
// branch; every pinned number is reproducible byte-for-byte against Python/TS. Mirrors
// python/mrmp/kinodynamic/db_cbs.py bit-for-bit.
class DbCbs final : public core::MultiAgentPlanner {
 public:
  explicit DbCbs(core::ParamSet params) : MultiAgentPlanner(std::move(params)) {}

  std::string name() const override { return "db_cbs"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }

  core::MultiPlanResult plan(const core::DiscreteSpace& space,
                             const std::vector<core::AgentTask>& tasks,
                             core::TraceRecorder* recorder) override;

 private:
  // One CT constraint. The agent must keep its position Manhattan-farther than
  // floor(delta) from `cell` at step t; below delta 1 that is exactly "not here".
  struct Constraint {
    core::Cell cell;
    int t = 0;
  };

  // One CT node: per-agent constraint lists + their individually-optimal state
  // paths (each a list of cells indexed by absolute step — the velocity component
  // never leaves the search, the executed artifact stays a cell sequence).
  struct CtNode {
    std::vector<std::vector<Constraint>> constraints;
    std::vector<std::vector<core::Cell>> paths;
  };

  // One full db-CBS run at one discontinuity bound: a fresh constraint tree whose root
  // plans every agent unconstrained (constraints never cross a delta boundary).
  // Returns the conflict-free joint state-path or nullopt (verdict/budget), plus how
  // many low-level expansions it cost.
  static std::pair<std::optional<std::vector<std::vector<core::Cell>>>, int> rung(
      const std::set<core::Cell>& free, const std::vector<core::AgentTask>& tasks, double delta,
      int max_ct, core::TraceRecorder* recorder);

  // db-A*: A* over the state space (cell, velocity) in absolute time. g == t never
  // improves, so a pop IS the expansion. Returns (path, expanded); empty when the
  // start is born inside a forbidden volume or the goal is statically unreachable.
  static std::pair<std::optional<std::vector<core::Cell>>, int> plan_one(
      const std::set<core::Cell>& free, const core::AgentTask& task,
      const std::vector<Constraint>& constraints, int f, int agent, core::TraceRecorder* recorder);

  // Steps to the goal cell can never undercut Chebyshev / (vmax + floor(delta)).
  static int heuristic(const core::Cell& cell, const core::Cell& goal, int vmax, int f);

  // Every state reachable in one legal transition, in canonical sorted order (the set's
  // own ordering: row, col, then velocity row, col — the same order Python's sorted()
  // yields). A transition (cell, v) -> (x', v') is legal iff SOME action u with
  // |u - v| <= 1 per axis and |u| <= vmax lands within Manhattan f of x' and has
  // velocity within f of v'. At f = 0 that collapses to the exact double integrator.
  static std::set<std::pair<core::Cell, core::Cell>> successors(const std::set<core::Cell>& free,
                                                                const core::Cell& cell,
                                                                const core::Cell& v, int vmax,
                                                                int f);

  // Flood fill over the move relation itself: from a reached cell every passable cell
  // within PER-AXIS distance reach_r is one transition's landing away. The result is a
  // set — flood order never matters — but it stays FIFO for determinism anyway.
  static std::set<core::Cell> reachable(const std::set<core::Cell>& free, const core::Cell& start,
                                        int reach_r);

  // Earliest step with any co-presence (same cell at the same sampled step); ties by
  // (cell row, col), then pair i < j. A swap across a step is NOT a conflict here —
  // point robots sampled at integer steps pass through each other.
  static std::optional<std::tuple<core::Cell, int, std::pair<int, int>>> first_conflict(
      const std::vector<std::vector<core::Cell>>& paths);

  // Cell occupied at step t by a finished path: after arrival the agent stays (its
  // state path simply ends).
  static core::Cell occupied(const std::vector<core::Cell>& path, int t);
};

}  // namespace mrmp::kinodynamic
