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

namespace mrmp::search {

// Push and Swap — Luna & Bekris (IJCAI 2011): decentralized plan-and-repair.
//
// The completion of the priority branch: agents are still processed one at a time
// in index order, but earlier agents are NOT frozen in place. U records which
// agents already reached their goals; PUSH walks agent r along its static BFS
// shortest path, chain-pushing any occupant blocking that path into the nearest
// reachable empty cell (a BFS from the blocker's cell whose blocked set is exactly
// {A[r]} ∪ U — a parked agent can never be shoved), and when no push can clear
// the way SWAP exchanges r with the blocker instead, restoring every displaced
// agent by replaying the maneuver reversed. On grids (bipartite, no triangles) a
// swap physically needs a free 2x2 block; the paper's Figure-1 T-junction sketch
// is schematic only. The paper leaves POP() order over swap vertices unspecified;
// pinned here to BFS-dequeue order from A[r] (nearest candidate first). Failed
// candidates are speculative: their moves roll back (assignment AND trace
// history); only a successful swap's EXECUTE_SWAP moves join the solution.
//
// Completeness: the paper claims completeness for n <= |V| - 2 on any graph; on
// grids that claim is NOT literally true and this implementation does not fake it
// — a parked agent blocking r's way on a tree has no swap site, so such instances
// fail honestly. The pinned semantics are the faithful algorithm, not an omniscient
// repair oracle.
//
// Determinism contract: every tie-break is pinned (neighbor order up/down/left/
// right everywhere, BFS enqueue order for hole selection and candidate iteration),
// so Python/C++/TS runs produce byte-identical assignments and traces. Mirrors
// python/mrmp/search/push_and_swap.py bit-for-bit.
class PushAndSwap final : public core::MultiAgentPlanner {
 public:
  explicit PushAndSwap(core::ParamSet params) : MultiAgentPlanner(std::move(params)) {}

  std::string name() const override { return "push_and_swap"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }

  core::MultiPlanResult plan(const core::DiscreteSpace& space,
                             const std::vector<core::AgentTask>& tasks,
                             core::TraceRecorder* recorder) override;

 private:
  // One executed move (agent, from, to) — the unit a swap records before rollback.
  struct Move {
    int agent;
    core::Cell from;
    core::Cell to;
  };

  // Mutable simulation state during planning (mirror of Python's _Sim): A is the
  // current assignment, T the target assignment, U the goal cells of finished
  // agents, pi the solution path (one assignment per executed move).
  // expanded_nodes counts BFS node expansions across every shortest-path /
  // hole-finding call — this algorithm has no search frontier of its own.
  struct Sim {
    std::set<core::Cell> free_cells;
    std::vector<core::Cell> A;
    std::vector<core::Cell> T;
    std::set<core::Cell> U;
    std::vector<std::vector<core::Cell>> pi;
    int expanded_nodes = 0;
  };

  // BFS result over free cells avoiding `blocked`: parent map plus the ENQUEUE
  // order (== dequeue order) that pins "first empty cell" scans and candidate
  // iteration. has() mirrors Python's dict membership; chain reconstruction walks
  // parents from a cell back to start, exactly like the Python parent walk.
  struct BfsResult {
    core::Cell start;
    std::vector<core::Cell> order;
    std::map<core::Cell, core::Cell> parent;
    bool has(const core::Cell& c) const { return c == start || parent.count(c) > 0; }
  };

  // Free 4-neighbors of c in the fixed order (the map layer's move-set convention).
  static std::vector<core::Cell> nbrs(const Sim& sim, const core::Cell& c);

  // Index of the agent occupying cell c (the assignment is injective), or nullopt.
  static std::optional<int> occupant(const Sim& sim, const core::Cell& c);

  // One action: `agent` steps to the empty free cell `to`; the new assignment joins
  // pi — and, inside a speculative swap candidate, also the local segment so the
  // whole attempt can roll back. Invariants hold by construction (asserted in debug).
  static void apply_move(Sim& sim, std::vector<Move>* segment, int agent, core::Cell to);

  // BFS over free cells avoiding `blocked`; counts order.size() dequeues into the
  // metric (every enqueued cell is dequeued exactly once).
  BfsResult bfs_parent(Sim& sim, const core::Cell& start,
                       const std::set<core::Cell>& blocked) const;

  // Static BFS shortest path start -> goal (nullopt when unreachable).
  std::optional<std::vector<core::Cell>> path_to(Sim& sim, const core::Cell& start,
                                                 const core::Cell& goal) const;

  // Algorithm 2: walk r along p* to T[r], chain-pushing blockers toward the first
  // reachable hole (BFS avoiding {A[r]} ∪ U).
  bool push(Sim& sim, int r) const;

  // Shared by PUSH and MULTIPUSH: BFS from blocker avoiding `blocked`; the first
  // EMPTY cell in enqueue order is the hole; occupants along the blocker->hole
  // parent chain shift one step toward it, farthest-from-r occupant first.
  bool chain_push(Sim& sim, std::vector<Move>* segment, const core::Cell& blocker,
                  const std::set<core::Cell>& blocked) const;

  // Algorithm 3: exchange r with the agent s blocking r's first step, iterating
  // candidate swap vertices in BFS-dequeue order from A[r].
  bool swap(Sim& sim, int r) const;

  // Composite pair walk: r leads along p = SHORTEST_PATH(A[r], v), s follows into
  // each vacated cell; third-party occupants are chain-pushed away (U IGNORED here,
  // both pair cells blocked in that BFS).
  bool multipush(Sim& sim, std::vector<Move>* segment, int r, int s, const core::Cell& v) const;

  // CLEAR + EXECUTE_SWAP fused: r@v and s@w1 exchange iff some w2 in N(v)\{w1} and
  // w4 in N(w1)\{v} ∩ N(w2) are clearable (an occupant is cleared by stepping it
  // into its own first free neighbor); the four exchange moves go straight onto pi.
  // Clearing a later w4 can refill an already-cleared w2 — that attempt fails like
  // any other clear failure (pinned; reachable from three or more agents).
  bool clear_and_swap(Sim& sim, std::vector<Move>* segment, int r, int s) const;

  // CLEAR one vertex: its occupant steps into its own first free neighbor (fixed
  // order); no free neighbor means this swap candidate fails.
  bool clear_cell(Sim& sim, std::vector<Move>* segment, const core::Cell& x) const;

  // Paper cases 1/2: s was already at its goal (T[s] in U) — push/swap r until it
  // vacates T[s], then send s home. A failing swap invalidates the original swap.
  bool resolve(Sim& sim, int r, int s) const;

  core::MultiPlanResult fail(core::TraceRecorder* recorder, int expanded) const;
};

}  // namespace mrmp::search
