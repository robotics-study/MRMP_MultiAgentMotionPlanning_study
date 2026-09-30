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

// Push and Rotate — de Wilde, ter Mors & Witteveen (JAIR 2014): the completion of
// the priority branch, a complete DECISION procedure. Where Push and Swap gives up
// (no 2x2 block on a tree), this algorithm does not: the graph is first decomposed
// into subgraphs — nontrivial biconnected components plus singleton junctions,
// merged while join vertices sit within distance m−2 of each other (the paper's
// Kornhauser-style feasibility condition) — agents are assigned to subgraphs by
// start AND goal (a mismatch means the instance is not solvable at all), and
// priority relations between subgraphs come from the subgraph tree. Then solve()
// walks each agent along a static BFS path with generalized push/swap primitives,
// and ROTATE rotates every rider on a cycle of the walker's still-open trail one
// step forward when the walker's next step lands on it. With at least two empty
// vertices it finds a move sequence whenever one exists and reports failure
// otherwise — a head-on swap on a tree stays honestly unsolvable, because no
// subgraph hosts an exchange there. (Algorithm 9 move-smoothing is NOT implemented
// — costs are raw move counts.)
//
// Every tie-break is pinned (neighbor order up/down/left/right everywhere,
// row-major scans for clear_vertex hole candidates and subgraph vertex iteration,
// BFS enqueue order for path and merge BFSes, first-clearable cycle vertex in c
// order), so Python/C++/TS runs produce byte-identical assignments and traces.
// Mirrors python/mrmp/search/push_and_rotate.py bit-for-bit; the line numbers in
// the comments cite the paper's pseudocode exactly as the Python docstrings do.
class PushAndRotate final : public core::MultiAgentPlanner {
 public:
  explicit PushAndRotate(core::ParamSet params) : MultiAgentPlanner(std::move(params)) {}

  std::string name() const override { return "push_and_rotate"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }

  core::MultiPlanResult plan(const core::DiscreteSpace& space,
                             const std::vector<core::AgentTask>& tasks,
                             core::TraceRecorder* recorder) override;

 private:
  // One executed move (agent, from, to) — the unit a rotate/swap records before rollback.
  struct Move {
    int agent;
    core::Cell from;
    core::Cell to;
  };

  // Mutable simulation state during planning (mirror of Python's _Sim): A is the
  // current assignment, T the target assignment, pi the solution path (one
  // assignment per executed move). expanded_nodes counts BFS dequeues across every
  // path-finding and clear_vertex scan — this algorithm has no search frontier of
  // its own (the decomposition DFS and merge BFSes are NOT counted). free_cells is
  // an ordered set: iteration order IS the row-major scan order Python sorts into.
  struct Sim {
    std::set<core::Cell> free_cells;
    std::vector<core::Cell> A;
    std::vector<core::Cell> T;
    std::vector<std::vector<core::Cell>> pi;
    int expanded_nodes = 0;
  };

  // BFS result over free cells avoiding `blocked`: parent map plus the ENQUEUE
  // order (== dequeue order) that pins hole-candidate scans and candidate iteration.
  // has() mirrors Python's dict membership; chain walks parents back to start
  // exactly like the Python parent walk.
  struct BfsResult {
    core::Cell start;
    std::vector<core::Cell> order;
    std::map<core::Cell, core::Cell> parent;
    bool has(const core::Cell& c) const { return c == start || parent.count(c) > 0; }
  };

  // Free 4-neighbors of c in the fixed order (the map layer's move-set convention).
  static std::vector<core::Cell> nbrs(const std::set<core::Cell>& free_cells, const core::Cell& c);

  // Index of the agent occupying cell c (the assignment is injective), or nullopt.
  static std::optional<int> occupant(const Sim& sim, const core::Cell& c);

  // One action: `agent` steps from its cell to the empty free cell `to`; the new
  // assignment joins pi — and, inside a speculative swap candidate, also the local
  // segment so the whole attempt can roll back. Invariants hold by construction
  // (asserted in debug builds; Python's caught AssertionError lives at the call
  // sites that can actually be reached).
  static void apply_move(Sim& sim, std::vector<Move>* segment, int agent, core::Cell to);

  // BFS over free cells avoiding `blocked`; counts order.size() dequeues into the
  // metric (every enqueued cell is dequeued exactly once).
  BfsResult bfs_parent(Sim& sim, const core::Cell& start,
                       const std::set<core::Cell>& blocked) const;

  // Static BFS shortest path start -> goal (nullopt when unreachable).
  std::optional<std::vector<core::Cell>> path_to(Sim& sim, const core::Cell& start,
                                                 const core::Cell& goal,
                                                 const std::set<core::Cell>& blocked) const;

  // Algorithm 10: scan unoccupied cells row-major; the FIRST one whose BFS (fixed
  // neighbor order, avoiding U) reaches v wins; occupants on that parent chain shift
  // one step toward u, farthest-from-v occupant first so every target cell is
  // already empty when its mover steps.
  bool clear_vertex(Sim& sim, std::vector<Move>* segment, const core::Cell& v,
                    const std::set<core::Cell>& blocked) const;

  // Algorithm 4: walk r one step onto v. If v is occupied, clear_vertex first (its
  // blocked set is the finished agents' cells plus r's own cell — a parked agent can
  // only be shoved when the walker has already made it unreachable-as-a-hole).
  bool push(Sim& sim, int r, const core::Cell& v, const std::set<core::Cell>& u_set) const;

  // Algorithm 11: bring the adjacent pair (r0, s0) to candidate vertex v — the
  // closer one leads along a static BFS path (pinned: the path's own start cell is
  // skipped), the other follows into each vacated cell.
  bool multipush(Sim& sim, std::vector<Move>* segment, int r0, int s0, const core::Cell& v) const;

  // Algorithm 12: clear two neighbors of the candidate vertex v (four stages; the
  // intricate ones roll their speculative attempts back on inner failure).
  bool clear_pair(Sim& sim, std::vector<Move>* segment, int rp, int sp, const core::Cell& v) const;

  // Algorithm 13: the physical exchange at the junction vertex v (six moves).
  void exchange(Sim& sim, int rp, int sp, const core::Cell& v) const;

  // Algorithm 5: exchange two adjacent agents at a degree >= 3 vertex of r's
  // subgraph (no subgraph — an unassigned agent can never swap, which is what makes
  // tree maps honestly unsolvable); candidates nearest-first (BFS dequeue from
  // A[r0]); speculative candidates roll back, the successful one replays its segment
  // reversed with r/s roles exchanged.
  bool swap(Sim& sim, int r0, int s0, const std::optional<std::set<core::Cell>>& sub_cells) const;

  // Algorithm 6: move every agent on cycle c one step forward. Phase 1 cascades from
  // the first empty cycle cell; phase 2 (fully occupied) pushes off + swaps + cascades.
  bool rotate(Sim& sim, const std::vector<core::Cell>& c, const std::vector<std::optional<int>>& f,
              const std::vector<std::set<core::Cell>>& s_list) const;

  // The cascade inside rotate: every rider on cycle c (cycle order = c's list order)
  // fills the empty slot e backwards — exactly n-1 visits, every rider moves once.
  void cascade(Sim& sim, const std::vector<core::Cell>& c, const core::Cell& e) const;

  // Algorithm 1: nontrivial biconnected components (Hopcroft-Tarjan DFS, roots and
  // neighbors in fixed order), singleton subgraphs for uncovered degree >= 3 cells
  // row-major, then the merge loop over pairs by index order (first pair whose
  // set distance <= m-2 merges; scan restarts from the top).
  std::vector<std::set<core::Cell>> find_subgraphs(const std::set<core::Cell>& free_cells,
                                                   int m) const;

  // Algorithm 1's merge test: multi-source BFS seeded from Si's cells row-major (the
  // FIRST Sj cell dequeued pins the pair and their path); returns the path cells
  // when that distance <= m-2, else nullopt.
  std::optional<std::vector<core::Cell>> merge_path(const std::set<core::Cell>& free_cells,
                                                    const std::set<core::Cell>& si,
                                                    const std::set<core::Cell>& sj, int m) const;

  // Algorithm 2: which subgraph confines each agent under assignment X (A or T).
  // Interior vertices assign directly; junctions assign along planks by the paper's
  // m'/m'' counts. Later assignments overwrite earlier ones (pinned).
  std::vector<std::optional<int>> assign_agents(
      const Sim& sim, const std::vector<core::Cell>& x,
      const std::vector<std::set<core::Cell>>& s_list, int m) const;

  // The unique maximal path from u AWAY from v (v excluded at the first step; cells
  // belonging to no subgraph have degree <= 2 so the continuation is unique),
  // INCLUDING the stopping cell — the far subgraph's join vertex — or a dead end.
  std::vector<core::Cell> plank_walk(const std::set<core::Cell>& free_cells, const core::Cell& v,
                                     const core::Cell& u,
                                     const std::vector<std::set<core::Cell>>& s_list) const;

  // Walk from u away from v; return (sj_index, cells INCLUDING the stopping cell),
  // or nullopt at a dead end.
  std::pair<std::optional<int>, std::vector<core::Cell>> plank_to_subgraph(
      const std::set<core::Cell>& free_cells, const core::Cell& v, const core::Cell& u,
      const std::vector<std::set<core::Cell>>& s_list) const;

  // Algorithm 3 lines 3-10 for one (Si, v): walk every plank leading away from v;
  // the first goal-bearing cell on a walk whose occupant-goal agent IS assigned to
  // some subgraph Sj yields that relation.
  std::optional<int> plank_relation(const std::set<core::Cell>& free_cells,
                                    const std::vector<std::set<core::Cell>>& s_list,
                                    const std::vector<core::Cell>& t,
                                    const std::vector<std::optional<int>>& f, int si,
                                    const std::set<core::Cell>& vi_set, const core::Cell& v) const;

  // Algorithm 3: Si < Sj means Si is planned FIRST (its agents get higher priority).
  // For every subgraph vertex row-major: the first plank of Si whose walk hits a
  // goal assigned to another subgraph adds (si, f(r)) — then move on to the next v.
  std::vector<std::pair<int, int>> subgraph_priority(
      const std::set<core::Cell>& free_cells, const std::vector<std::set<core::Cell>>& s_list,
      const std::vector<core::Cell>& t, const std::vector<std::optional<int>>& f) const;

  // Transitive closure of the priority relation; a self-relation means unsolvable.
  static std::set<std::pair<int, int>> closure(std::vector<std::pair<int, int>> rels);

  // Pinned next-agent rule (the paper leaves it open): assigned agents go first —
  // candidates are unfinished agents whose subgraph has no ACTIVE predecessor; the
  // lowest index among candidates wins. With none left, the lowest unassigned one.
  static int next_agent(const std::set<int>& unfinished, const std::vector<std::optional<int>>& f,
                        const std::set<std::pair<int, int>>& closure);

  // Algorithm 8: walk each agent along a static BFS path with push/swap; rotate
  // fires when the walker's next step lands on a resolving agent's open trail.
  bool solve(Sim& sim, const std::vector<std::set<core::Cell>>& s_list,
             const std::vector<std::optional<int>>& f,
             const std::set<std::pair<int, int>>& closure) const;

  core::MultiPlanResult fail(core::TraceRecorder* recorder, int expanded) const;
};

}  // namespace mrmp::search
