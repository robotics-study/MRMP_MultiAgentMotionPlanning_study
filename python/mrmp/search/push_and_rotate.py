"""Push and Rotate — de Wilde, ter Mors & Witteveen (JAIR 2014): a complete decision procedure.

Where Push and Swap gives up (no 2x2 block on a tree), this algorithm does not: it is the
completion of the priority branch. The graph is first decomposed into subgraphs — nontrivial
biconnected components plus singleton junctions, merged while join vertices sit within distance
m−2 of each other (the paper's Kornhauser-style feasibility condition) — agents are assigned to
subgraphs by start AND goal (a mismatch means the instance is not solvable at all), and priority
relations between subgraphs come from the subgraph tree. Then solve() walks each agent along a
static BFS path with generalized push/swap primitives, and ROTATE rotates every rider on a cycle
of the walker's still-open trail one step forward when the walker's next step lands on it.

Completeness here means DECISION procedure (the paper's Theorem 1): with at least two empty
vertices the algorithm finds a move sequence whenever one exists and reports failure otherwise —
a head-on swap on a tree stays honestly unsolvable, because no subgraph hosts an exchange there.

Grid reading of the paper's pseudocode (Algorithms 1-8 for the pipeline, 10-13 for the
primitives; the paper's Algorithm 9 move-smoothing post-processing is NOT implemented — costs are
raw move counts):

- Alg 1 find_subgraphs: Hopcroft-Tarjan biconnected components (DFS roots and neighbors scanned
  in fixed order), singleton subgraphs for uncovered degree >= 3 cells, then the merge loop over
  pairs by index order; a pair merges when their cell-set distance is <= m - 2 (m = empty cells).
- Alg 2 assign_agents: an agent is assigned to the subgraph that hosts its position — interior
  vertices assign directly; junctions assign along planks by the paper's m'/m'' counts. An agent
  whose start and goal land in different subgraphs (or unassigned at one end) makes the instance
  unsolvable: f != f'.
- Alg 3 subgraph_priority: Si < Sj when a plank leaving Si reaches a cell whose goal-assigned
  agent belongs to Sj; the relation is transitively closed, and a self-relation means unsolvable.
- Alg 8 solve: agents are walked one at a time (assigned first by priority closure, lowest index
  among candidates; unassigned agents last). Finished agents' cells block path BFS only on polygon
  graphs (line 5); otherwise push may shove even finished agents off their goals and the unwind
  phase walks them back. When the walker's next step lands on the still-open trail q of a
  resolving agent, rotate fires instead of push/swap.

Every tie-break is pinned (neighbor order up/down/left/right everywhere, row-major scans for
clear_vertex hole candidates and subgraph vertex iteration, BFS enqueue order for path and merge
BFSes, first clearable cycle vertex in c-order), so Python/C++/TS runs produce byte-identical
assignments and traces.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from ..core.capabilities import Capability, DiscreteSpace
from ..core.planner import MultiAgentPlanner
from ..core.trace import TraceRecorder
from ..core.types import AgentTask, Cell, MultiPlanResult, PlanStats

# 4-connected neighbor order — the SAME fixed convention as the map layer's move set. The wait
# self-loop there is not an action of this algorithm: every recorded assignment change is a real
# move.
_MOVES_4 = ((-1, 0), (1, 0), (0, -1), (0, 1))

# One executed move (agent, from, to) — the unit a rotate/swap records before rollback.
Move = tuple[int, Cell, Cell]


def _nbrs(free: frozenset[Cell], c: Cell) -> list[Cell]:
    """Free 4-neighbors of c in the fixed order (out-of-bounds cells are not in `free` by
    construction — membership is the whole check)."""
    return [(c[0] + dr, c[1] + dc) for dr, dc in _MOVES_4 if (c[0] + dr, c[1] + dc) in free]


def _occupant(sim: _Sim, c: Cell) -> int | None:
    """Index of the agent occupying cell c (the assignment is injective), or None."""
    for i, a in enumerate(sim.A):
        if a == c:
            return i
    return None


def _move(sim: _Sim, segment: list[Move] | None, agent: int, to: Cell) -> None:
    """One action: `agent` steps from its cell to the empty free cell `to`, the new assignment
    joins Pi — and when inside a speculative swap candidate, also into the local segment so the
    whole attempt can be rolled back. Every call site constructs these invariants (a move that
    actually moves, an empty target); they are asserted, not recovered from."""
    frm = sim.A[agent]
    assert frm != to and to in sim.free and _occupant(sim, to) is None
    sim.A[agent] = to
    sim.Pi.append(tuple(sim.A))
    if segment is not None:
        segment.append((agent, frm, to))


@dataclass
class _Sim:
    """Mutable simulation state during planning. A is the current assignment (one cell per agent),
    T the target assignment, Pi the solution path — a list of assignments where every executed
    move appends the new one. expanded_nodes counts BFS dequeues across every path-finding and
    clear_vertex scan: this algorithm has no search frontier of its own, so this is what the
    expanded_nodes metric reports (the decomposition DFS and merge BFSes are not counted)."""

    free: frozenset[Cell]
    A: list[Cell]
    T: list[Cell]
    Pi: list[tuple[Cell, ...]] = field(default_factory=list)
    expanded_nodes: int = 0


class PushAndRotate(MultiAgentPlanner):
    @property
    def name(self) -> str:
        return "push_and_rotate"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE}

    def plan(
        self,
        space: DiscreteSpace,
        tasks: list[AgentTask],
        recorder: TraceRecorder | None = None,
    ) -> MultiPlanResult:
        free = frozenset(space.cells())
        starts = [t.start for t in tasks]
        goals = [t.goal for t in tasks]
        # The paper's assignment is injective by definition (unique starts AND unique targets) and
        # every involved cell must be passable. A violating input is not an instance of this
        # problem — report it honestly, no plan.
        if (
            len(set(starts)) < len(starts)
            or len(set(goals)) < len(goals)
            or any(c not in free for c in [*starts, *goals])
        ):
            return self._fail(recorder, 0)

        sim = _Sim(free=free, A=list(starts), T=list(goals), Pi=[tuple(starts)])
        m = len(free) - len(tasks)  # Alg 7 line 1: the number of empty vertices
        s_list = self._find_subgraphs(free, m)
        f_a = _assign_agents(sim, sim.A, s_list, m)
        f_t = _assign_agents(sim, sim.T, s_list, m)
        if f_a != f_t:  # Alg 7 line 5: an agent split across subgraphs — not solvable at all
            return self._fail(recorder, sim.expanded_nodes)
        rels = _subgraph_priority(free, s_list, sim.T, f_a)
        closure = _closure(rels)
        if any(a == b for (a, b) in closure):  # cyclic priority — unsolvable (Proposition 2)
            return self._fail(recorder, sim.expanded_nodes)
        ok = self._solve(sim, s_list, f_a, closure)
        if not ok:
            return self._fail(recorder, sim.expanded_nodes)

        # Full-horizon space-time paths: paths[k][t] is the cell agent k occupies at global step t.
        # Agents get displaced by other agents' swap/rotate maneuvers and ride the replayed segment
        # back home — a trimmed path would misrepresent that.
        paths = [[assign[k] for assign in sim.Pi] for k in range(len(tasks))]
        cost = sum(sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in paths)
        if recorder is not None:
            for k, path in enumerate(paths):
                recorder.path_found(path, k)
            recorder.planning_finished(
                True,
                {
                    "expanded_nodes": float(sim.expanded_nodes),
                    "makespan": float(len(sim.Pi) - 1),
                    "sum_of_costs": float(cost),
                },
            )
        return MultiPlanResult(
            True, paths, float(cost), PlanStats(expanded_nodes=sim.expanded_nodes)
        )

    def _fail(self, recorder: TraceRecorder | None, expanded: int) -> MultiPlanResult:
        """Honest failure (the prioritized branch's convention): no paths, 0 cost. BFS expansions
        counted up to the failure are still reported."""
        if recorder is not None:
            recorder.planning_finished(
                False,
                {
                    "expanded_nodes": float(expanded),
                    "makespan": 0.0,
                    "sum_of_costs": 0.0,
                },
            )
        return MultiPlanResult(False, [], 0.0, PlanStats(expanded_nodes=expanded))

    # --- BFS (the paper's shortest path and clearability searches) ---------------------

    def _bfs_parent(
        self, sim: _Sim, start: Cell, blocked: frozenset[Cell] = frozenset()
    ) -> dict[Cell, Cell | None]:
        """BFS over free cells in fixed order avoiding `blocked`; the returned dict iterates in
        ENQUEUE order (== dequeue order). Every enqueued cell is dequeued once, so len(par) IS the
        expansion count this call contributes to the metric."""
        if start in blocked:
            return {}
        par: dict[Cell, Cell | None] = {start: None}
        q: deque[Cell] = deque([start])
        while q:
            c = q.popleft()
            for n in _nbrs(sim.free, c):
                if n not in par and n not in blocked:
                    par[n] = c
                    q.append(n)
        sim.expanded_nodes += len(par)
        return par

    def _path(
        self, sim: _Sim, start: Cell, goal: Cell, blocked: frozenset[Cell] = frozenset()
    ) -> list[Cell] | None:
        """Static BFS shortest path start -> goal (None when unreachable)."""
        par = self._bfs_parent(sim, start, blocked)
        if goal not in par:
            return None
        out: list[Cell] = []
        cur: Cell | None = goal
        while cur is not None:
            out.append(cur)
            cur = par[cur]
        out.reverse()
        return out

    # --- Algorithm 10: clear_vertex ------------------------------------------------------

    def _clear_vertex(
        self, sim: _Sim, segment: list[Move] | None, v: Cell, U: frozenset[Cell]
    ) -> bool:
        """Scan unoccupied cells row-major; the FIRST one whose BFS (fixed neighbor order, avoiding
        U) reaches v wins; occupants on that parent chain shift one step toward u, farthest-from-v
        occupant first so every target cell is already empty when its mover steps."""
        for u in sorted(sim.free):  # row-major over all free cells
            if _occupant(sim, u) is not None:
                continue  # only unoccupied vertices are scanned as hole candidates
            par = self._bfs_parent(sim, u, frozenset(U))
            if v not in par:
                continue
            chain: list[Cell] = []
            cur: Cell | None = v
            while cur is not None:
                chain.append(cur)
                cur = par[cur]
            chain.reverse()  # [u = x0, ..., xk = v]
            xp: Cell | None = None
            for x in chain:
                if xp is not None:
                    a = _occupant(sim, x)
                    if a is not None:
                        _move(sim, segment, a, xp)
                xp = x
            return True
        return False

    # --- Algorithm 4: push ---------------------------------------------------------------

    def _push(self, sim: _Sim, r: int, v: Cell, U: frozenset[Cell]) -> bool:
        """Walk r one step onto v. If v is occupied, clear_vertex first (its blocked set is the
        finished agents' cells plus r's own cell — a parked agent can only be shoved when the
        walker has already made it unreachable-as-a-hole); if no hole reaches v, push fails."""
        if _occupant(sim, v) is not None:
            u2 = frozenset(set(U) | {sim.A[r]})
            if not self._clear_vertex(sim, None, v, u2):
                return False
        _move(sim, None, r, v)
        return True

    # --- Algorithm 11: multipush -----------------------------------------------------------

    def _multipush(self, sim: _Sim, segment: list[Move], r0: int, s0: int, v: Cell) -> bool:
        """Bring the adjacent pair (r0, s0) to the candidate vertex v: the closer one leads along a
        static BFS path, the other follows into each vacated cell. The pinned reading skips p[0]
        (the leader's own start — clearing it literally would push the leader off its own start
        cell); a path through the follower's own cell fails the candidate exactly like Push and
        Swap's rule."""
        pr = self._path(sim, sim.A[r0], v)
        ps = self._path(sim, sim.A[s0], v)
        # closer wins; unreachable == infinite; tie -> first argument (r0).
        if pr is not None and (ps is None or len(pr) <= len(ps)):
            r, s, p = r0, s0, pr
        elif ps is not None:
            r, s, p = s0, r0, ps
        else:
            return False
        for x in p[1:]:  # pinned: skip the start cell
            vr, vs = sim.A[r], sim.A[s]
            if _occupant(sim, x) is not None:
                if not self._clear_vertex(sim, segment, x, frozenset({vr, vs})):
                    return False
            _move(sim, segment, r, x)
            _move(sim, segment, s, vr)
        return sim.A[r] == v and sim.A[s] in _nbrs(sim.free, sim.A[r])

    # --- Algorithm 12: clear (the four stages around an exchange site) --------------------

    def _clear_pair(self, sim: _Sim, segment: list[Move], rp: int, sp: int, v: Cell) -> bool:
        """Clear two neighbors of the candidate vertex v. r is the pair member standing ON v, s the
        other one; v' = A(s). Stage 1 pushes occupants of occupied neighbors away; stages 2-4 are
        the intricate cases Luna and Bekris omitted. Stages 2/3 try their candidates in fixed
        order and BREAK (fall through) once a candidate got past its first clear_vertex but failed
        the second one."""
        r = rp if sim.A[rp] == v else sp  # line 1 (defensive re-derivation by position)
        s = sp if r == rp else rp
        vp = sim.A[s]  # v' — s's cell, adjacent to v by construction

        def neigh(x: Cell) -> list[Cell]:
            return _nbrs(sim.free, x)

        # line 2 (fixed order): the free neighbors of v.
        e_set: list[Cell] = [n for n in neigh(v) if _occupant(sim, n) is None]
        if len(e_set) >= 2:  # line 3 — two free neighbors already: nothing to clear
            return True

        # Stage 1 (lines 5-9): push each occupied neighbor's occupant away; the first success seeds
        # E, a second success finishes.
        for n in [n for n in neigh(v) if n not in e_set and n != vp]:
            if self._clear_vertex(sim, segment, n, frozenset(set(e_set) | {v, vp})):
                if len(e_set) >= 1:
                    return True
                e_set.append(n)
        if not e_set:  # lines 10-11
            return False
        eps = e_set[0]  # line 12: the single empty neighbor

        # Stage 2 (lines 13-19): per candidate n, try on a copy (snapshot/rollback). The OUTER
        # clear_vertex failing means the loop simply continues to the NEXT candidate; only when n
        # cleared but eps did not (inner if fails) does line 19's break leave stage 2 for good.
        cands = [n for n in neigh(v) if n != vp and n not in e_set]
        for n in cands:
            before, pi_len = list(sim.A), len(sim.Pi)
            seg2: list[Move] = []
            try:
                ok = self._clear_vertex(sim, seg2, n, frozenset({v, vp}))
                if ok and not self._clear_vertex(sim, seg2, eps, frozenset({v, vp, n})):
                    sim.A[:] = before  # line 19: rollback (the copy was never committed)
                    del sim.Pi[pi_len:]
                    break  # inner failure stops stage 2 — fall through to stage 3
                if ok:
                    return True  # lines 17-18: commit is implicit (live state mutated)
            except AssertionError:
                pass  # impossible by construction; treated as the outer clear failing
            sim.A[:] = before
            del sim.Pi[pi_len:]

        # Stage 3 (lines 20-27): r steps into eps, s steps onto v (both on the copy); then n clears
        # (its occupant leaves on a path avoiding {v, eps}) and v' stays clearable. Same nesting as
        # stage 2: outer failure tries the next candidate, inner failure breaks out for good.
        for n in [n for n in neigh(v) if n != vp and n not in e_set]:
            before, pi_len = list(sim.A), len(sim.Pi)
            seg3: list[Move] = []
            try:
                _move(sim, seg3, r, eps)  # line 22 (r is on v — eps adjacent)
                _move(sim, seg3, s, v)
                ok = self._clear_vertex(sim, seg3, n, frozenset({v, eps}))
                if ok and not self._clear_vertex(sim, seg3, vp, frozenset({v, eps, n})):
                    sim.A[:] = before
                    del sim.Pi[pi_len:]
                    break  # line 27 (same nesting as line 19)
                if ok:
                    return True
            except AssertionError:
                pass  # impossible by construction; treated as the outer clear failing
            sim.A[:] = before
            del sim.Pi[pi_len:]

        # Stage 4 (lines 28-36): create space behind eps by walking n's occupant through v while r
        # and s step aside, then push it away from eps for good.
        try:
            if not self._clear_vertex(sim, segment, vp, frozenset({v})):
                return False
            _move(sim, segment, r, vp)
            if not self._clear_vertex(sim, segment, eps, frozenset({v, vp, sim.A[s]})):
                return False
            n2 = [n for n in neigh(v) if n != vp and n not in e_set]
            if not n2:
                return False
            n = n2[0]  # pinned: fixed-order first ("any vertex")
            t = _occupant(sim, n)
            assert t is not None
            _move(sim, segment, t, v)     # line 34 (through v ...)
            _move(sim, segment, t, eps)   # ... to eps — two single steps
            _move(sim, segment, r, v)
            _move(sim, segment, s, vp)
            return self._clear_vertex(sim, segment, eps, frozenset({v, vp, n}))
        except AssertionError:
            return False

    # --- Algorithm 13: exchange --------------------------------------------------------------

    def _exchange(self, sim: _Sim, rp: int, sp: int, v: Cell) -> None:
        """The physical exchange at the junction vertex v: r (the one ON v) steps aside to v1, s
        crosses through v onto v2, r crosses back through v onto s's old cell, s comes back from v2
        onto v. Net effect: the pair exchanged positions."""
        r = rp if sim.A[rp] == v else sp  # line 1 (re-derived by position)
        s = sp if r == rp else rp
        vs = sim.A[s]
        free_nbrs = [n for n in _nbrs(sim.free, v) if _occupant(sim, n) is None]
        assert len(free_nbrs) >= 2  # no exchange site — the caller's candidate fails
        v1, v2 = free_nbrs[0], free_nbrs[1]
        _move(sim, None, r, v1)   # line 5: r leaves v to the first free neighbor
        _move(sim, None, s, v)    # line 6 step 1 (through v)
        _move(sim, None, s, v2)   # line 6 step 2
        _move(sim, None, r, v)    # line 7 step 1 (through v)
        _move(sim, None, r, vs)   # line 7 step 2 — the exchange lands
        _move(sim, None, s, v)    # line 8: s back onto r's old cell

    # --- Algorithm 5: swap ---------------------------------------------------------------------

    def _swap(self, sim: _Sim, r0: int, s0: int, sub_cells: frozenset[Cell] | None) -> bool:
        """Exchange two adjacent agents at a degree >= 3 vertex of r's subgraph (none exists when
        f(r) is None — an unassigned agent can never swap, which is what makes tree maps honestly
        unsolvable). Candidates are evaluated nearest-first (BFS dequeue from A(r)); the exchange
        itself is Algorithm 12 + Algorithm 13. Speculative candidates roll back (assignment AND
        trace history); only the successful candidate's exchange moves survive, plus the reversed
        replay of everything before them with r/s roles exchanged — that replay carries every
        displaced bystander (finished agents included) back home."""
        if sub_cells is None:
            return False
        par = self._bfs_parent(sim, sim.A[r0])  # pinned candidate order: BFS dequeue from A(r)
        cands = [x for x in par.keys() if x in sub_cells and len(_nbrs(sim.free, x)) >= 3]
        for v in cands:
            before = list(sim.A)
            pi_len = len(sim.Pi)
            segment: list[Move] = []
            try:
                ok = self._multipush(
                    sim, segment, r0, s0, v
                ) and self._clear_pair(sim, segment, r0, s0, v)
            except AssertionError:
                ok = False
            if not ok:
                sim.A[:] = before
                del sim.Pi[pi_len:]
                continue
            # Commit is implicit (A mutated live, Pi already appended). The exchange runs on the
            # real state; then every recorded segment move replays reversed with r/s roles
            # exchanged — that replay carries displaced bystanders home.
            self._exchange(sim, r0, s0, v)
            for agent, frm, to in reversed(segment):
                who = s0 if agent == r0 else (r0 if agent == s0 else agent)
                assert sim.A[who] == to, "swap replay does not land"
                _move(sim, None, who, frm)
            return True
        return False

    # --- Algorithm 6: rotate -------------------------------------------------------------------

    def _cascade(self, sim: _Sim, c: list[Cell], e: Cell) -> None:
        """Move EVERY rider on cycle c one step forward (cycle order = c's list order, the wrap edge
        closing it). The empty slot e is filled by the rider on its predecessor, whose old cell is
        filled in turn, propagating backwards — exactly n-1 visits (every cycle cell except e
        itself), so every rider moves EXACTLY once; visiting c[j]=e itself would move the first
        moved rider a second time, which is why the loop stops at n-1."""
        n = len(c)
        j = c.index(e)
        for k in range(1, n):
            w = c[(j - k) % n]
            a = _occupant(sim, w)
            if a is not None:
                _move(sim, None, a, c[(j - k + 1) % n])

    def _rotate(
        self,
        sim: _Sim,
        c: list[Cell],
        f: list[int | None],
        s_list: list[frozenset[Cell]],
    ) -> bool:
        """Move every agent on cycle c one step forward. Phase 1 (lines 1-4): an empty cell exists —
        cascade from it. Phase 2 (lines 5-16, fully occupied): push the rider at the first clearable
        cycle vertex off the cycle, step its predecessor up onto the vacated slot, SWAP the pair
        (the rotate borrows swap's junction machinery), cascade from the new empty cell, then replay
        the push-off reversed with roles exchanged — walking the pushed-off rider back onto the
        vacated slot and completing the rotation."""
        # Phase 1: first empty cycle vertex in c order.
        for v in c:
            if _occupant(sim, v) is None:
                self._cascade(sim, c, v)
                return True
        # Phase 2: first clearable cycle vertex in c order.
        n = len(c)
        for idx, v in enumerate(c):
            r = _occupant(sim, v)
            assert r is not None  # phase 2 only runs on a fully occupied cycle
            seg: list[Move] = []
            try:
                ok = self._clear_vertex(sim, seg, v, frozenset(set(c) - {v}))
            except AssertionError:
                ok = False
            if not ok:
                continue
            vp = c[(idx - 1) % n]  # predecessor in cycle order (line 10)
            rp_ = _occupant(sim, vp)
            assert rp_ is not None  # fully occupied; the push-off only vacated v itself
            _move(sim, None, rp_, v)  # line 12: predecessor steps into the vacated cell
            fr = f[r]
            cells = s_list[fr] if fr is not None else None
            try:
                ok = self._swap(sim, r, rp_, cells)
            except AssertionError:
                ok = False
            if not ok:
                return False  # honest failure propagates out of solve
            self._cascade(sim, c, vp)  # line 14: everyone forward starting at the new empty cell
            for agent, frm, to in reversed(seg):  # line 15: replay reversed with roles exchanged
                who = rp_ if agent == r else (r if agent == rp_ else agent)
                assert sim.A[who] == to, "rotate replay does not land"
                _move(sim, None, who, frm)
            return True
        return False

    # --- Algorithm 8: solve -----------------------------------------------------------------

    def _solve(
        self,
        sim: _Sim,
        s_list: list[frozenset[Cell]],
        f: list[int | None],
        closure: set[tuple[int, int]],
    ) -> bool:
        """Alg 8 verbatim. F = finished agents; q = the trail of resolving cells; r = the agent
        being walked (None between walks). is_polygon gates whether finished agents' cells block
        r's path BFS."""
        free = sim.free
        n_agents = len(sim.A)
        is_polygon = all(len(_nbrs(free, c)) == 2 for c in free)  # line 5
        F: set[int] = set()
        q: list[Cell] = []
        r: int | None = None
        while F != set(range(n_agents)):  # line 6
            if r is None:  # line 7
                r = _next_agent(set(range(n_agents)) - F, f, closure)  # line 8
            blocked = frozenset(sim.A[i] for i in F) if is_polygon else frozenset()  # lines 9-12
            p = self._path(sim, sim.A[r], sim.T[r], blocked)
            if p is None:
                return False
            q.append(sim.A[r])  # line 13
            try:
                while sim.A[r] != sim.T[r]:  # line 14
                    v = p[p.index(sim.A[r]) + 1]  # line 15 (A(r) is on p, never at its end here)
                    if v in q:  # line 16 — cycle of resolving agents detected
                        c = q[q.index(v):]  # get_cycle pinned: first occurrence to the end
                        q = q[: q.index(v)]
                        if not self._rotate(sim, c, f, s_list):
                            return False
                    else:
                        if not self._push(sim, r, v, frozenset(sim.A[i] for i in F)):  # line 21
                            occ = _occupant(sim, v)
                            assert occ is not None
                            fr = f[r]
                            cells = s_list[fr] if fr is not None else None
                            if not self._swap(sim, r, occ, cells):  # line 22
                                return False
                    q.append(v)  # line 23 (UNCONDITIONAL — pinned)
            except AssertionError:
                return False
            F.add(r)  # line 24
            r = None  # line 25
            try:
                while len(q) > 0:  # line 26 — shrink q, returning resolving agents home
                    v = q[-1]
                    s = _occupant(sim, v)
                    if s is not None and s in F and v != sim.T[s]:
                        rn = _occupant(sim, sim.T[s])  # line 30
                        if rn is None:
                            _move(sim, None, s, sim.T[s])  # line 32 (single step — adjacency holds)
                        else:
                            r = rn  # line 34: break inner loop, continue outer WITH q kept
                            break
                    q.pop()  # line 35 (runs on every non-handoff branch)
            except AssertionError:
                return False
        return True

    # --- Algorithms 1/2/3: decomposition, assignment, priority ---------------------------------

    def _find_subgraphs(self, free: frozenset[Cell], m: int) -> list[frozenset[Cell]]:
        """Alg 1: nontrivial biconnected components (Hopcroft-Tarjan DFS, roots row-major, neighbors
        fixed order), singleton subgraphs for uncovered degree >= 3 cells in row-major order, then
        the merge loop over pairs by index order (the first pair whose set distance <= m-2 merges;
        the path is pinned by a multi-source BFS seeded from Si's cells row-major)."""
        comps: list[frozenset[Cell]] = []
        disc: dict[Cell, int] = {}
        low: dict[Cell, int] = {}
        par: dict[Cell, Cell | None] = {}
        t = 0
        for root in sorted(free):  # row-major DFS roots
            if root in disc:
                continue
            t += 1
            disc[root] = low[root] = t
            par[root] = None
            edge_stack: list[tuple[Cell, Cell]] = []
            stack: list[list] = [[root, iter(_nbrs(free, root))]]
            while stack:
                u, it = stack[-1][0], stack[-1][1]
                nxt = next(it, None)
                if nxt is None:
                    stack.pop()
                    if not stack:
                        break  # this DFS tree is complete
                    p = par[u]
                    assert p is not None
                    if low[u] >= disc[p]:
                        comp: list[Cell] = []
                        while True:
                            ea, eb = edge_stack.pop()
                            for x in (ea, eb):
                                if x not in comp:
                                    comp.append(x)
                            if (ea, eb) == (p, u) or (eb, ea) == (p, u):
                                break
                        if len(comp) >= 3:  # nontrivial = a whole cycle's worth of vertices
                            comps.append(frozenset(comp))
                    else:
                        low[p] = min(low[p], low[u])
                    continue
                w = nxt
                if w not in disc:
                    par[w] = u
                    t += 1
                    disc[w] = low[w] = t
                    edge_stack.append((u, w))
                    stack.append([w, iter(_nbrs(free, w))])
                elif w != par[u] and disc[w] < disc[u]:
                    edge_stack.append((u, w))
                    low[u] = min(low[u], disc[w])
        covered: set[Cell] = set().union(*comps) if comps else set()
        for v in sorted(free):  # row-major singletons for uncovered join vertices
            if len(_nbrs(free, v)) >= 3 and v not in covered:
                comps.append(frozenset({v}))
        changed = True
        while changed:  # merge loop (Alg 1 lines 3-5), pinned scan order + restart
            changed = False
            for i in range(len(comps)):
                merged_found = None
                for j in range(i + 1, len(comps)):
                    hit = _merge_path(free, comps[i], comps[j], m)
                    if hit is not None:
                        merged_found = (j, hit)
                        break
                if merged_found is not None:
                    j, path_cells = merged_found
                    merged = frozenset(set(comps[i]) | set(comps[j]) | set(path_cells))
                    comps = comps[:i] + [merged] + comps[i + 1 : j] + comps[j + 1 :]
                    changed = True
                    break  # restart the scan from the top (pinned)
        return comps


def _merge_path(
    free: frozenset[Cell], si: frozenset[Cell], sj: frozenset[Cell], m: int
) -> list[Cell] | None:
    """Alg 1 merge test: multi-source BFS seeded from Si's cells in row-major order (fixed neighbor
    order); the FIRST Sj cell dequeued pins the pair and their path. Returns the path cells when
    that distance <= m-2, else None."""
    par: dict[Cell, Cell | None] = {}
    depth: dict[Cell, int] = {}
    q: deque[Cell] = deque()
    for seed in sorted(si):  # row-major seeds
        if seed not in par:
            par[seed] = None
            depth[seed] = 0
            q.append(seed)
    while q:
        c = q.popleft()
        d = depth[c]
        if c in sj:  # first Sj cell in BFS order — the pinned pair, decide now
            if d <= m - 2:
                path: list[Cell] = []
                cur: Cell | None = c
                while cur is not None:
                    path.append(cur)
                    cur = par[cur]
                return path
            return None
        for n in _nbrs(free, c):
            if n not in par:
                par[n] = c
                depth[n] = d + 1
                q.append(n)
    return None


def _assign_agents(
    sim: _Sim, x: list[Cell], s_list: list[frozenset[Cell]], m: int
) -> list[int | None]:
    """Alg 2: which subgraph confines each agent under assignment X (A or T). Later assignments
    overwrite earlier ones (pinned)."""
    f: list[int | None] = [None] * len(x)

    def x_inv(c: Cell) -> int | None:
        return x.index(c) if c in x else None

    for si, vi_set in enumerate(s_list):
        for v in sorted(vi_set):  # row-major over Vi
            us = [u for u in _nbrs(sim.free, v) if u not in vi_set]
            if not us:  # interior vertex (lines 11-12)
                ai = x_inv(v)
                if ai is not None:
                    f[ai] = si
                continue
            # m'' (line 5): unoccupied cells reachable from Vi\{v} in G[V\{v}] — a count.
            reach: set[Cell] = set()
            dq: deque[Cell] = deque()
            for seed in sorted(c for c in vi_set if c != v):
                if seed not in reach:
                    reach.add(seed)
                    dq.append(seed)
            while dq:
                c = dq.popleft()
                for n in _nbrs(sim.free, c):
                    if n != v and n not in reach:  # vertex v removed from the graph
                        reach.add(n)
                        dq.append(n)
            m_dprime = sum(1 for c in reach if x_inv(c) is None)
            for u in us:  # fixed neighbor order (line 6)
                # m' (line 7): unoccupied cells reachable from v with the EDGE (u,v) removed.
                reach2: set[Cell] = {v}
                dq2: deque[Cell] = deque([v])
                while dq2:
                    c = dq2.popleft()
                    for n in _nbrs(sim.free, c):
                        if (c == v and n == u) or (n == v and c == u):
                            continue  # the removed edge
                        if n not in reach2:
                            reach2.add(n)
                            dq2.append(n)
                m_prime = sum(1 for c in reach2 if x_inv(c) is None)
                ai = x_inv(v)
                # Lines 8-9: the gate gates ONLY the junction occupant. (Verified against Fig 11(a):
                # a4 IS assigned "in line 10 since m'=3" while the junction cell was unoccupied —
                # line 10 must run ungated.)
                if ((m_prime >= 1 and m_prime < m) or m_dprime >= 1) and ai is not None:
                    f[ai] = si
                # Line 10 — UNCONDITIONAL inside the u-loop: walk the plank from u away from v
                # (through cells in no Vj, INCLUDING the stopping cell, e.g. the far subgraph's join
                # vertex — Fig 11(b) assigns a8 on s5); assign the first max(0, m'-1) agents
                # encountered, in encounter order.
                count = 0
                for w in _plank_walk(sim.free, v, u, s_list):
                    if count >= max(0, m_prime - 1):
                        break
                    ai2 = x_inv(w)
                    if ai2 is not None:
                        f[ai2] = si
                        count += 1
    return f


def _plank_walk(
    free: frozenset[Cell], v: Cell, u: Cell, s_list: list[frozenset[Cell]]
) -> list[Cell]:
    """The unique maximal path from u AWAY from v (v is explicitly excluded at the first step; other
    cells belonging to no subgraph have degree <= 2, so the continuation is unique). The walk
    INCLUDES the stopping cell — the join vertex of the far subgraph (Fig 11(b): a8 on s5 assigned
    via line 10) — and ends there, or at a dead end."""
    covered = set().union(*s_list) if s_list else set()
    walk = [u]
    cur = u
    while cur not in covered:
        nxts = [n for n in _nbrs(free, cur) if n not in walk and n != v]
        nxt = nxts[0] if nxts else None  # unique continuation (non-covered cells have degree <= 2)
        if nxt is None:
            return walk  # dead-end plank
        walk.append(nxt)
        cur = nxt
    return walk


def _plank_to_subgraph(
    free: frozenset[Cell], v: Cell, u: Cell, s_list: list[frozenset[Cell]]
) -> tuple[int | None, list[Cell]]:
    """Walk from u away from v; return (sj_index, cells INCLUDING the stopping cell that belongs to
    Sj), or (None, []) at a dead end."""
    walk = _plank_walk(free, v, u, s_list)
    last = walk[-1]
    for i, s in enumerate(s_list):
        if last in s:
            return i, walk
    return None, []


def _plank_relation(
    free: frozenset[Cell],
    s_list: list[frozenset[Cell]],
    t: list[Cell],
    f: list[int | None],
    si: int,
    vi_set: frozenset[Cell],
    v: Cell,
) -> int | None:
    """Alg 3 lines 3-10 for one (Si, v): walk every plank leading away from v; the first
    goal-bearing cell on a walk whose occupant-goal agent IS assigned to some subgraph Sj yields
    that assignment (the relation); an unassigned-goal cell keeps the walk going (line 10), a
    no-goal or Si-assigned goal ends that walk. Dead-end planks yield nothing (line 4)."""
    for u in [u for u in _nbrs(free, v) if u not in vi_set]:
        sj, plank = _plank_to_subgraph(free, v, u, s_list)
        if sj is None:
            continue  # dead-end plank (line 4) — no relation can come from this u
        walk = [v] + plank
        hit: int | None = None
        for vp in walk:
            r = t.index(vp) if vp in t else None
            if r is None:
                break  # nobody's goal here — the while condition (line 6) ends
            fr = f[r]
            if fr == si:
                break  # one of "our" agents — no relation, next u
            if fr is None:
                continue  # unassigned-goal agent: keep walking (line 10)
            hit = fr
            break
        if hit is not None:
            return hit
    return None


def _subgraph_priority(
    free: frozenset[Cell], s_list: list[frozenset[Cell]], t: list[Cell], f: list[int | None]
) -> list[tuple[int, int]]:
    """Alg 3: Si < Sj means Si is planned FIRST (its agents get higher priority). For every
    subgraph vertex v in row-major order: the first plank of Si whose walk hits a goal assigned to
    another subgraph adds (si, f(r)) — and line 9 then moves on to the NEXT v (remaining u's of
    this v are skipped)."""
    rels: list[tuple[int, int]] = []
    for si, vi_set in enumerate(s_list):
        for v in sorted(vi_set):  # row-major
            hit = _plank_relation(free, s_list, t, f, si, vi_set, v)
            if hit is not None:
                rels.append((si, hit))
    return rels


def _closure(rels: list[tuple[int, int]]) -> set[tuple[int, int]]:
    """Transitive closure of the priority relation; a self-relation means unsolvable
    (Proposition 2)."""
    closure = set(rels)
    changed = True
    while changed:
        changed = False
        snap = list(closure)
        for (a, b) in snap:
            for (c2, d) in snap:
                if b == c2 and (a, d) not in closure:
                    closure.add((a, d))
                    changed = True
    return closure


def _next_agent(
    unfinished: set[int], f: list[int | None], closure: set[tuple[int, int]]
) -> int:
    """Pinned next-agent rule (the paper leaves it open): assigned agents go first — candidates are
    unfinished agents whose subgraph has no ACTIVE predecessor (active = subgraphs with unfinished
    members; transitive closure of <); lowest index among candidates wins. With no assigned
    unfinished agent left, the lowest-index unassigned one goes."""
    assigned_unfinished = [i for i in sorted(unfinished) if f[i] is not None]
    if assigned_unfinished:
        active = {f[i] for i in unfinished if f[i] is not None}
        cands = []
        for i in assigned_unfinished:
            si = f[i]
            preds = [a for (a, b) in closure if a != b and b == si and a in active]
            if not preds:
                cands.append(i)
        return min(cands)  # pinned lowest index
    unassigned = [i for i in sorted(unfinished) if f[i] is None]
    return min(unassigned)
