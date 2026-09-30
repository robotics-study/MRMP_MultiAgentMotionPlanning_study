"""Push and Swap — Luna & Bekris (IJCAI 2011): decentralized plan-and-repair.

The completion of the priority branch: agents are still processed one at a time in
index order, but earlier agents are NOT frozen in place. The set U records which
agents have already reached their goals; PUSH walks agent r along its static BFS
shortest path, chain-pushing any occupant blocking that path into the nearest
reachable empty cell (a BFS from the blocker's cell whose blocked set is exactly
{A[r]} ∪ U — so a parked agent can never be shoved), and when no push can clear
the way SWAP exchanges r with the blocker instead, restoring every displaced
agent to its old cell by replaying the maneuver reversed.

Grid reading of the paper's primitives (the paper works on general graphs; this
repository only ever hands it a 4-connected grid):

- SHORTEST_PATH is BFS over free cells in the map layer's fixed neighbor order
  (up/down/left/right — the wait self-loop is not an action here, so it is
  filtered out). The parent chain doubles as the push chain.
- A swap needs a free 2x2 block: r@v and s@w1 exchange iff some w2 in N(v)\\{w1}
  and w4 in N(w1)\\{v} ∩ N(w2) are clearable (an occupant is cleared by stepping it
  into its own first free neighbor, fixed order). Grid graphs are bipartite — no
  triangle — so the paper's Figure-1 T-junction sketch cannot host a swap; only a
  2x2 block can. This is also why Case 3 of the paper's CLEAR discussion is
  unnecessary rather than merely skipped.
- The paper leaves POP() order over swap vertices unspecified; pinned here to BFS
  dequeue order from A[r] (nearest candidate first, deterministic across
  languages). A candidate fails iff s sits on path(A[r], v) — the composite pair
  can never walk past its own member — or if CLEAR cannot free the block. Freeing
  a later w4 can REFILL an already-cleared w2 (the cleared occupant steps into
  its first free neighbor, which may be exactly that w2 — reachable from three or
  more agents); that attempt fails like any other clear failure instead of moving
  into the refilled cell.
- Failed candidates are speculative: their moves land on a local segment and are
  rolled back (assignment AND trace history restored); only a successful swap's
  EXECUTE_SWAP moves join the solution directly, followed by the forward segment
  replayed reversed with r/s roles exchanged — which is also what carries every
  displaced U agent back onto its goal.

Completeness: the paper claims completeness for n <= |V| - 2 agents on any graph.
On grids that claim is NOT literally true and this implementation does not fake
it: a parked (U) agent standing between r and T[r] on a tree can only be moved by
a swap, and a tree has no 2x2 block — so e.g. the corridor head-on scenario fails
honestly even though a human would duck the blocker into the pocket first. The
pinned semantics are the faithful algorithm, not an omniscient repair oracle.

Determinism contract: every tie-break is pinned (neighbor order up/down/left/
right everywhere, BFS enqueue order for hole selection and candidate iteration),
so Python/C++/TS runs produce byte-identical assignments and traces.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from ..core.capabilities import Capability, DiscreteSpace
from ..core.planner import MultiAgentPlanner
from ..core.trace import TraceRecorder
from ..core.types import AgentTask, Cell, MultiPlanResult, PlanStats

# 4-connected neighbor order — the SAME fixed convention as the map layer's move
# set (maps/occupancy_grid.py). The wait self-loop there is not an action of this
# algorithm: every recorded assignment change is a real move.
_MOVES_4 = ((-1, 0), (1, 0), (0, -1), (0, 1))

# One executed move (agent, from, to) — the unit a swap records before rollback.
Move = tuple[int, Cell, Cell]


def _nbrs(free: frozenset[Cell], c: Cell) -> list[Cell]:
    """Free 4-neighbors of c in the fixed order (membership test on the cell set —
    out-of-bounds cells are not in it by construction)."""
    return [(c[0] + dr, c[1] + dc) for dr, dc in _MOVES_4 if (c[0] + dr, c[1] + dc) in free]


@dataclass
class _Sim:
    """Mutable simulation state during planning. A is the current assignment (one
    cell per agent), T the target assignment, U the goal cells of agents already
    finished, Pi the solution path — a list of assignments where every executed
    move appends the new one. expanded_nodes counts BFS node expansions (dequeues)
    across every shortest-path/hole-finding call: this algorithm has no search
    frontier of its own, so this is what the expanded_nodes metric reports."""

    free: frozenset[Cell]
    A: list[Cell]
    T: list[Cell]
    U: set[Cell] = field(default_factory=set)
    Pi: list[tuple[Cell, ...]] = field(default_factory=list)
    expanded_nodes: int = 0


def _occupant(sim: _Sim, c: Cell) -> int | None:
    """Index of the agent occupying cell c (the assignment is injective), or None."""
    for i, a in enumerate(sim.A):
        if a == c:
            return i
    return None


def _move(
    sim: _Sim, segment: list[Move] | None, agent: int, to: Cell
) -> None:
    """One action: `agent` steps from its cell to the empty free cell `to`, the new
    assignment joins Pi — and when inside a speculative swap candidate, also into
    the local segment so the whole attempt can be rolled back."""
    frm = sim.A[agent]
    # Invariants every call site constructs (asserted, not recovered from):
    # a move actually moves, the target is free, and nothing occupies it yet.
    assert frm != to and to in sim.free and _occupant(sim, to) is None
    sim.A[agent] = to
    sim.Pi.append(tuple(sim.A))
    if segment is not None:
        segment.append((agent, frm, to))


class PushAndSwap(MultiAgentPlanner):
    @property
    def name(self) -> str:
        return "push_and_swap"

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
        # The paper's assignment is injective by definition (unique starts AND
        # unique targets) and every involved cell must be passable. A violating
        # input is not an instance of this problem — report it honestly, no plan.
        if (
            len(set(starts)) < len(starts)
            or len(set(goals)) < len(goals)
            or any(c not in free for c in [*starts, *goals])
        ):
            return self._fail(recorder, 0)

        sim = _Sim(free=free, A=starts, T=goals, Pi=[tuple(starts)])
        # Algorithm 1: agents in index order; PUSH first, SWAP only when pushing
        # cannot clear the way, honest failure when neither works.
        for r in range(len(tasks)):
            while sim.A[r] != sim.T[r]:
                if not self._push(sim, r) and not self._swap(sim, r):
                    return self._fail(recorder, sim.expanded_nodes)
            sim.U.add(sim.T[r])

        # Full-horizon space-time paths: paths[k][t] is the cell agent k occupies
        # at global step t. U agents get displaced mid-later-agent swaps and ride
        # the replayed segment back home — a trimmed path would misrepresent that.
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
        """Honest failure (the prioritized branch's convention): no paths, 0 cost.
        BFS expansions counted up to the failure are still reported."""
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

    # --- BFS (the paper's SHORTEST_PATH and hole-finding) ---------------------
    def _bfs_parent(
        self, sim: _Sim, start: Cell, blocked: frozenset[Cell] = frozenset()
    ) -> dict[Cell, Cell | None]:
        """BFS over free cells in fixed neighbor order avoiding `blocked`. The
        returned parent map iterates in ENQUEUE order — which is also dequeue
        order — so "first empty cell" scans and candidate iteration are pinned.
        Every enqueued cell is dequeued exactly once, so len(par) IS the number of
        node expansions this call contributes to the metric."""
        if start in blocked:
            return {}
        par: dict[Cell, Cell | None] = {start: None}
        queue: deque[Cell] = deque([start])
        while queue:
            c = queue.popleft()
            for n in _nbrs(sim.free, c):
                if n not in par and n not in blocked:
                    par[n] = c
                    queue.append(n)
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

    # --- Algorithm 2: PUSH ------------------------------------------------------
    def _push(self, sim: _Sim, r: int) -> bool:
        """Walk r along the static shortest path p* from A[r] to T[r], moving it
        through empty cells; when the next cell v is occupied, chain-push that
        occupant (and whoever else stands between it and the nearest reachable
        hole) one step toward the hole. The hole-finding BFS treats {A[r]} ∪ U as
        walls: r's own cell can never be a hole, and parked agents are untouchable."""
        if sim.A[r] == sim.T[r]:
            return True  # vacuously done (defensive — callers loop on A[r] != T[r])
        p_star = self._path(sim, sim.A[r], sim.T[r])
        if p_star is None:
            return False  # statically unreachable goal -> honest failure upstream
        idx = 1
        while sim.A[r] != sim.T[r]:
            # r stands on p_star[idx-1] != T[r] == p_star[-1], so the next cell exists.
            v = p_star[idx]
            if _occupant(sim, v) is None:
                _move(sim, None, r, v)
                idx += 1
                continue
            # Blocked at v. A parked (U) blocker marks itself -> BFS cannot even
            # start there; otherwise the first empty cell in enqueue order is the
            # hole and the chain shifts toward it farthest-from-r occupant first.
            if not self._chain_push(sim, None, v, frozenset({sim.A[r]} | sim.U)):
                return False
        return True

    def _chain_push(
        self,
        sim: _Sim,
        segment: list[Move] | None,
        blocker: Cell,
        blocked: frozenset[Cell],
    ) -> bool:
        """Shared by PUSH and MULTIPUSH (paper Algorithm 2 lines 9-22): BFS from the
        blocker cell avoiding `blocked`; the first EMPTY cell in enqueue/dequeue
        order is the hole; occupants along the blocker->hole parent chain shift one
        step toward the hole, farthest-from-r occupant first."""
        par = self._bfs_parent(sim, blocker, blocked)
        v_empty: Cell | None = None
        for c in par:  # dict order == BFS enqueue order == dequeue order
            if _occupant(sim, c) is None:
                v_empty = c
                break
        if v_empty is None:
            return False
        chain: list[Cell] = []
        cur: Cell | None = v_empty
        while cur is not None:
            chain.append(cur)
            cur = par[cur]
        chain.reverse()  # blocker cell first, hole last
        # Farthest-from-r occupant moves first so each target cell is already empty.
        for k in range(len(chain) - 2, -1, -1):
            a = _occupant(sim, chain[k])
            if a is not None:
                _move(sim, segment, a, chain[k + 1])
        return True

    # --- Algorithm 3: SWAP --------------------------------------------------------
    def _swap(self, sim: _Sim, r: int) -> bool:
        """Exchange r with the agent s blocking r's first step on its shortest
        path. The paper iterates ALL degree-≥3 vertices (POP() order unspecified);
        pinned here to BFS-dequeue order from A[r] — nearest candidate first. On a
        grid a swap physically needs a free 2x2 block, which is exactly what the
        CLEAR check below decides."""
        if sim.A[r] == sim.T[r]:
            return True  # vacuously done (defensive)
        p_star = self._path(sim, sim.A[r], sim.T[r])
        if p_star is None or len(p_star) < 2:
            return False
        s = _occupant(sim, p_star[1])
        if s is None:
            return False  # nothing blocks r's first step — push would not have failed
        for v in list(self._bfs_parent(sim, sim.A[r]).keys()):
            before = list(sim.A)
            pi_len = len(sim.Pi)
            segment: list[Move] = []
            if not self._multipush(sim, segment, r, s, v):
                sim.A[:] = before
                del sim.Pi[pi_len:]
                continue
            if not self._clear_and_swap(sim, segment, r, s):
                sim.A[:] = before
                del sim.Pi[pi_len:]
                continue
            # Success. The forward segment already landed on Pi move by move; the
            # EXECUTE_SWAP moves went straight to Pi too (they are NOT part of the
            # reversible segment). Replaying the segment reversed with r/s roles
            # exchanged restores every displaced agent — including U agents — home.
            for agent, frm, to in reversed(segment):
                who = s if agent == r else (r if agent == s else agent)
                assert sim.A[who] == to  # the reversal lands exactly by construction
                _move(sim, None, who, frm)
            if sim.T[s] in sim.U:
                return self._resolve(sim, r, s)
            return True
        return False

    def _multipush(
        self, sim: _Sim, segment: list[Move], r: int, s: int, v: Cell
    ) -> bool:
        """Composite pair walk (the paper's MULTIPUSH): r leads along p =
        SHORTEST_PATH(A[r], v), s follows into each vacated cell, third-party
        occupants of the path are chain-pushed away — U is IGNORED here (the swap
        may disturb parked agents; the replay restores them) and both pair cells
        count as blocked in that BFS."""
        p = self._path(sim, sim.A[r], v)  # p starts at A[lead], so r always leads
        if p is None or len(p) < 2:
            return False
        if sim.A[s] in p:
            # s sits on the path ahead of r: the composite can never walk to v
            # without dragging s past r, so this candidate simply fails.
            return False
        lead, follow = r, s
        if sim.A[follow] not in _nbrs(sim.free, sim.A[lead]):
            return False  # defensive: by construction the pair IS adjacent
        i = 0  # p starts at A[lead], so lead's index on p is 0
        while sim.A[lead] != v:
            w = p[i + 1]
            a = _occupant(sim, w)
            if a is not None and a != follow:
                if not self._chain_push(
                    sim, segment, w, frozenset({sim.A[lead], sim.A[follow]})
                ):
                    return False
            _move(sim, segment, lead, w)
            i += 1
            vacated = p[i - 1]  # lead just left it — adjacent to follow by construction
            if _occupant(sim, vacated) is None:
                _move(sim, segment, follow, vacated)
        return sim.A[r] == v and sim.A[s] in _nbrs(sim.free, sim.A[r])

    def _clear_and_swap(self, sim: _Sim, segment: list[Move], r: int, s: int) -> bool:
        """CLEAR + EXECUTE_SWAP fused (the paper's cases 1/2): r sits on v = A[r],
        s on its neighbor w1 = A[s]. The exchange exists iff some w2 in N(v)\\{w1}
        and w4 in N(w1)\\{v} ∩ N(w2) are clearable — an occupant is cleared by
        stepping it into its own first free neighbor (fixed order; no free neighbor
        means this candidate fails). Then the four moves r:v→w2, s:w1→v, r:w2→w4,
        r:w4→w1 exchange the pair. They go straight onto Pi — only the segment
        before them is reversible.

        Clearing w4 can REFILL an already-cleared w2: the cleared occupant steps
        into its own first free neighbor, and that cell may be exactly the w2 that
        was just vacated (3+ agents make this reachable). The exchange then cannot
        execute — pinned behavior: this attempt fails like any other clear failure
        (continue to the next candidate; if every attempt fails the whole vertex
        candidate fails and _swap rolls the segment back), never an assert."""
        v = sim.A[r]
        w1 = sim.A[s]
        nbrs_v = _nbrs(sim.free, v)
        if w1 not in nbrs_v:
            return False
        for w2 in [w for w in nbrs_v if w != w1]:
            if _occupant(sim, w2) is not None and not self._clear_cell(sim, segment, w2):
                continue
            nbrs_w1 = _nbrs(sim.free, w1)
            for w4 in [w for w in nbrs_w1 if w != v and w in _nbrs(sim.free, w2)]:
                if _occupant(sim, w4) is not None and not self._clear_cell(sim, segment, w4):
                    continue
                if _occupant(sim, w2) is not None:
                    # clearing w4 just re-occupied w2 — this attempt cannot execute.
                    continue
                # EXECUTE_SWAP: r vacates v for s, then rounds the block into w1.
                _move(sim, None, r, w2)
                _move(sim, None, s, v)
                _move(sim, None, r, w4)
                _move(sim, None, r, w1)
                return True
        return False

    def _clear_cell(self, sim: _Sim, segment: list[Move], x: Cell) -> bool:
        """CLEAR one vertex: its occupant steps into its own first free neighbor in
        fixed order; no free neighbor means this swap candidate fails."""
        for n in _nbrs(sim.free, x):
            if _occupant(sim, n) is None:
                a = _occupant(sim, x)
                assert a is not None  # callers only ever call this on occupied cells
                _move(sim, segment, a, n)
                return True
        return False

    def _resolve(self, sim: _Sim, r: int, s: int) -> bool:
        """The swapped-away agent s was already at its goal (T[s] in U). Paper cases
        1 and 2: push/swap r until it vacates T[s], then send s home the same way.
        A failing swap here invalidates the original swap — honest failure."""
        while sim.A[r] == sim.T[s]:
            if not self._push(sim, r) and not self._swap(sim, r):
                return False
        while sim.A[s] != sim.T[s]:
            if not self._push(sim, s) and not self._swap(sim, s):
                return False
        return True
