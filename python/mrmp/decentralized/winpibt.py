"""winPIBT — Okumura, Tamura & Défago (IJCAI 2019, arXiv:1905.10149).

The decentralized branch's second member: PIBT generalized along the TIME WINDOW.
PIBT negotiates one cell per timestep; winPIBT lets every agent hold a whole
provisional space-time path Π_i and extend it w steps ahead at a time, securing the
steps one by one in priority order. The paper's Algorithm 1/2, mirrored here line for
line (hard mode — §4.2.2's iterative-use relaxation is NOT applied: goals stay fixed,
an agent can be pushed off its goal and re-travels to it; termination is still every
agent on its goal at once):

* Priorities are PIBT's exactly: standing on the goal resets p_i to ε_i, travelling
  increments it by one per round; agents extend their paths in decreasing priority.
* Each round t (Algorithm 2): agent i extends only while ℓ_i ≤ t. The highest-priority
  such agent gets α = t + w; every later one is capped at κ, the running minimum of
  what higher priorities already secured — lower priorities may never reserve beyond
  what higher ones covered. That cap is why a big w behaves like prioritized planning.
* A call winpibt(i, α) (Algorithm 1): β = max(α, every registered length) fixed at
  entry; compute an ideal path from the agent's secured end to its goal — static
  parent-chain route padded with waits at the goal, else a pinned space-time search —
  valid against every VISIBLE reservation: entering cell v at step τ is invalid iff
  some other agent's path occupies v at any step in [τ, min(β, ℓ_j)] (a secured cell
  stays poison until its reservation visibly ends) or swaps with a fully-secured move.
  No valid walk → copeStuck: pin waits to α and report invalid. Register the whole
  extension at once but SECURE it step by step; securing step t forces anyone parked
  on that cell (retroactively, one step per inheritance call) out of it first. A failed
  inheritance retracts the unsecured suffix and replans from the last secured step.
* Visibility is by secured length ℓ_j, not registration: a candidate step at τ only
  sees what agents j secured up to their own ℓ_j. That is the disentangled condition
  (paper Def 3.4) doing its job — beyond ℓ_j nothing is decided yet, and whoever
  still occupies a cell on the incoming path gets pulled forward by inheritance before
  that step is ever secured. Rotations need no special case: an agent whose own path
  already covers the contested step simply is not an inheritance target (ℓ_j ≥ τ).
* w = 1 collapses to PIBT's per-cell negotiation; large w approaches prioritized
  planning. The window is this branch's coupling axis as a single parameter.

Pinned degrees of freedom (the paper leaves these free or random; identical here in
Python, C++ and TS by construction): ε_i = (k-1-i)/k with agent 0 highest; the static
route's BFS parent chain over fixed neighbor order up/down/left/right, padded with
waits at the goal / truncated to the horizon; the fallback space-time search — A* over
(cell, step) states, f = step + static BFS distance to the goal, ties broken by later
step first then insertion order, expansion in fixed neighbor order with wait last, and
at every popped state a fresh try at the padded static route (success = surviving to β);
inheritance targets picked in agent-index order. The paper's random candidate shuffle
has no analogue here — nothing is random.

The run stops when every agent simultaneously stands on its goal (makespan = that
step) or at the max_steps budget — an honest "no solution found within budget", never
a proof of unsolvability. Reachability (paper Theorem 4.3): on dodgeable graphs
(every adjacent pair of free cells on a simple cycle) and finite window, every agent
reaches its goal in finite time — the same condition PIBT needed, for the same reason:
an edge on no cycle gives an agent with a reserved cell nowhere to retroactively leave.

Paths are full-horizon like the PIBT page: paths[k][t] is agent k's cell at step t up
to the makespan; sum_of_costs counts actual moves (waits cost nothing); expanded_nodes
counts winpibt() decision-procedure invocations (top-level + inherited) — this
algorithm has no search frontier of its own, only these calls.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field
from itertools import count

from ..core.capabilities import Capability, DiscreteSpace
from ..core.planner import MultiAgentPlanner
from ..core.trace import TraceRecorder
from ..core.types import AgentTask, Cell, MultiPlanResult, PlanStats

# 4-connected neighbor order — the SAME fixed convention as the map layer's move set.
# The wait action is not a neighbor: staying put is the current cell itself, which the
# ideal path expresses by repeating its cell (the paper's padding with waits).
_MOVES_4 = ((-1, 0), (1, 0), (0, -1), (0, 1))

# Distance from an unreachable goal: sorts after every reachable cell; among equals
# the fixed row-major order still decides. Same convention as the PIBT page.
_INF = float("inf")


def _nbrs(free: frozenset[Cell], c: Cell) -> list[Cell]:
    """Free 4-neighbors of c in the fixed order (out-of-bounds cells are not in the
    free set by construction, so membership alone bounds the grid)."""
    return [(c[0] + dr, c[1] + dc) for dr, dc in _MOVES_4 if (c[0] + dr, c[1] + dc) in free]


@dataclass
class _Sim:
    """Mutable state of one run. paths[i][t] is agent i's provisional path — cells
    registered up to ℓ_i (sim.ell[i]); positions at time t are paths[i][t]. eps/eta/p
    carry PIBT's priorities; calls counts every winpibt() invocation (the
    expanded_nodes metric); dists[k] is the static BFS distance table from goal k."""

    free: frozenset[Cell]
    goals: list[Cell]
    dists: list[dict[Cell, int]]
    paths: list[list[Cell]]
    ell: list[int] = field(default_factory=list)
    eps: list[float] = field(default_factory=list)
    calls: int = 0


class Winpibt(MultiAgentPlanner):
    @property
    def name(self) -> str:
        return "winpibt"

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
        # The paper's instance is well-formed by definition (unique starts AND unique
        # goals on passable cells). A violating input is not an instance of this
        # problem — report it honestly, no plan.
        if (
            len(set(starts)) < len(starts)
            or len(set(goals)) < len(goals)
            or any(c not in free for c in [*starts, *goals])
        ):
            return self._fail(recorder, 0)

        k = len(tasks)
        w = self.params.get_int("window")
        max_steps = self.params.get_int("max_steps")
        sim = _Sim(
            free=free,
            goals=goals,
            dists=[self._bfs(free, g) for g in goals],
            paths=[[s] for s in starts],
            ell=[0] * k,
            eps=[(k - 1 - i) / k for i in range(k)],
        )
        p = list(sim.eps)  # priorities start at ε (η starts at 0)

        t = 0
        kappa = 0  # κ: cap on what lower priorities may still reserve (Algorithm 2 line 14/15)
        while any(sim.paths[i][t] != goals[i] for i in range(k)):
            if t >= max_steps:
                # Honest budget exhaustion — not a proof of unsolvability.
                return self._fail(recorder, sim.calls)
            # Priority update (Algorithm 2 line 3): on goal → reset to ε; travelling → +1.
            p = [sim.eps[i] if sim.paths[i][t] == goals[i] else p[i] + 1.0 for i in range(k)]
            order = sorted(range(k), key=lambda i: -p[i])  # values distinct ⇒ order total
            for j, i in enumerate(order):
                if sim.ell[i] <= t:  # path not yet registered beyond the current step (line 7)
                    alpha = t + w if j == 0 else min(t + w, kappa)
                    self._winpibt(sim, i, alpha)  # top-level call — its verdict ends here
                kappa = sim.ell[i] if j == 0 else min(kappa, sim.ell[i])
            t += 1

        cost = sum(
            sum(1 for s in range(1, len(p)) if p[s] != p[s - 1])
            for p in (path[: t + 1] for path in sim.paths)
        )
        if recorder is not None:
            for i, path in enumerate(sim.paths):
                recorder.path_found(path[: t + 1], i)
            recorder.planning_finished(
                True,
                {
                    "expanded_nodes": float(sim.calls),
                    "makespan": float(t),
                    "sum_of_costs": float(cost),
                },
            )
        paths = [path[: t + 1] for path in sim.paths]
        return MultiPlanResult(True, paths, float(cost), PlanStats(expanded_nodes=sim.calls))

    def _fail(self, recorder: TraceRecorder | None, calls: int) -> MultiPlanResult:
        """Honest failure (the branch's convention): no paths emitted, the decision
        count up to the budget is still reported."""
        if recorder is not None:
            recorder.planning_finished(
                False, {"expanded_nodes": float(calls), "makespan": 0.0, "sum_of_costs": 0.0}
            )
        return MultiPlanResult(False, [], 0.0, PlanStats(expanded_nodes=calls))

    @staticmethod
    def _bfs(free: frozenset[Cell], goal: Cell) -> dict[Cell, int]:
        """Static BFS distance table from the goal over free cells (fixed neighbor
        order). Reachability only — distances ignore all other agents by design."""
        dist: dict[Cell, int] = {goal: 0}
        queue: list[Cell] = [goal]
        head = 0
        while head < len(queue):
            c = queue[head]
            head += 1
            for n in _nbrs(free, c):
                if n not in dist:
                    dist[n] = dist[c] + 1
                    queue.append(n)
        return dist

    def _winpibt(self, sim: _Sim, i: int, alpha: int) -> bool:
        """The paper's Algorithm 1. Extend agent i's provisional path to step alpha;
        secure the new steps one at a time, forcing anyone parked on a secured cell
        out of it (retroactive inheritance). Invalid → copeStuck (pin waits to alpha)
        and report invalid so the claimant backtracks."""
        sim.calls += 1
        ell_i = sim.ell[i]
        if ell_i >= alpha:  # line 2: already registered beyond alpha
            return True
        # line 3: the prophetic timestep β, fixed at entry
        beta = max(alpha, max(len(p_) - 1 for p_ in sim.paths))

        path = self._ideal_path(sim, i, ell_i, beta)
        if path is None:  # lines 4-7: no valid walk — copeStuck pins waits to alpha
            v = sim.paths[i][ell_i]
            sim.paths[i].extend([v] * (alpha - ell_i))
            sim.ell[i] = alpha
            return False

        # line 8 (hard mode): register the whole ideal path up to alpha at once; only
        # the secured prefix (ℓ_i, advanced step by step below) is visible to others.
        # path is RELATIVE-indexed: index 0 is step ell_i, so steps ell_i+1..alpha are
        # indices 1..alpha-ell_i.
        sim.paths[i].extend(path[1 : alpha - ell_i + 1])

        t = ell_i + 1
        while t <= alpha:
            v = sim.paths[i][t]
            sim.ell[i] = t  # the secured step becomes visible right now, not before
            # lines 12-14 (retroactive): anyone parked on v with ℓ_j < t-1 extends one
            # step at a time until their path ends past t-1 or off v — return ignored.
            j = self._target(sim, v, t - 1)
            while j is not None:
                self._winpibt(sim, j, sim.ell[j] + 1)
                j = self._target(sim, v, t - 1)
            # lines 15-26 (the PIBT core): the occupant parked exactly at step t-1
            # inherits and must vacate by t; failure backtracks this call's extension.
            j = self._target(sim, v, t)
            if j is not None:
                if not self._winpibt(sim, j, t):
                    for _ in range(len(sim.paths[i]) - t):  # revoke Π_i(t..α)
                        sim.paths[i].pop()
                    sim.ell[i] = t - 1
                    replan = self._ideal_path(sim, i, t - 1, beta)
                    if replan is None:  # copeStuck: pin waits at the last secured cell
                        v2 = sim.paths[i][t - 1]
                        sim.paths[i].extend([v2] * (alpha - (t - 1)))
                        sim.ell[i] = alpha
                        return False
                    # re-register from the last secured step (replan is relative: index 0
                    # is step t-1, so its steps t..alpha live at indices 1..alpha-t+1)
                    sim.paths[i].extend(replan[1 : alpha - t + 2])
                    continue  # retry securing step t against the replanned route
            if v == sim.goals[i] and t < alpha:
                # Hard mode with a fixed goal (the paper's classical-MAPF framing: leaving
                # g_i re-issues task {g_i}): from the secured arrival onward the tail is
                # re-planned FROM the goal, so the agent parks there as soon as waiting
                # is disentangled instead of wandering on to β. This replan can never fail:
                # every reservation still visible at step t ends by step t, so plain waits
                # on the goal from t + 1 onward are disentangled by construction.
                replan = self._ideal_path(sim, i, t, beta)
                assert replan is not None
                sim.paths[i][t + 1 : alpha + 1] = replan[1 : alpha - t + 1]
            t += 1
        return True

    def _target(self, sim: _Sim, v: Cell, t: int) -> int | None:
        """First agent (index order) whose path ENDS on v strictly before t — the one
        that must vacate a cell being secured at step t. An agent whose registered
        path covers t is not a target: their reservation already governs validity."""
        for j, path in enumerate(sim.paths):
            if sim.ell[j] < t and path[sim.ell[j]] == v:
                return j
        return None

    def _ideal_path(self, sim: _Sim, i: int, t1: int, beta: int) -> list[Cell] | None:
        """validPath + registerPath as one search (the paper leaves "compute the ideal
        path" free; this is what we pin for it): a walk from v_i(t1) at step t1 that
        stays disentangled from every VISIBLE reservation up to the horizon beta.

        A* over (cell, step) states: f = step + static BFS distance to the goal, ties
        by later step first then insertion order; expansion in fixed neighbor order
        with wait last. At EVERY popped state the pinned static route (parent chain to
        the goal padded with waits / truncated to beta) is tried first — if valid it IS
        the ideal path from there; otherwise the search fans out one more step. Success
        = surviving to a state at step ≥ beta; failure returns None → copeStuck."""
        start = sim.paths[i][t1]
        seq = count()
        dist = sim.dists[i]
        # Heap entries (f, -step, insertion order, cell): min f first; ties resolved by
        # LATER step first (the reference's heap comparator tie), then FIFO.
        heap: list[tuple[float, int, int, Cell]] = [(dist.get(start, _INF), -t1, next(seq), start)]
        parent: dict[tuple[Cell, int], tuple[Cell, int] | None] = {(start, t1): None}
        closed: set[tuple[Cell, int]] = set()
        while heap:
            _, neg_t, _, v = heapq.heappop(heap)
            g_t = -neg_t
            if g_t >= beta:  # survived to the horizon — backtrack the chain
                cells = self._chain(parent, (v, g_t))
                return cells
            seg = self._static_chain(sim, i, v, g_t, beta)
            if seg is not None and self._valid(sim, i, seg, g_t, beta):
                return self._chain(parent, (v, g_t)) + seg[1:]
            closed.add((v, g_t))
            for m in [*_nbrs(sim.free, v), v]:  # fixed order, wait last
                state = (m, g_t + 1)
                if state in closed or state in parent:
                    continue
                if not self._valid_step(sim, i, v, m, g_t + 1, beta):
                    continue
                parent[state] = (v, g_t)
                heapq.heappush(heap, (g_t + 1.0 + dist.get(m, _INF), -(g_t + 1), next(seq), m))
        return None

    @staticmethod
    def _chain(
        parent: dict[tuple[Cell, int], tuple[Cell, int] | None], state: tuple[Cell, int]
    ) -> list[Cell]:
        """Backtrack the space-time chain to its start; index 0 is the start step."""
        cells: list[Cell] = []
        while state is not None:
            cells.append(state[0])
            state = parent[state]  # type: ignore[assignment]
        cells.reverse()
        return cells

    def _static_chain(
        self, sim: _Sim, i: int, start: Cell, t1: int, beta: int
    ) -> list[Cell] | None:
        """The static BFS parent-chain from start to goal (fixed neighbor order),
        padded with waits at the goal or truncated so it spans exactly steps t1..beta.
        Returns None when the goal is unreachable on the free graph."""
        dist = sim.dists[i]
        if start not in dist:
            return None  # unreachable goal — no ideal path exists
        cells: list[Cell] = [start]
        c = start
        while c != sim.goals[i]:
            d = dist[c]
            nxt = [n for n in _nbrs(sim.free, c) if dist.get(n, _INF) == d - 1]
            c = nxt[0]  # fixed order decides the parent — pinned by construction
            cells.append(c)
        while len(cells) - 1 + t1 < beta:
            cells.append(sim.goals[i])
        while len(cells) - 1 + t1 > beta:
            cells.pop()
        return cells

    def _valid(self, sim: _Sim, i: int, seg: list[Cell], t1: int, beta: int) -> bool:
        """Every step of the segment (occupying seg[j] at step t1+j) must stay
        disentangled from every visible reservation."""
        for j in range(1, len(seg)):
            if not self._valid_step(sim, i, seg[j - 1], seg[j], t1 + j, beta):
                return False
        return True

    def _valid_step(self, sim: _Sim, i: int, v1: Cell, v2: Cell, tau: int, beta: int) -> bool:
        """A step v1 → v2 landing at step tau is invalid iff some other agent whose
        path is SECURED up to at least tau (ℓ_j ≥ tau) occupies v2 at any step in
        [tau, min(beta, ℓ_j)] — a secured reservation stays poison until it visibly
        ends (the paper's isolation condition Def 3.4: rule 1 at the exact step plus
        rule 3 over the later secured steps) — or iff it swaps with a fully-secured
        move. Beyond ℓ_j nothing is decided for agent j: no constraint applies, and
        whoever still stands on v2 gets pulled forward by inheritance when the step
        securing it is reached."""
        for j in range(len(sim.paths)):
            if j == i:
                continue
            ell_j = sim.ell[j]
            if ell_j >= tau:
                path_j = sim.paths[j]
                for x in range(tau, min(beta, ell_j) + 1):
                    if path_j[x] == v2:
                        return False
                if path_j[tau] == v1 and path_j[tau - 1] == v2:
                    return False  # swap with a fully-secured move at tau
        return True
