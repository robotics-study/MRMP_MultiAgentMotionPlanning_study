"""PIBT — Okumura, Machida, Défago & Tamura (IJCAI 2019 / Artificial Intelligence 310, 2022).

The decentralized branch's representative: everything before this branch planned the
WHOLE plan offline and only then executed it. PIBT has no plan until it exists one
timestep at a time. Every timestep: (1) every agent updates its priority — standing on
the goal resets p_i to ε_i, still travelling increments p_i by 1, so any active agent
always outranks any parked one and within each group the pinned index order rules;
(2) agents decide in decreasing priority order, each picking its next cell from
Neigh(current) ∪ {current} sorted by BFS distance to its own goal (ties: unoccupied
first — an occupied candidate would force inheritance nobody needs yet — then fixed
row-major cell order); (3) a cell another agent still occupies is not taken but
CLAIMED: the occupant inherits the claimant's priority and must vacate into one of
its own candidates, and if it cannot, the claim backtracks. A swap is structurally
impossible — an inheriting agent may never move into its claimant's current cell —
and a vertex conflict is impossible because every claimed cell is excluded from every
later choice. The top-level call can never fail: whoever decides with nobody above
them always keeps their own cell, so the group never deadlocks by construction; what
kills PIBT instead is topology — an edge on no cycle (a width-1 corridor) gives the
blocked agent nowhere to vacate to, and the run deadlocks honestly there.

Completeness (paper Theorem 1): if every pair of adjacent free cells lies on a simple
cycle of length ≥ 3 (the same sufficient condition Push and Swap needed, for the same
reason), every agent reaches its goal within diam(G)·|A| steps regardless of priorities.
Where the graph violates that condition there is no guarantee — head-on swaps in
width-1 corridors deadlock at the budget and fail honestly, exactly like the priority
branch's honest failures on tree-shaped maps.

Pinned degrees of freedom (the paper leaves all of these free or random; identical
here in Python, C++ and TS by construction):

* ε_i = (k-1-i)/k — distinct values in [0,1) as required, agent 0 highest (the repo's
  index-order convention). Priorities stay distinct forever: every step adds the same
  +1 to active agents and resets parked ones below all active ones.
* Candidate order: BFS distance to goal ascending; ties unoccupied-before-occupied;
  final tie-break row-major cell order.
* Distance tables are static BFS from each goal over free cells, computed once at
  setup (the paper's own suggestion against the on-demand-A* bottleneck). Cells
  unreachable from a goal sort after every reachable one, still in fixed order.
* The run stops when every agent simultaneously stands on its goal (makespan = that
  step) or at the max_steps budget — an honest "no solution found within budget",
  never a proof of unsolvability (same convention as CBS's tree budget).

Paths are full-horizon like the priority branch: paths[k][t] is agent k's cell at
step t for every agent up to the makespan, because a parked agent CAN be pushed off
its goal by an active one — that is what the priority reset encodes. sum_of_costs
counts actual moves (waits cost nothing), expanded_nodes counts decision-procedure
invocations (top-level + inherited) — this algorithm has no search frontier at all.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from ..core.capabilities import Capability, DiscreteSpace
from ..core.planner import MultiAgentPlanner
from ..core.trace import TraceRecorder
from ..core.types import AgentTask, Cell, MultiPlanResult, PlanStats

# 4-connected neighbor order — the SAME fixed convention as the map layer's move set.
# The wait action is not a neighbor: staying put is the current cell itself, which is
# always in the candidate list (that is what makes top-level decisions fail-proof).
_MOVES_4 = ((-1, 0), (1, 0), (0, -1), (0, 1))

# Distance from an unreachable goal: sorts after every reachable cell; among equals
# the fixed row-major order still decides. float('inf') compares identically to the
# C++/TS mirrors' infinity by IEEE-754 construction.
_INF = float("inf")


def _nbrs(free: frozenset[Cell], c: Cell) -> list[Cell]:
    """Free 4-neighbors of c in the fixed order (out-of-bounds cells are not in the
    free set by construction, so membership alone bounds the grid)."""
    return [(c[0] + dr, c[1] + dc) for dr, dc in _MOVES_4 if (c[0] + dr, c[1] + dc) in free]


@dataclass
class _Sim:
    """Mutable state of one timestep loop. pos is π[t] (current cells), nxt is the
    partially decided π[t+1] (None = not yet assigned — an agent still undecided can
    be pulled into an inheritance chain); dists[k][c] is the static BFS distance from
    cell c to agent k's goal, float('inf') when unreachable; calls counts every
    decision-procedure invocation (the expanded_nodes metric)."""

    free: frozenset[Cell]
    goals: list[Cell]
    dists: list[dict[Cell, int]]
    pos: list[Cell]
    nxt: list[Cell | None] = field(default_factory=list)
    calls: int = 0


class Pibt(MultiAgentPlanner):
    @property
    def name(self) -> str:
        return "pibt"

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
        max_steps = self.params.get_int("max_steps")
        # Static distance tables (paper: BFS from each goal up front). Distances are
        # plain integers — the fixed neighbor order matters only for reproducibility
        # of the map traversal itself, never for a value.
        sim = _Sim(
            free=free,
            goals=goals,
            dists=[self._bfs(free, g) for g in goals],
            pos=list(starts),
            nxt=[],
        )
        # Pinned priorities: agent 0 highest (ε closest to 1). Distinct by
        # construction — the paper requires distinct ε and only asks ε ∈ [0,1).
        eps = [(k - 1 - i) / k for i in range(k)]
        p = list(eps)
        paths: list[list[Cell]] = [[s] for s in starts]

        t = 0
        while any(pos != g for pos, g in zip(sim.pos, goals, strict=True)):
            if t >= max_steps:
                # Honest budget exhaustion — not a proof of unsolvability.
                return self._fail(recorder, sim.calls)
            # Priority update (paper line 7): on goal → reset to ε; travelling → +1.
            p = [eps[i] if sim.pos[i] == goals[i] else p[i] + 1.0 for i in range(k)]
            # Decreasing priority; values are distinct so the order is total.
            order = sorted(range(k), key=lambda i: -p[i])
            sim.nxt = [None] * k
            for i in order:
                if sim.nxt[i] is None:
                    self._decide(sim, i, None)
            t += 1
            for i in range(k):
                assert sim.nxt[i] is not None  # top-level calls never fail (Lemma 1)
                paths[i].append(sim.nxt[i])  # type: ignore[arg-type]
                sim.pos[i] = sim.nxt[i]  # type: ignore[assignment]

        cost = sum(sum(1 for s in range(1, len(path)) if path[s] != path[s - 1]) for path in paths)
        if recorder is not None:
            for i, path in enumerate(paths):
                recorder.path_found(path, i)
            recorder.planning_finished(
                True,
                {
                    "expanded_nodes": float(sim.calls),
                    "makespan": float(t),
                    "sum_of_costs": float(cost),
                },
            )
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

    def _decide(self, sim: _Sim, i: int, from_agent: int | None) -> bool:
        """The paper's recursive procedure. Agent i picks its next cell; a candidate
        still occupied by an UNDECIDED agent k triggers inheritance (PIBT(k, i): k
        inherits i's claim and must vacate or make this claim fail), and the parent's
        current cell is excluded from i's candidates so a swap can never form. A
        candidate already claimed by anyone is skipped outright — vertex conflicts are
        structurally impossible."""
        sim.calls += 1
        cur = sim.pos[i]
        dist = sim.dists[i]

        def rank(u: Cell) -> tuple[float, int, int, int]:
            occupied = 1 if any(pos == u for pos in sim.pos) else 0
            return (dist.get(u, _INF), occupied, u[0], u[1])

        candidates = sorted([cur] + _nbrs(sim.free, cur), key=rank)
        for v in candidates:
            if any(nxt == v for nxt in sim.nxt):
                continue  # already claimed this step — vertex conflict by definition
            if from_agent is not None and v == sim.pos[from_agent]:
                continue  # the claimant's own cell: taking it would be a swap
            # tentative claim BEFORE recursing — that is what forces an occupant to vacate
            sim.nxt[i] = v
            occ = next((k for k, pos in enumerate(sim.pos) if pos == v), None)
            if occ is not None and occ != i and sim.nxt[occ] is None:
                if self._decide(sim, occ, i):
                    return True
                sim.nxt[i] = None  # backtracked — the claim never happened
                continue
            return True  # unoccupied, or its occupant already vacates: settled
        sim.nxt[i] = cur  # stuck: stay put and report invalid to whoever claimed me
        return False
