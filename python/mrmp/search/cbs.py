"""CBS — Conflict-Based Search, the hybrid pole of the MAPF genealogy.

Sharon, Stern, Felner & Sturtevant (2015), "Conflict-Based Search for Optimal
Multi-Agent Pathfinding": stop searching the joint state AND stop hoping a fixed
priority order works out. Instead plan every agent alone against explicit
constraints (the low level — wave 1's space-time A*, reservations replaced by
constraints), keep each single-agent solution as a node of a CONSTRAINT TREE, and
when the cheapest node's joint solution still collides, branch: one child forbids
the first agent from that collision, the other forbids the second. Optimal like
joint-space search without ever touching |V|^k — agents are coupled only at the
exact cell and step where they actually collide.

One honest caveat about termination, which is why this planner has a parameter
where its two siblings have none: CBS is OPTIMAL and COMPLETE (a solution, if one
exists, sits under every live branch's cost bound, so best-first eventually pops
it) — but it is only SEMI-decidable. On an unsolvable instance the tree never
dies: each branch merely pushes the conflict to a later step, forever. So the
search runs on a budget of constraint-tree expansions (`max_ct_expansions`, the
root counting as the first). A queue that empties before the budget is spent IS a
verdict (every branch died on its own constraints); hitting the budget instead is
honestly "no solution found within budget", never a proof of unsolvability.

Semantics fixed here (identical in C++/TS):

* Occupancy is stay-at-goal everywhere: occ_k(t) = path_k[min(t, T_k)]. A VERTEX
  conflict at step t means occ_i(t) == occ_j(t); an EDGE conflict at step t means
  a pair swapped cells across the step (t-1 -> t). Both kinds are keyed by the
  ARRIVAL step — the same convention wave 1's reservation checks used.
* Edge conflicts and constraints are canonicalized: `cell` is the lexicographic
  min of the two cells, `to` the max; direction is not encoded, so a constraint
  forbids traversing between the pair in either direction across that step.
* Conflict selection: earliest step with any conflict; ties broken by (cell row,
  cell col), then agent pair i < j. The scan runs over every pair at once — no
  priority among agents here, only lexicographic tie-breaks.
* A constraint (vertex c@t) forbids occupying c at t; a constraint (edge {c,d}@τ)
  forbids the swap across that step in either direction. The low level accepts a
  goal pop only when no vertex constraint on the goal cell binds at any t' >= t
  (stay-at-goal occupancy persists forever after arrival).
* High level: best-first over CT nodes keyed by (sum-of-costs, creation seq).
  The root expands at construction — every agent planned unconstrained in index
  order; any sub-search failing fails the whole instance right there (a goal no
  constraint-free path can reach is unreachable under EVERY constraint set).
  Popping a node scans its solution: no conflict means success (best-first makes
  that first conflict-free node optimal), otherwise the node branches into both
  children in pair order. A child whose low level fails dies and is never pushed;
  an empty queue is the honest verdict that no joint plan exists at all.
* Trace chronology: root expansions come first (agent-index order); each pop
  emits `conflict_found` before its children's `constraint_added` + re-plan
  expansions; `path_found` events appear ONLY for the final solution paths in
  agent-index order. expanded_nodes counts every low-level pop across ALL
  sub-searches, dead branches included — that is the honest price of optimality.
"""

from __future__ import annotations

import heapq
import itertools
from collections import deque
from dataclasses import dataclass

from ..core.capabilities import Capability, DiscreteSpace
from ..core.planner import MultiAgentPlanner
from ..core.trace import ConflictKind, TraceRecorder
from ..core.types import AgentTask, Cell, MultiPlanResult, PlanStats


@dataclass(frozen=True)
class Constraint:
    """One CT constraint. Vertex: occupy `cell` at step t is forbidden. Edge:
    traverse between `cell` and `to` across step t (canonicalized, both ways)."""

    kind: ConflictKind
    cell: Cell
    t: int
    to: Cell | None = None


def _occupied(path: list[Cell], t: int) -> Cell:
    """Cell occupied at step t by a finished path: after arrival the agent stays."""
    return path[t] if t < len(path) else path[-1]


def _reachable(space: DiscreteSpace, start: Cell) -> set[Cell]:
    """Static flood fill over passable cells — no constraints involved."""
    seen: set[Cell] = {start}
    queue: deque[Cell] = deque([start])
    while queue:
        cell = queue.popleft()
        for succ, _cost in space.neighbors(cell):
            if succ not in seen:
                seen.add(succ)
                queue.append(succ)
    return seen


@dataclass
class _Node:
    """One CT node: per-agent constraint lists + their individually-optimal paths."""

    constraints: list[list[Constraint]]
    paths: list[list[Cell]]


class Cbs(MultiAgentPlanner):
    @property
    def name(self) -> str:
        return "cbs"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE}

    def plan(
        self,
        space: DiscreteSpace,
        tasks: list[AgentTask],
        recorder: TraceRecorder | None = None,
    ) -> MultiPlanResult:
        max_ct = int(self.params.get_int("max_ct_expansions"))

        # Root expansion (the first of the budget): every agent planned
        # unconstrained, in index order. If an unconstrained sub-search fails the
        # goal is statically unreachable — no constraint set can ever help, so the
        # whole instance fails right here.
        expanded = 0
        root_paths: list[list[Cell]] = []
        for k, task in enumerate(tasks):
            path, n = self._plan_one(space, task, [], k, recorder)
            expanded += n
            if path is None:
                return self._fail(expanded, recorder)
            root_paths.append(path)

        # Best-first CT queue: cheapest sum-of-costs first, ties broken by
        # creation order (the same FIFO tie-break as every other page).
        counter = itertools.count()
        root_cost = float(sum(len(p) - 1 for p in root_paths))
        queue: list[tuple[float, int, _Node]] = [
            (root_cost, next(counter), _Node([[] for _ in tasks], root_paths))
        ]
        ct_expansions = 1

        while queue and ct_expansions < max_ct:
            cost, _, node = heapq.heappop(queue)
            ct_expansions += 1

            conflict = self._first_conflict(node.paths)
            if conflict is None:
                # Best-first pops solutions by increasing sum-of-costs and every
                # node's paths are individually optimal under their constraints —
                # the first conflict-free node IS jointly optimal. Done.
                makespan = float(max((len(p) - 1 for p in node.paths), default=0))
                if recorder is not None:
                    for k, path in enumerate(node.paths):
                        recorder.path_found(path, k)
                    recorder.planning_finished(
                        True,
                        {
                            "expanded_nodes": float(expanded),
                            "makespan": makespan,
                            "sum_of_costs": cost,
                        },
                    )
                return MultiPlanResult(True, node.paths, cost, PlanStats(expanded_nodes=expanded))

            kind, cell, t, (i, j), to = conflict
            if recorder is not None:
                recorder.conflict_found(kind, cell, t, (i, j), to)

            # Branch: one child per conflicting agent, in pair order. Each child
            # forbids its agent from THIS collision and re-plans only that agent;
            # a child whose sub-search fails dies and is never pushed.
            for agent in (i, j):
                child_constraints = [list(c) for c in node.constraints]
                child_constraints[agent].append(Constraint(kind, cell, t, to))
                if recorder is not None:
                    recorder.constraint_added(agent, kind, cell, t, to)
                path, n = self._plan_one(
                    space, tasks[agent], child_constraints[agent], agent, recorder
                )
                expanded += n
                if path is None:
                    continue
                child_paths = list(node.paths)
                child_paths[agent] = path
                child_cost = cost - float(len(node.paths[agent]) - 1) + float(len(path) - 1)
                heapq.heappush(
                    queue, (child_cost, next(counter), _Node(child_constraints, child_paths))
                )

        # Queue exhausted before the budget: every branch died on its own
        # constraints. A joint plan would satisfy at least one constraint on every
        # branch point it disagrees with, so a live branch always survives for it —
        # exhaustion is an honest unsolvability verdict. (Budget exhaustion, by
        # contrast, is only "no solution found within budget" — see the module doc.)
        return self._fail(expanded, recorder)

    @staticmethod
    def _fail(expanded: int, recorder: TraceRecorder | None) -> MultiPlanResult:
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

    def _plan_one(
        self,
        space: DiscreteSpace,
        task: AgentTask,
        constraints: list[Constraint],
        agent: int,
        recorder: TraceRecorder | None,
    ) -> tuple[list[Cell] | None, int]:
        """A* over space-time (cell, t) against this node's constraints for one
        agent — wave 1's sub-search with constraints replacing reservations.

        A move into succ at step t+1 is legal only if no vertex constraint binds
        (succ, t+1) and no edge constraint canonical-matches {cell, succ} at t+1;
        popping the goal counts only when no vertex constraint on the goal cell
        binds at any step >= t (stay-at-goal occupancy persists). Finiteness:
        past the last constrained step nothing binds anymore, so every feasible
        plan has an equivalent one of length <= max(constrained t) + |reachable|;
        states past that horizon cannot matter (A* pops by f before reaching it).
        """
        start, goal = task.start, task.goal
        # A vertex constraint on the start cell at t=0 is unavoidable — no move
        # can undo being born inside a forbidden cell.
        if any(c.kind == "vertex" and c.cell == start and c.t == 0 for c in constraints):
            return None, 0

        reachable = _reachable(space, start)
        if goal not in reachable:
            return None, 0
        constrained_until = max((c.t for c in constraints), default=0)
        horizon = constrained_until + len(reachable)

        counter = itertools.count()  # stable FIFO tie-break among equal f
        h0 = space.heuristic(start, goal)
        frontier: list[tuple[float, int, Cell, int]] = [(h0, next(counter), start, 0)]
        seen: set[tuple[Cell, int]] = {(start, 0)}
        parent: dict[tuple[Cell, int], Cell] = {}
        expanded = 0

        while frontier:
            _, _, cell, t = heapq.heappop(frontier)
            # Every state is pushed exactly once (g == t never improves), so a pop
            # IS the expansion — no lazy-deletion skip needed.
            expanded += 1
            if recorder is not None:
                recorder.node_expanded(list(cell), float(t), agent, t)
            # Goal guard: after arrival the agent occupies `goal` forever, so the
            # pop counts only when NO vertex constraint on goal binds at any t' >= t.
            if cell == goal and all(
                not (c.kind == "vertex" and c.cell == goal and c.t >= t) for c in constraints
            ):
                path: list[Cell] = [cell]
                cur_cell, cur_t = cell, t
                while cur_t > 0:
                    prev = parent[(cur_cell, cur_t)]
                    path.append(prev)
                    cur_cell, cur_t = prev, cur_t - 1
                path.reverse()
                return path, expanded
            if t == horizon:
                continue
            t2 = t + 1
            for succ, _cost in space.neighbors(cell):
                blocked = any(
                    c.kind == "vertex" and c.cell == succ and c.t == t2 for c in constraints
                ) or any(
                    c.kind == "edge" and c.t == t2 and self._matches_edge(c, cell, succ)
                    for c in constraints
                )
                if blocked:
                    continue
                key = (succ, t2)
                if key in seen:
                    continue
                seen.add(key)
                parent[key] = cell
                heapq.heappush(
                    frontier, (float(t2) + space.heuristic(succ, goal), next(counter), succ, t2)
                )

        return None, expanded

    @staticmethod
    def _matches_edge(c: Constraint, from_cell: Cell, to_cell: Cell) -> bool:
        """An edge constraint forbids traversing between its two cells in EITHER
        direction across its step — the pair is canonicalized, not directed."""
        return (c.cell == from_cell and c.to == to_cell) or (
            c.cell == to_cell and c.to == from_cell
        )

    @staticmethod
    def _first_conflict(
        paths: list[list[Cell]],
    ) -> tuple[ConflictKind, Cell, int, tuple[int, int], Cell | None] | None:
        """Earliest conflict over all agent pairs; ties by (cell row, col), then
        pair i < j. Past the last arrival every occupancy is constant, so a swap
        can no longer happen and any shared cell was already caught — scanning to
        max(T_k) covers everything."""
        horizon = max((len(p) - 1 for p in paths), default=0)
        for t in range(horizon + 1):
            candidates: list[tuple[Cell, int, int]] = []  # (canonical cell, i, j)
            kinds: dict[tuple[Cell, int, int], tuple[ConflictKind, Cell | None]] = {}
            for i, j in itertools.combinations(range(len(paths)), 2):
                now_i, now_j = _occupied(paths[i], t), _occupied(paths[j], t)
                if now_i == now_j:
                    key = (now_i, i, j)
                    candidates.append(key)
                    kinds[key] = ("vertex", None)
                    continue
                if t >= 1:
                    prev_i, prev_j = _occupied(paths[i], t - 1), _occupied(paths[j], t - 1)
                    # Swap across the step (t-1 -> t): both moved and swapped cells.
                    if now_i == prev_j and now_j == prev_i:
                        key = (min(now_i, now_j), i, j)
                        candidates.append(key)
                        kinds[key] = ("edge", max(now_i, now_j))
            if candidates:
                cell, i, j = min(candidates)
                kind, to = kinds[(cell, i, j)]
                return kind, cell, t, (i, j), to
        return None
