"""RHCR — Rolling-Horizon Collision Resolution, the search branch's hybrid axis folded
into time itself.

Li, Tinka, Kiesel, Durham, Kumar & Koenig (AAAI-21), "Lifelong Multi-Agent Path
Finding in Large-Scale Warehouses" (arXiv:2005.07371): stop resolving collisions over
the whole horizon at once. Decompose the problem into a sequence of WINDOWED MAPF
instances — each one replanned from the actual positions every h steps, resolving
collisions only inside the window and ignoring everything beyond it. This module is
the paper's batch adaptation: every agent carries exactly one real task (start →
goal), and once an agent stands on its goal its remaining task sequence is empty —
the paper's own degenerate case for finite tasks (their footnote 1: the task assigner
assigns a dummy task whose goal location is the agent's current position, i.e. stay
put).

The windowed solver inside is CBS itself — the search branch's hybrid pole, verbatim:
same space-time A* low level, same canonicalized vertex/edge constraints, same
best-first constraint tree with the same tie-breaks (earliest conflict step; ties by
cell row, col, then agent pair i < j). The only difference is what counts as a
conflict: an episode starting at step T resolves collisions whose ARRIVAL step falls
in (T, T+w] and ignores every collision beyond. Execution then commits the first h
steps (h ≤ w — every executed step must fall inside some resolved window), and those
executed positions become the next episode's starts. That is the whole algorithm.

What the window buys and costs, pinned here identically to C++/TS:

* PLIABLE, never frozen. Executed history carries forward only through each agent's
  CURRENT position; no old path segment is reserved. A parked agent can be constrained
  off its own goal by a later window and must leave and come back — exactly the
  flexibility that makes RHCR beat endpoint-holding (the paper's method-3 comparison);
  here it is the same hard-mode semantics the decentralized branch pinned for PIBT.
* NOT optimal, NOT complete — by design, not as a caveat. A collision beyond w is
  invisible forever. And every windowed instance is satisfiable: parking at your start
  past the horizon always fits, so the constraint tree NEVER empties and no episode
  ever proves unsolvability — unlike plain CBS's semi-decidable tree, this planner has
  no verdict failure mode, only the honest budget stop. Small w also deadlocks honestly
  on head-on corridors: every window sees a locally-solvable instance (both agents wait
  past the horizon), agents creep or stall, and the budget expires. When w covers an
  instance's whole horizon the window hides nothing: the single episode IS plain CBS
  with identical tie-breaks — RHCR is literally the generalization whose w = ∞ limit is
  the hybrid pole it wraps (the h = 1 rolling on top re-solves optimally every step,
  and sum-of-costs decomposes over time, so the executed cost still equals CBS's).
* Termination: execution stops at the first step where every agent simultaneously
  stands on its final goal (from there everyone trivially stays). The reported paths
  are the EXECUTED trajectories — like the decentralized branch's page, every path
  spans the whole makespan and an agent that already arrived just keeps repeating its
  goal cell. sum_of_costs therefore sums each agent's FIRST arrival at its final goal
  (the paper's flowtime objective): exactly what the search branch reports as
  Σ(len(path) - 1) over paths that end at arrival, which is why a large-window run
  reports CBS's number; makespan is the simultaneous-co-presence step itself.
* Well-formedness (distinct starts, distinct goals, passable cells) is checked up front
  like the decentralized branch does: a t=0 collision is history, not a conflict the
  window could resolve — an ill-formed input is simply not an instance of this problem.

Fixed degrees of freedom (the paper leaves these free; identical here in Python, C++
and TS by construction): neighbor order up/down/left/right-then-wait; A* frontier
tie-break f = absolute step + Manhattan heuristic, ties by insertion order; conflict
selection earliest arrival step, ties by (cell row, col) then pair i < j; CT queue
keyed by (sum of absolute arrivals, creation sequence); root expansions in agent-index
order. The paper's windowed solvers are CBS/ECBS/CA*/PBS — this is the plain-CBS one,
so inside its own window the resolution is exactly as optimal as the hybrid pole gets.
"""

from __future__ import annotations

import heapq
import itertools
from dataclasses import dataclass

from ..core.capabilities import Capability, DiscreteSpace
from ..core.params import ParamError
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


@dataclass
class _CtNode:
    """One CT node of one episode: per-agent constraint lists + their individually-
    optimal absolute-indexed paths."""

    constraints: list[list[Constraint]]
    paths: list[list[Cell]]


def _occupied(path: list[Cell], t: int) -> Cell:
    """Cell occupied at absolute step t by a finished path: after arrival stays."""
    return path[t] if t < len(path) else path[-1]


def _reachable(space: DiscreteSpace, start: Cell) -> set[Cell]:
    """Static flood fill over passable cells — no constraints involved."""
    seen: set[Cell] = {start}
    queue: list[Cell] = [start]
    head = 0
    while head < len(queue):
        cell = queue[head]
        head += 1
        for succ, _cost in space.neighbors(cell):
            if succ not in seen:
                seen.add(succ)
                queue.append(succ)
    return seen


class Rhcr(MultiAgentPlanner):
    @property
    def name(self) -> str:
        return "rhcr"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE}

    def plan(
        self,
        space: DiscreteSpace,
        tasks: list[AgentTask],
        recorder: TraceRecorder | None = None,
    ) -> MultiPlanResult:
        window = int(self.params.get_int("window"))
        period = int(self.params.get_int("replan_period"))
        max_steps = int(self.params.get_int("max_steps"))
        # Cross-parameter constraint the param schema cannot express: h > w would
        # execute steps that no resolved window ever covered — not a harder problem,
        # an unsafe one. The paper states it as a requirement (w ≥ h), so raise,
        # never clamp.
        if period > window:
            raise ParamError(
                f"param error: replan_period {period} exceeds window {window} "
                "(every executed step must fall inside some resolved window)"
            )

        free = frozenset(space.cells())
        starts = [t.start for t in tasks]
        goals = [t.goal for t in tasks]
        # Well-formed instance: distinct starts, distinct goals, passable cells. A
        # t=0 collision is history rather than a conflict the window could resolve —
        # an ill-formed input is not an instance of this problem at all.
        if (
            len(set(starts)) < len(starts)
            or len(set(goals)) < len(goals)
            or any(c not in free for c in [*starts, *goals])
        ):
            return self._fail(recorder, 0)

        k = len(tasks)
        executed: list[list[Cell]] = [[s] for s in starts]  # index t = cell at step t
        t_now = 0
        calls = 0  # every low-level pop across ALL episodes — the honest price of rolling

        while True:
            if all(executed[i][t_now] == goals[i] for i in range(k)):
                return self._finish(executed, goals, t_now, calls, recorder)
            if t_now >= max_steps:
                # Honest budget exhaustion — and RHCR can never say more: a collision
                # beyond w is invisible forever, so no episode ever proves unsolvability.
                return self._fail(recorder, calls)

            # One Windowed MAPF episode at absolute step t_now: fresh constraint-free
            # root from the ACTUAL positions (pliable — nothing old stays reserved),
            # conflicts resolved only for arrival steps in (t_now, t_now + window].
            paths, n = self._episode(space, goals, executed, t_now, window, recorder)
            calls += n
            if paths is None:  # a sub-search died on its constraints — honest failure
                return self._fail(recorder, calls)

            # Commit h steps; scan them for the first simultaneous co-presence.
            done_at = -1
            for s in range(t_now + 1, t_now + period + 1):
                for i in range(k):
                    executed[i].append(_occupied(paths[i], s))
                if done_at < 0 and all(executed[i][s] == goals[i] for i in range(k)):
                    done_at = s
            if done_at >= 0:
                # Everyone simultaneously on their final goal — from here everyone
                # trivially stays, so execution stops there. Costs are the FIRST
                # arrivals (the paper's flowtime); makespan is this step.
                return self._finish(executed, goals, done_at, calls, recorder)
            t_now += period

    def _finish(
        self,
        executed: list[list[Cell]],
        goals: list[Cell],
        done_at: int,
        calls: int,
        recorder: TraceRecorder | None,
    ) -> MultiPlanResult:
        """Success: truncate every trajectory at the co-presence step and report.
        Each agent's cost is its FIRST arrival (the paper's flowtime); makespan is
        the simultaneous co-presence step."""
        paths = [path[: done_at + 1] for path in executed]
        cost = float(
            sum(
                next(s for s in range(done_at + 1) if path[s] == goal)
                for path, goal in zip(paths, goals, strict=True)
            )
        )
        if recorder is not None:
            for i, path in enumerate(paths):
                recorder.path_found(path, i)
            recorder.planning_finished(
                True,
                {
                    "expanded_nodes": float(calls),
                    "makespan": float(done_at),
                    "sum_of_costs": cost,
                },
            )
        return MultiPlanResult(True, paths, cost, PlanStats(expanded_nodes=calls))

    def _fail(self, recorder: TraceRecorder | None, calls: int) -> MultiPlanResult:
        if recorder is not None:
            recorder.planning_finished(
                False,
                {
                    "expanded_nodes": float(calls),
                    "makespan": 0.0,
                    "sum_of_costs": 0.0,
                },
            )
        return MultiPlanResult(False, [], 0.0, PlanStats(expanded_nodes=calls))

    def _episode(
        self,
        space: DiscreteSpace,
        goals: list[Cell],
        executed: list[list[Cell]],
        t_now: int,
        window: int,
        recorder: TraceRecorder | None,
    ) -> tuple[list[list[Cell]] | None, int]:
        """One Windowed MAPF episode: plain CBS over absolute space-time, but a
        conflict only counts when its arrival step falls in (t_now, t_now + window].
        Returns each agent's full path (absolute-indexed) or None if some sub-search
        dies on its constraints. The tree always terminates: every constraint carries
        a step ≤ t_now + window, so past that horizon nothing binds and each agent
        has finitely many constrained-optimal paths — unlike plain CBS's
        semi-decidable tree, emptiness here WOULD be a verdict (on these maps it
        never happens; parking past the horizon always fits)."""
        expanded = 0
        root_paths: list[list[Cell]] = []
        for i in range(len(goals)):
            rel, n = self._plan_one(
                space, executed[i][t_now], goals[i], [], t_now, recorder, i
            )
            expanded += n
            if rel is None:
                return None, expanded
            root_paths.append(executed[i][:t_now] + rel)

        counter = itertools.count()
        root_cost = float(sum(len(p) - 1 for p in root_paths))
        queue: list[tuple[float, int, _CtNode]] = [
            (root_cost, next(counter), _CtNode([[] for _ in goals], root_paths))
        ]

        while queue:
            cost, _, node = heapq.heappop(queue)
            conflict = self._first_conflict(node.paths, t_now, window)
            if conflict is None:
                return node.paths, expanded  # window-clean — this episode's solution
            kind, cell, t, (i, j), to = conflict
            if recorder is not None:
                recorder.conflict_found(kind, cell, t, (i, j), to)
            for agent in (i, j):
                child_constraints = [list(c) for c in node.constraints]
                child_constraints[agent].append(Constraint(kind, cell, t, to))
                if recorder is not None:
                    recorder.constraint_added(agent, kind, cell, t, to)
                rel, n = self._plan_one(
                    space,
                    executed[agent][t_now],
                    goals[agent],
                    child_constraints[agent],
                    t_now,
                    recorder,
                    agent,
                )
                expanded += n
                if rel is None:
                    continue
                child_paths = list(node.paths)
                child_paths[agent] = executed[agent][:t_now] + rel
                child_cost = (
                    cost
                    - float(len(node.paths[agent]) - 1)
                    + float(t_now + len(rel) - 1)
                )
                heapq.heappush(
                    queue, (child_cost, next(counter), _CtNode(child_constraints, child_paths))
                )

        return None, expanded

    def _plan_one(
        self,
        space: DiscreteSpace,
        start: Cell,
        goal: Cell,
        constraints: list[Constraint],
        t_now: int,
        recorder: TraceRecorder | None,
        agent: int,
    ) -> tuple[list[Cell] | None, int]:
        """A* over absolute space-time (cell, t), t ≥ t_now — the same sub-search as
        cbs.py with the clock shifted to the episode's start. Finiteness: every
        constraint binds at some step ≤ t_now + window, so past that horizon nothing
        binds and every feasible plan has an equivalent one of length ≤
        max(constrained t) + |reachable|."""
        # A vertex constraint on the start cell AT t_now is unavoidable — no move can
        # undo being born inside a forbidden cell. (By construction constraints here
        # only ever carry steps > t_now, so this guard never fires; mirrored from
        # cbs.py for exactness.)
        if any(c.kind == "vertex" and c.cell == start and c.t <= t_now for c in constraints):
            return None, 0

        reachable = _reachable(space, start)
        if goal not in reachable:
            # Statically unreachable from the CURRENT position: no constraint set
            # can ever help, so this episode (and with it the whole instance)
            # fails honestly here — same guard as cbs.py.
            return None, 0
        constrained_until = max((c.t for c in constraints), default=t_now)
        horizon = constrained_until + len(reachable)

        counter = itertools.count()  # stable FIFO tie-break among equal f
        h0 = float(t_now) + space.heuristic(start, goal)
        frontier: list[tuple[float, int, Cell, int]] = [(h0, next(counter), start, t_now)]
        seen: set[tuple[Cell, int]] = {(start, t_now)}
        parent: dict[tuple[Cell, int], Cell] = {}
        expanded = 0

        while frontier:
            _, _, cell, t = heapq.heappop(frontier)
            # Every state is pushed exactly once (g == t never improves), so a pop IS
            # the expansion — no lazy-deletion skip needed.
            expanded += 1
            if recorder is not None:
                recorder.node_expanded(list(cell), float(t), agent, t)
            # Goal guard: after arrival the agent occupies `goal` forever, so the pop
            # counts only when NO vertex constraint on goal binds at any step >= t.
            if cell == goal and all(
                not (c.kind == "vertex" and c.cell == goal and c.t >= t) for c in constraints
            ):
                path: list[Cell] = [cell]
                cur_cell, cur_t = cell, t
                while cur_t > t_now:
                    prev = parent[(cur_cell, cur_t)]
                    path.append(prev)
                    cur_cell, cur_t = prev, cur_t - 1
                path.reverse()  # index 0 == step t_now; caller re-absolutizes
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
        paths: list[list[Cell]], t_now: int, window: int
    ) -> tuple[ConflictKind, Cell, int, tuple[int, int], Cell | None] | None:
        """Earliest conflict whose ARRIVAL step falls inside the window, (t_now,
        t_now + window]; ties by (cell row, col), then pair i < j — the same fixed
        order as plain CBS. Beyond the window nothing is a conflict: that is what
        the window IS."""
        horizon = min(t_now + window, max(len(p) - 1 for p in paths))
        for t in range(t_now + 1, horizon + 1):
            candidates: list[tuple[Cell, int, int]] = []  # (canonical cell, i, j)
            kinds: dict[tuple[Cell, int, int], tuple[ConflictKind, Cell | None]] = {}
            for i, j in itertools.combinations(range(len(paths)), 2):
                now_i, now_j = _occupied(paths[i], t), _occupied(paths[j], t)
                if now_i == now_j:
                    key = (now_i, i, j)
                    candidates.append(key)
                    kinds[key] = ("vertex", None)
                    continue
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
