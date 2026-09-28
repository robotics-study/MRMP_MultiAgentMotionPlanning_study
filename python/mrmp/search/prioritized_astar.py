"""Prioritized planning — plan agents one at a time, later agents treat earlier
paths as moving obstacles.

Erdmann & Lozano-Pérez (1987), "On Multiple Moving Objects": assign priorities
to the objects, then plan motions one object at a time; each planner searches a
configuration space-time that represents the time-varying constraints imposed by
the already-planned (time-parameterized) paths. Here the single-agent search is
plain A* over states ``(cell, t)`` — unit-cost 4-connected moves plus wait,
Manhattan heuristic, goal tested at pop.

Priority order IS the agent index: agent 0 plans first against the static map
only; every later agent reserves all earlier paths in space-time. Occupancy is
stay-at-goal: a finished path keeps occupying its final cell forever, so a later
agent may only enter a cell an earlier path visits *after* it leaves — and may
never finish on a cell an earlier path will still occupy later (the goal guard).

Not optimal (each agent is individually optimal against fixed reservations; the
joint plan need not be), not complete (an early path can box a later agent in —
that returns ``success=False``, honestly). The time axis is made finite exactly:
after every reservation has frozen, only simple static paths matter, so states
beyond ``frozen + |reachable cells|`` cannot belong to any minimal feasible plan.

Determinism contract: neighbor order is the map's fixed up/down/left/right/wait
order and equal-f ties pop in push order, so C++/Python/TS runs produce identical
expansions and paths.
"""

from __future__ import annotations

import heapq
import itertools
from collections import deque

from ..core.capabilities import Capability, DiscreteSpace
from ..core.planner import MultiAgentPlanner
from ..core.trace import TraceRecorder
from ..core.types import AgentTask, Cell, MultiPlanResult, PlanStats


def _occupied(path: list[Cell], t: int) -> Cell:
    """Cell occupied at step t by a finished path: after arrival the agent stays."""
    return path[t] if t < len(path) else path[-1]


def _reachable(space: DiscreteSpace, start: Cell) -> set[Cell]:
    """Static flood fill over passable cells — no reservations involved."""
    seen: set[Cell] = {start}
    queue: deque[Cell] = deque([start])
    while queue:
        cell = queue.popleft()
        for succ, _cost in space.neighbors(cell):
            if succ not in seen:
                seen.add(succ)
                queue.append(succ)
    return seen


class PrioritizedAStar(MultiAgentPlanner):
    @property
    def name(self) -> str:
        return "prioritized_astar"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE}

    def plan(
        self,
        space: DiscreteSpace,
        tasks: list[AgentTask],
        recorder: TraceRecorder | None = None,
    ) -> MultiPlanResult:
        paths: list[list[Cell]] = []
        expanded = 0
        for agent, task in enumerate(tasks):
            path, n = self._plan_one(space, task, paths, agent, recorder)
            expanded += n
            if path is None:
                # A later agent boxed in by earlier paths fails the whole plan.
                # The trace keeps every expansion and the earlier agents'
                # path_found events; metrics report the failure honestly (0 cost).
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
            if recorder is not None:
                recorder.path_found(path, agent)
            paths.append(path)
        cost = float(sum(len(p) - 1 for p in paths))
        makespan = float(max((len(p) - 1 for p in paths), default=0))
        if recorder is not None:
            recorder.planning_finished(
                True,
                {
                    "expanded_nodes": float(expanded),
                    "makespan": makespan,
                    "sum_of_costs": cost,
                },
            )
        return MultiPlanResult(True, paths, cost, PlanStats(expanded_nodes=expanded))

    def _plan_one(
        self,
        space: DiscreteSpace,
        task: AgentTask,
        planned: list[list[Cell]],
        agent: int,
        recorder: TraceRecorder | None,
    ) -> tuple[list[Cell] | None, int]:
        """A* over space-time (cell, t) against the reserved paths in `planned`.

        Every action costs one step so g == t — a state's cost is fixed at
        discovery and never improves, hence each state is pushed exactly once and
        no relaxation is needed. A move into ``succ`` at step t+1 is legal only if
        no earlier path occupies succ at t+1 (vertex conflict) nor swaps it with
        the current cell across that step (edge/swap conflict). Popping the goal
        is not enough: under stay-at-goal semantics the finished agent occupies
        its goal forever, so the pop is accepted only when no earlier path visits
        the goal at any step >= t.

        Finiteness: a plain flood fill decides static unreachability up front; for
        reachable goals every feasible plan has an equivalent one of length <=
        frozen + |reachable| (after time `frozen` all reservations are parked, so
        any feasible suffix simplifies to a simple static path), and states past
        that horizon cannot matter — A* pops by f before ever reaching it on the
        success path.
        """
        start, goal = task.start, task.goal
        # An earlier path standing on this agent's start at t=0 is an unavoidable
        # joint conflict — no move can undo it.
        if any(_occupied(p, 0) == start for p in planned):
            return None, 0

        reachable = _reachable(space, start)
        if goal not in reachable:
            return None, 0
        frozen = max((len(p) - 1 for p in planned), default=0)
        horizon = frozen + len(reachable)

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
            if cell == goal and all(
                _occupied(p, tt) != goal for p in planned for tt in range(t, len(p))
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
            for succ, _cost in space.neighbors(cell):
                t2 = t + 1
                blocked = any(
                    _occupied(p, t2) == succ
                    or (_occupied(p, t) == succ and _occupied(p, t2) == cell)
                    for p in planned
                )
                if blocked or (succ, t2) in seen:
                    continue
                seen.add((succ, t2))
                parent[(succ, t2)] = cell
                heapq.heappush(
                    frontier,
                    (float(t2) + space.heuristic(succ, goal), next(counter), succ, t2),
                )
        return None, expanded
