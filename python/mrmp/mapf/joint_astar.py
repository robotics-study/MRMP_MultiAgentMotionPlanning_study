"""Joint-space A* — the coupled baseline of the MAPF genealogy.

Every earlier page's single-robot search, applied to the whole team at once: the
search state is the TUPLE of all agent positions at one instant, and one search
(not one per agent) finds a jointly optimal plan. This is the honest baseline —
complete and sum-of-costs optimal by construction — paid for with a state space
of |V|^k; it exists to show exactly what the decoupled and hybrid poles trade away.

Semantics fixed here (identical in C++/TS):

* A joint state is ``(c_0, ..., c_{k-1})``. An agent whose position equals its
  goal is ARRIVED: it pins in place (stay-at-goal semantics made structural —
  its only action is the self-loop) and pays nothing further. The step at which
  an agent first reaches its goal is that agent's cost; sum-of-costs therefore
  accumulates one per still-unarrived agent per joint step, exactly matching
  ``len(path) - 1`` of the prioritized representation.
* A joint transition is legal only if the new tuple has no repeated cell (vertex
  conflict) and no pair swaps cells across the step (edge conflict). Duplicate
  starts fail the instance outright — no joint state can separate them at t=0.
* The search is plain lazy-deletion A*: heap keyed by ``(f, seq)`` with a push
  counter breaking equal-f ties in insertion order; a state popped for the first
  time is expanded once and never again (the heuristic below is consistent, so
  the first pop already carries optimal g). Heuristic = sum of per-agent Manhattan
  distances — admissible and consistent for this cost model. Goal tested at pop.
* Successor enumeration order is fixed: ``itertools.product`` over the per-agent
  action lists in agent-index order (last agent varies fastest), each list in the
  map's fixed up/down/left/right/wait order. Same tie-breaks, same expansions in
  every language.

The joint state carries no time — arrival times live in the reconstructed paths:
agent k's path is its position sequence up to its FIRST arrival (after which it
pinned anyway), so both algorithms hand back identical space-time paths for an
identical solution.
"""

from __future__ import annotations

import heapq
import itertools

from ..core.capabilities import Capability, DiscreteSpace
from ..core.planner import MultiAgentPlanner
from ..core.trace import TraceRecorder
from ..core.types import AgentTask, Cell, MultiPlanResult, PlanStats


def _flatten(state: tuple[Cell, ...]) -> list[int]:
    """Joint state wire form: [r0, c0, r1, c1, ...] (see core/trace.py)."""
    out: list[int] = []
    for row, col in state:
        out.extend([row, col])
    return out


class JointAStar(MultiAgentPlanner):
    @property
    def name(self) -> str:
        return "joint_astar"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE}

    def plan(
        self,
        space: DiscreteSpace,
        tasks: list[AgentTask],
        recorder: TraceRecorder | None = None,
    ) -> MultiPlanResult:
        goals = tuple(task.goal for task in tasks)
        start = tuple(task.start for task in tasks)

        # Two agents sharing a cell at t=0 is an unsolvable instance — the joint
        # state space contains no conflict-free state that could ever separate them.
        for i, j in itertools.combinations(range(len(tasks)), 2):
            if start[i] == start[j]:
                if recorder is not None:
                    recorder.planning_finished(
                        False,
                        {
                            "expanded_nodes": 0.0,
                            "makespan": 0.0,
                            "sum_of_costs": 0.0,
                        },
                    )
                return MultiPlanResult(False, [], 0.0, PlanStats(expanded_nodes=0))

        def h(state: tuple[Cell, ...]) -> float:
            # Sum of per-agent Manhattan distances — admissible (each unarrived
            # agent needs at least that many steps, each paying 1) and consistent
            # (one joint step moves any agent at most one cell).
            return sum(
                space.heuristic(pos, goal)
                for pos, goal in zip(state, goals, strict=True)
            )

        counter = itertools.count(1)
        h0 = h(start)
        frontier: list[tuple[float, int, tuple[Cell, ...]]] = [(h0, next(counter), start)]
        g_score = {start: 0}
        parent: dict[tuple[Cell, ...], tuple[Cell, ...]] = {}
        closed: set[tuple[Cell, ...]] = set()
        expanded = 0

        while frontier:
            _, _, state = heapq.heappop(frontier)
            if state in closed:
                # Stale heap entry — the state was already popped (and then at its
                # optimal g, h being consistent). Not an expansion.
                continue
            closed.add(state)
            expanded += 1
            if recorder is not None:
                recorder.node_expanded(_flatten(state), float(g_score[state]))

            if state == goals:
                chain = [state]
                while chain[-1] != start:
                    chain.append(parent[chain[-1]])
                chain.reverse()  # chain[t] = joint positions at step t
                paths: list[list[Cell]] = []
                for k in range(len(tasks)):
                    arrival = next(t for t in range(len(chain)) if chain[t][k] == goals[k])
                    paths.append([chain[t][k] for t in range(arrival + 1)])
                cost = float(sum(len(p) - 1 for p in paths))
                makespan = float(max((len(p) - 1 for p in paths), default=0))
                if recorder is not None:
                    for k, path in enumerate(paths):
                        recorder.path_found(path, k)
                    recorder.planning_finished(
                        True,
                        {
                            "expanded_nodes": float(expanded),
                            "makespan": makespan,
                            "sum_of_costs": cost,
                        },
                    )
                return MultiPlanResult(True, paths, cost, PlanStats(expanded_nodes=expanded))

            # One joint step: every unarrived agent takes exactly one action (a
            # move or the wait self-loop, each costing its one time step); arrived
            # agents pin in place and pay nothing.
            step_cost = sum(
                1 for pos, goal in zip(state, goals, strict=True) if pos != goal
            )
            actions = [
                [pos] if pos == goal else [n for n, _cost in space.neighbors(pos)]
                for pos, goal in zip(state, goals, strict=True)
            ]
            g_now = g_score[state]
            for combo in itertools.product(*actions):
                succ = tuple(combo)
                # Vertex conflict: two agents on one cell at the new step.
                if any(
                    succ[i] == succ[j]
                    for i, j in itertools.combinations(range(len(state)), 2)
                ):
                    continue
                # Edge conflict: a pair swaps cells across this step.
                if any(
                    succ[i] == state[j] and state[i] == succ[j]
                    for i, j in itertools.combinations(range(len(state)), 2)
                ):
                    continue
                g2 = g_now + step_cost
                if g2 >= g_score.get(succ, float("inf")):
                    # No improvement — with a consistent heuristic the state was
                    # (or will be) settled at a better g; skip the push.
                    continue
                parent[succ] = state
                g_score[succ] = g2
                heapq.heappush(frontier, (float(g2) + h(succ), next(counter), succ))

        # Frontier exhausted: no joint plan exists (complete search — this is a
        # verdict on the instance, not on any priority order). Report honestly.
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
