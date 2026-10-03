"""db-CBS — CBS whose low level has momentum. Moldagalieva, Ortiz-Haro, Toussaint &
Hönig (arXiv:2309.16445), "db-CBS: Discontinuity-Bounded Conflict-Based Search for
Multi-Robot Kinodynamic Motion Planning": three levels — a single-robot search over
motion primitives that may be chained with bounded discontinuities (db-A*), the CBS
high level resolving inter-robot collisions by constraining states, and a joint-space
trajectory optimization seeded with the result, iterated under a shrinking bound. The
third level is out of scope for this repo exactly as MAPF-POST's LP variants were;
what IS discretizable — and what this module implements literally — is the first two,
plus the iteration over the discontinuity bound itself.

The lattice translation (pinned here, mirrored in every language):

* STATE = (cell, velocity). Velocity is an integer pair, per-axis bounded by the
  agent's vmax (the same AgentTask field MAPF-POST reads; db-CBS requires integer
  values — the lattice only quantizes integers). Acceleration is bounded by 1
  cell/step^2 per axis: from state (x, v) the ideal landings are exactly (x + u, u)
  for actions u with |u - v| <= 1 per axis and |u| <= vmax per axis — at vmax = 1
  that bound is vacuous (every 8-neighbor move plus wait is legal from every state:
  this branch's "plain" motion still differs from the search branch's 4-connected
  model, and swaps are NOT conflicts here — sampled point robots pass through each
  other).
* The discontinuity bound d(x_{k+1}, step(x_k, u_k)) <= delta becomes lattice
  arithmetic on the state distance max(Manhattan(position), |velocity delta|): a
  transition is legal iff SOME action u has |u - v| <= 1 per axis, |u| <= vmax, and
  lands within Manhattan floor(delta) of x' with velocity within floor(delta) of v'.
  At delta < 1 (floor = 0) both slack terms vanish: exact double integrator —
  momentum is law. At delta >= 1 the chaining gap covers an extra cell of position
  AND a jump of velocity: physics becomes negotiable, which is exactly what
  "discontinuity bound" means. The ladder runs [delta_start, delta_end] = [1.5, 0.5]
  by default: loose first (feasibility), exact last (the honest model); the LAST rung
  that succeeds supplies the answer — the paper's anytime structure, honestly labeled
  in metrics. Looser is NOT simply easier: a looser bound fattens the constraint
  volume too, so the rungs are independent attempts, never monotone refinements.
* CONSTRAINTS are volumes: a constraint (cell c at step t) binds agent i iff its
  position at step t is within Manhattan floor(delta) of c — the paper's d(x_c, x_k)
  > δ read on a lattice. At delta < 1 that is exactly CBS's vertex constraint; the
  volume only fattens as the bound loosens. Conflicts are sampled-state co-presence
  (same cell at the same step) — point robots have no shape to overlap, so edge/swap
  conflicts do not exist in this branch at ANY delta.
* The high level is CBS verbatim: best-first over CT nodes keyed by
  sum-of-arrival-steps then creation order; the earliest conflict (ties by cell row,
  col, then pair i < j) branches into both agents' children; a node whose sub-search
  dies never enters the queue; an empty queue is the unsolvability verdict and budget
  exhaustion is not. Each rung starts a FRESH tree from an unconstrained root (the
  paper re-initializes per iteration — constraints never cross a delta boundary).
* Low level: A* over states (cell, velocity) in absolute time; g == t, f = t + h with
  the admissible-and-consistent h = ceil(Chebyshev(cell, goal) / (vmax +
  floor(delta))); every state is pushed exactly once. The goal test reads POSITION
  only — arrival is arrival, velocity unobserved (a deliberate narrowing of the
  paper's state-valued x_f: our goal set is every state on the goal cell). A pop
  counts as the goal only when no constraint binds that cell at any step >= t.
* The paper's third level (joint-space DDP) and its asymptotic-optimality limit are
  OUT OF SCOPE: the lattice never refines, so this repo's honest claim ends at
  "complete on the discretized model at delta < 1" — the ladder tightens dynamics
  and constraint volume, not resolution.

Determinism across languages: successor sets are enumerated in canonical sorted order
(row, col, v_row, v_col); heap ties break by creation order exactly like the search
branch; every pinned number below is reproducible byte-for-byte in C++/TS.
"""

from __future__ import annotations

import heapq
import itertools
import math
from collections import deque
from dataclasses import dataclass

from ..core.capabilities import Capability, DiscreteSpace
from ..core.params import ParamError
from ..core.planner import MultiAgentPlanner
from ..core.trace import TraceRecorder
from ..core.types import AgentTask, Cell, MultiPlanResult, PlanStats


@dataclass(frozen=True)
class _Constraint:
    """One CT constraint. The agent must keep its position Manhattan-farther than
    floor(delta) from `cell` at step t; below delta 1 that is exactly "not here"."""

    cell: Cell
    t: int


@dataclass
class _Node:
    """One CT node: per-agent constraint lists + their individually-optimal state
    paths (each a list of cells indexed by absolute step — the velocity component
    never leaves the search, the executed artifact stays a cell sequence)."""

    constraints: list[list[_Constraint]]
    paths: list[list[Cell]]


def _manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class DbCbs(MultiAgentPlanner):
    @property
    def name(self) -> str:
        return "db_cbs"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE}

    def plan(
        self,
        space: DiscreteSpace,
        tasks: list[AgentTask],
        recorder: TraceRecorder | None = None,
    ) -> MultiPlanResult:
        delta_start = float(self.params.get_float("delta_start"))
        delta_end = float(self.params.get_float("delta_end"))
        max_ct = int(self.params.get_int("max_ct_expansions"))
        # The ladder only tightens — a widening bound is not what the paper's loop
        # does (it reduces the discontinuity each iteration, never grows it). Each
        # parameter alone validates fine; only plan() can see the pair.
        if delta_end > delta_start:
            raise ParamError(
                f"db_cbs: delta_end ({delta_end}) must not exceed delta_start "
                f"({delta_start}) — the ladder tightens, never loosens"
            )
        # The velocity lattice only quantizes integers.
        for task in tasks:
            if not float(task.vmax).is_integer() or task.vmax < 1:
                raise ParamError(
                    f"db_cbs: vmax must be a positive integer (got {task.vmax}) — "
                    "the velocity lattice quantizes integers only"
                )

        # The capability stays the plain discrete grid, but this planner never reads
        # its 4-connected move model — the lattice needs only the map's vertex set:
        # the passable cells every landing must lie on (row-major order is irrelevant
        # here; membership is what matters).
        free = frozenset(space.cells())

        ladder = [delta_start] if delta_start == delta_end else [delta_start, delta_end]
        expanded = 0
        solved: list[list[Cell]] | None = None
        answered_delta = 0.0
        for delta in ladder:
            paths, n = self._rung(free, tasks, float(delta), max_ct, recorder)
            expanded += n
            if paths is None:
                # This rung died (verdict or budget). Looser rungs are not easier
                # (a looser bound fattens the constraint volume too), so this stops
                # nothing — the answer stays whatever an earlier rung solved, and a
                # ladder with no survivor is an honest failure.
                continue
            solved = paths
            answered_delta = delta

        if solved is None:
            return self._fail(expanded, ladder[-1], recorder)

        cost = float(sum(len(p) - 1 for p in solved))
        makespan = float(max((len(p) - 1 for p in solved), default=0))
        if recorder is not None:
            for k, path in enumerate(solved):
                recorder.path_found(path, k)
            recorder.planning_finished(
                True,
                {
                    "delta": answered_delta,
                    "expanded_nodes": float(expanded),
                    "makespan": makespan,
                    "sum_of_costs": cost,
                },
            )
        return MultiPlanResult(True, solved, cost, PlanStats(expanded_nodes=expanded))

    def _fail(self, expanded: int, delta: float, recorder: TraceRecorder | None) -> MultiPlanResult:
        # The delta an honest failure names is the tightest rung ever attempted —
        # the model whose every branch died.
        if recorder is not None:
            recorder.planning_finished(
                False,
                {
                    "delta": delta,
                    "expanded_nodes": float(expanded),
                    "makespan": 0.0,
                    "sum_of_costs": 0.0,
                },
            )
        return MultiPlanResult(False, [], 0.0, PlanStats(expanded_nodes=expanded))

    def _rung(
        self,
        free: frozenset[Cell],
        tasks: list[AgentTask],
        delta: float,
        max_ct: int,
        recorder: TraceRecorder | None,
    ) -> tuple[list[list[Cell]] | None, int]:
        """One full db-CBS run at one discontinuity bound: a fresh constraint tree
        whose root plans every agent unconstrained (constraints never cross a
        delta boundary — the paper re-initializes per iteration). Returns the
        conflict-free joint state-path or None (verdict/budget), plus how many
        low-level expansions it cost."""
        f = math.floor(delta)  # lattice slack: <1 -> exact, >=1 that many cells
        expanded = 0
        root_paths: list[list[Cell]] = []
        for k, task in enumerate(tasks):
            path, n = self._plan_one(free, task, [], f, k, recorder)
            expanded += n
            if path is None:
                return None, expanded
            root_paths.append(path)

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
                # Best-first pops solutions by increasing sum-of-arrival-steps and
                # every node's paths are individually optimal under their
                # constraints — the first conflict-free node IS jointly optimal at
                # this rung's delta. Done.
                return node.paths, expanded
            cell, t, (i, j) = conflict
            if recorder is not None:
                recorder.conflict_found("vertex", cell, t, (i, j))
            for agent in (i, j):
                child_constraints = [list(c) for c in node.constraints]
                child_constraints[agent].append(_Constraint(cell, t))
                if recorder is not None:
                    recorder.constraint_added(agent, "vertex", cell, t)
                path, n = self._plan_one(
                    free, tasks[agent], child_constraints[agent], f, agent, recorder
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

        return None, expanded

    def _plan_one(
        self,
        free: frozenset[Cell],
        task: AgentTask,
        constraints: list[_Constraint],
        f: int,
        agent: int,
        recorder: TraceRecorder | None,
    ) -> tuple[list[Cell] | None, int]:
        """db-A*: A* over the state space (cell, velocity) in absolute time. g == t
        never improves, so a pop IS the expansion; f = t + ceil(Chebyshev / (vmax +
        f)) — admissible and consistent because every legal move shifts each axis by
        at most vmax + floor(delta). A successor state is blocked when any constraint
        binds its cell at its step; the goal pops only when no constraint binds the
        goal cell at ANY step >= t (occupancy persists after arrival). Finiteness:
        past the last constrained step nothing binds anymore, and from every state
        some equivalent SIMPLE state-path exists — so states past
        constrained_until + |reachable| x (2*vmax+1)^2 cannot matter."""
        start, goal = task.start, task.goal
        vmax = int(task.vmax)
        # Being born inside a forbidden volume is not undoable — no move can erase
        # step 0. The instance itself is malformed for this rung's delta.
        if any(c.t == 0 and _manhattan(c.cell, start) <= f for c in constraints):
            return None, 0

        reach = self._reachable(free, start, vmax + f)
        if goal not in reach:
            return None, 0  # statically unreachable — honest fail
        constrained_until = max((c.t for c in constraints), default=0)
        # Finiteness: the state space over reachable cells is exactly
        # |reach| x (2*vmax+1)^2 states; any feasible continuation shortens to a
        # SIMPLE state-path, at most that many steps long. Past the last constrained
        # step nothing binds anymore, so no plan needs one step more.
        horizon = constrained_until + len(reach) * (2 * vmax + 1) ** 2

        counter = itertools.count()  # stable FIFO tie-break among equal f
        frontier: list[tuple[float, int, Cell, tuple[int, int], int]] = [
            (float(self._heuristic(start, goal, vmax, f)), next(counter), start, (0, 0), 0)
        ]
        seen: set[tuple[Cell, tuple[int, int], int]] = {(start, (0, 0), 0)}
        parent: dict[tuple[Cell, tuple[int, int], int], tuple[Cell, tuple[int, int]]] = {}
        expanded = 0

        while frontier:
            _, _, cell, v, t = heapq.heappop(frontier)
            expanded += 1
            if recorder is not None:
                recorder.node_expanded(list(cell), float(t), agent, t)
            # Goal guard: after arrival the agent occupies `goal` forever, so the
            # pop counts only when NO constraint binds the goal cell at any t' >= t.
            if cell == goal and all(
                not (c.t >= t and _manhattan(c.cell, goal) <= f) for c in constraints
            ):
                path: list[Cell] = []
                key: tuple[Cell, tuple[int, int], int] | None = (cell, v, t)
                while key is not None:
                    path.append(key[0])
                    # The parent map stores the predecessor STATE (cell, velocity);
                    # its step is exactly one lower — every transition costs one.
                    prev = parent.get(key)
                    key = None if prev is None else (prev[0], prev[1], key[2] - 1)
                path.reverse()  # index 0 == step 0
                return path, expanded
            if t == horizon:
                continue
            t2 = t + 1
            for succ_cell, succ_v in self._successors(free, cell, v, vmax, f):
                blocked = any(
                    c.t == t2 and _manhattan(c.cell, succ_cell) <= f for c in constraints
                )
                if blocked:
                    continue
                key = (succ_cell, succ_v, t2)
                if key in seen:
                    continue
                seen.add(key)
                parent[key] = (cell, v)
                heapq.heappush(
                    frontier,
                    (
                        float(t2) + self._heuristic(succ_cell, goal, vmax, f),
                        next(counter),
                        succ_cell,
                        succ_v,
                        t2,
                    ),
                )

        return None, expanded

    @staticmethod
    def _heuristic(cell: Cell, goal: Cell, vmax: int, f: int) -> int:
        """Steps to the goal cell can never undercut Chebyshev / (vmax + floor(f)):
        no legal move shifts either axis by more than that."""
        return math.ceil(max(abs(cell[0] - goal[0]), abs(cell[1] - goal[1])) / (vmax + f))

    @staticmethod
    def _successors(
        free: frozenset[Cell], cell: Cell, v: tuple[int, int], vmax: int, f: int
    ) -> list[tuple[Cell, tuple[int, int]]]:
        """Every state reachable in one legal transition, in canonical sorted order.
        A transition (cell, v) -> (x', v') is legal iff SOME action u with
        |u - v| <= 1 per axis and |u| <= vmax lands within Manhattan f of x' and has
        velocity within f of v'. At f = 0 that collapses to the exact double
        integrator: x' = x + v' and v' itself is the action."""
        out: set[tuple[Cell, tuple[int, int]]] = set()
        for ur in range(max(-vmax, v[0] - 1), min(vmax, v[0] + 1) + 1):
            for uc in range(max(-vmax, v[1] - 1), min(vmax, v[1] + 1) + 1):
                # Landing cells within Manhattan f of the ideal landing x+u.
                for dr in range(-f, f + 1):
                    for dc in range(-(f - abs(dr)), (f - abs(dr)) + 1):
                        land: Cell = (cell[0] + ur + dr, cell[1] + uc + dc)
                        if land not in free:
                            continue
                        # Landing states carry any velocity within f of the action.
                        for vr in range(max(-vmax, ur - f), min(vmax, ur + f) + 1):
                            for vc in range(max(-vmax, uc - f), min(vmax, uc + f) + 1):
                                out.add((land, (vr, vc)))
        return sorted(out)

    @staticmethod
    def _reachable(free: frozenset[Cell], start: Cell, reach_r: int) -> set[Cell]:
        """Flood fill over the move relation itself: from a reached cell every
        passable cell within PER-AXIS distance reach_r = vmax + floor(delta) is one
        transition's landing away. At floor 0 and vmax 1 that is exactly the
        8-neighborhood; wider bounds hop cells (and walls — sampled point robots
        only ever ARE at sampled endpoints, overflying is legal by construction)."""
        seen: set[Cell] = {start}
        queue: deque[Cell] = deque([start])
        while queue:
            cell = queue.popleft()
            r, c = cell
            for dr in range(-reach_r, reach_r + 1):
                for dc in range(-reach_r, reach_r + 1):
                    succ: Cell = (r + dr, c + dc)
                    if succ not in seen and succ in free:
                        seen.add(succ)
                        queue.append(succ)
        return seen

    @staticmethod
    def _first_conflict(
        paths: list[list[Cell]],
    ) -> tuple[Cell, int, tuple[int, int]] | None:
        """Earliest step with any co-presence (same cell at the same sampled step);
        ties by (cell row, col), then pair i < j. A swap across a step is NOT a
        conflict here — point robots sampled at integer steps pass through each
        other; that is what this branch's per-timestep check actually guarantees."""
        horizon = max((len(p) - 1 for p in paths), default=0)
        for t in range(horizon + 1):
            candidates: list[tuple[Cell, int, int]] = []
            for i, j in itertools.combinations(range(len(paths)), 2):
                now_i = DbCbs._occupied(paths[i], t)
                now_j = DbCbs._occupied(paths[j], t)
                if now_i == now_j:
                    candidates.append((now_i, i, j))
            if candidates:
                cell, i, j = min(candidates)
                return cell, t, (i, j)
        return None

    @staticmethod
    def _occupied(path: list[Cell], t: int) -> Cell:
        """Cell occupied at step t by a finished path: after arrival the agent
        stays (its state path simply ends)."""
        return path[t] if t < len(path) else path[-1]
