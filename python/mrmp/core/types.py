"""Language-neutral state / result types shared by every planner.

Mirrors the C++ `core/types.hpp`. Two state kinds live side by side: discrete
planners search grid cells (row, col) of ints; continuous planners (dRRT family)
plan for disc robots on the same map's continuous free space, so their states are
world points (x, y) in meters. The trace wire stays numeric pairs either way —
`planning_started.coords` tells readers which interpretation applies. A third
result kind (TimedPlanResult, kinodynamic branch) keeps Cell states but adds the
time axis: per-agent routes plus each retained location's earliest arrival time.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# Discrete state: grid index (row, col), ints; row 0 = top image row. Every agent
# shares one grid; a joint state is just several Cells (flattened [r0,c0,r1,c1,...]
# on the trace wire), so there is no separate joint-state type.
Cell = tuple[int, int]
# World point (x, y) in meters — map-layer coordinates only (scenario start/goal,
# continuous planner states). The kinodynamic branch plans on CELLS and converts
# nothing: its velocity limits are cells per time unit on the same grid.
Point = tuple[float, float]


@dataclass(frozen=True)
class AgentTask:
    """One agent's planning task on the shared grid (the demo driver converts the
    scenario's world-coord start/goal into Cells — coordinate frames stay owned by
    the map layer, per the repo rule).

    `vmax` is the kinodynamic branch's per-agent velocity limit in CELLS per time
    unit (default 1.0 = one cell per time unit, which reproduces the discrete
    step timing up to the safety distance). The search/sampling branches never read
    it; only KinodynamicPlanner implementations do."""

    start: Cell
    goal: Cell
    vmax: float = 1.0


@dataclass(frozen=True)
class ContinuousTask:
    """One agent's planning task on the shared continuous space: a disc robot of
    `radius` (meters) with world-coord start/goal. The scenario file carries the
    radius; discrete planners never see these tasks."""

    start: Point
    goal: Point
    radius: float


@dataclass
class PlanStats:
    expanded_nodes: int = 0


@dataclass
class MultiPlanResult:
    success: bool
    # paths[k] is agent k's space-time path: paths[k][t] is the cell occupied at
    # time step t. A finished agent's path simply ends; later steps do not exist.
    paths: list[list[Cell]] = field(default_factory=list)
    # sum of costs: every action (move or wait) costs one time step, so an agent's
    # cost is len(path) - 1 and this is the sum over agents (standard MAPF metric;
    # makespan — arrival step of the last agent — is derived from paths by callers).
    cost: float = 0.0
    stats: PlanStats = field(default_factory=PlanStats)


@dataclass
class ContinuousPlanResult:
    """Continuous-space counterpart of MultiPlanResult. paths[k][t] is agent k's
    world point at time step t (motion between consecutive waypoints is linear);
    a finished agent's path simply ends (a waypoint equal to the previous one is a
    wait). Unlike the discrete result, the METRICS here are what each planner's own
    paper defines as cost — dRRT counts steps like the discrete branch (cost = steps
    that move, makespan = arrival step of the last agent), while dRRT* reports
    geometric arc lengths (sum and max over agents) because its own cost functions
    are reparameterization-invariant lengths. Both fields are explicit here so no
    caller ever derives a metric from paths by convention."""

    success: bool
    paths: list[list[Point]] = field(default_factory=list)
    cost: float = 0.0
    makespan: float = 0.0
    stats: PlanStats = field(default_factory=PlanStats)


@dataclass
class TimedPlanResult:
    """Kinodynamic-branch result: a plan-execution SCHEDULE, not a space-time path.

    routes[k] is agent k's route — the collision-free plan's cell sequence with the
    wait actions removed (consecutive cells are adjacent; every move edge has unit
    length). times[k][i] is the earliest arrival time at routes[k][i]; times[k][0]
    is 0 for every agent (every start event is pinned to t = 0 by the STN source).
    Execution under the uniform velocity model: an agent dwells on a cell until its
    departure (arrival of the next location minus l(e)/v_k) and traverses at exactly
    v_k — so arrival lands exactly on the scheduled time.

    Metrics are explicit like ContinuousPlanResult: cost is the sum over agents of
    their goal arrival times (the flow-time analogue the paper's LP variant minimizes)
    and makespan is t(X_F), the latest arrival. Both are in TIME units, not steps."""

    success: bool
    routes: list[list[Cell]] = field(default_factory=list)
    times: list[list[float]] = field(default_factory=list)
    cost: float = 0.0
    makespan: float = 0.0
    stats: PlanStats = field(default_factory=PlanStats)
