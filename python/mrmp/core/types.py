"""Language-neutral state / result types shared by every planner.

Mirrors the C++ `core/types.hpp`. Every agent plans on one shared occupancy grid,
so a single-agent state is just a grid cell (row, col) of ints — a joint state is
several Cells and needs no separate type. World coordinates stay in the map layer;
only scenario loading ever touches floats here.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# Discrete state: grid index (row, col), row 0 = top image row. Every agent shares
# one grid; a joint state is just several Cells (flattened [r0,c0,r1,c1,...] on the
# trace wire), so there is no separate joint-state type.
Cell = tuple[int, int]
# World point (x, y) in meters — map-layer coordinates only (scenario start/goal).
Point = tuple[float, float]


@dataclass(frozen=True)
class AgentTask:
    """One agent's planning task on the shared grid (the demo driver converts the
    scenario's world-coord start/goal into Cells — coordinate frames stay owned by
    the map layer, per the repo rule)."""

    start: Cell
    goal: Cell


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
    # makespan — arrival time of the last agent — is derived from paths by callers).
    cost: float = 0.0
    stats: PlanStats = field(default_factory=PlanStats)
