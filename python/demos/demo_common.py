"""Shared demo scaffolding: arg parsing + run flow for multi-agent planners.

Demos are assembly only — they wire params + map + scenario + planner and emit
`planning_started` (the one event the planner cannot emit, since the map path is
known only here). Everything else is emitted by the planner. Discrete demos use
run() (world coords -> Cells, cell-interpreted traces); continuous demos use
run_continuous() (raw world Points + disc radius, world-point traces declared via
the planning_started `coords` field).
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Callable

from mrmp.core.capabilities import Capability
from mrmp.core.params import ParamSet
from mrmp.core.planner import ContinuousMultiAgentPlanner, MultiAgentPlanner
from mrmp.core.trace import open_trace
from mrmp.core.types import (
    AgentTask,
    ContinuousPlanResult,
    ContinuousTask,
    MultiPlanResult,
)
from mrmp.maps.loader import load_map, load_scenario
from mrmp.maps.occupancy_grid import OccupancyGrid2D

PlannerFactory = Callable[[ParamSet], MultiAgentPlanner]
ContinuousFactory = Callable[[ParamSet], ContinuousMultiAgentPlanner]


def _parse_args(name: str) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=f"mrmp {name} demo")
    parser.add_argument("--map", required=True, help="map yaml path")
    parser.add_argument("--scenario", required=True, help="scenario yaml path")
    parser.add_argument("--params", required=True, help="algorithm config yaml path")
    parser.add_argument("--trace", required=True, help="output trace jsonl path")
    return parser.parse_args()


def run(name: str, factory: PlannerFactory) -> None:
    args = _parse_args(name)
    params = ParamSet.from_yaml(args.params)
    grid = load_map(args.map)
    assert isinstance(grid, OccupancyGrid2D)
    scenario = load_scenario(args.scenario)
    # World-coord start/goal per agent -> grid cells (coordinate frames stay owned
    # by the map layer, per the repo rule). Agent index == position in the list.
    tasks: list[AgentTask] = [
        AgentTask(
            start=grid.world_to_cell(*spec.start),
            goal=grid.world_to_cell(*spec.goal),
        )
        for spec in scenario.agents
    ]
    planner = factory(params)
    with open_trace(args.trace) as recorder:
        recorder.planning_started(planner.name, args.map, params.values())
        result = planner.plan(grid, tasks, recorder)
    _report(planner.name, result)


def run_continuous(name: str, factory: ContinuousFactory) -> None:
    args = _parse_args(name)
    params = ParamSet.from_yaml(args.params)
    grid = load_map(args.map)
    assert isinstance(grid, OccupancyGrid2D)
    scenario = load_scenario(args.scenario)
    # Continuous planners keep the scenario's world coords as-is and take the
    # disc radius straight from the scenario (agent index == list position).
    tasks: list[ContinuousTask] = [
        ContinuousTask(start=spec.start, goal=spec.goal, radius=spec.radius)
        for spec in scenario.agents
    ]
    planner = factory(params)
    assert Capability.CONTINUOUS_SPACE in planner.required_capabilities()
    with open_trace(args.trace) as recorder:
        recorder.planning_started(
            planner.name,
            args.map,
            params.values(),
            coords="world",
            radius=[task.radius for task in tasks],
        )
        result = planner.plan(grid, tasks, recorder)
    _report(planner.name, result)


def _report(name: str, result: MultiPlanResult | ContinuousPlanResult) -> None:
    # One-line JSON metrics on stdout (bench + web export read it). Discrete
    # results derive makespan = arrival step of the last agent; continuous results
    # carry cost AND makespan explicitly (dRRT counts steps, dRRT* reports arc
    # lengths — see ContinuousPlanResult). Parsed values are what must match.
    summary = {
        "algorithm": name,
        "success": result.success,
        "sum_of_costs": round(result.cost, 4),
        "makespan": (
            result.makespan if isinstance(result, ContinuousPlanResult)
            else float(max((len(p) - 1 for p in result.paths), default=0))
        ),
        "expanded_nodes": result.stats.expanded_nodes,
    }
    print(json.dumps(summary))
