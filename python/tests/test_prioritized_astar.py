"""Prioritized A*: per-agent optimality against reservations, conflict-freedom,
the stay-at-goal guard, and the honest failure of an unsolvable priority order."""

from __future__ import annotations

import io
import json

import pytest
from conftest import REPO_ROOT, config, grid_from

from mrmp.core.capabilities import Capability
from mrmp.core.trace import TraceRecorder
from mrmp.core.types import AgentTask, Cell
from mrmp.maps.loader import load_map, load_scenario
from mrmp.maps.occupancy_grid import OccupancyGrid2D
from mrmp.search import PrioritizedAStar


def _occ(path: list[Cell], t: int) -> Cell:
    """Stay-at-goal occupancy of a finished path (same rule as the planner)."""
    return path[t] if t < len(path) else path[-1]


def assert_joint_valid(paths: list[list[Cell]]) -> None:
    """No vertex conflict and no swap at any step, across every agent pair."""
    horizon = max(len(p) - 1 for p in paths)
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            for t in range(1, horizon + 1):
                a_now, a_prev = _occ(paths[i], t), _occ(paths[i], t - 1)
                b_now, b_prev = _occ(paths[j], t), _occ(paths[j], t - 1)
                assert a_now != b_now, f"vertex conflict at step {t}"
                assert not (a_now == b_prev and a_prev == b_now), f"swap at step {t}"


def _scenario(name: str) -> tuple[OccupancyGrid2D, list[AgentTask]]:
    scenario = load_scenario(REPO_ROOT / "maps" / "scenarios" / f"{name}.yaml")
    grid = load_map(scenario.map_path)
    tasks = [
        AgentTask(grid.world_to_cell(*spec.start), grid.world_to_cell(*spec.goal))
        for spec in scenario.agents
    ]
    return grid, tasks


def test_contract_matches_config() -> None:
    planner = PrioritizedAStar(config("prioritized_astar"))
    assert planner.name == config("prioritized_astar").algorithm == "prioritized_astar"
    assert planner.required_capabilities() == {Capability.DISCRETE_SPACE}


def test_single_agent_is_plain_optimal_a_star() -> None:
    # With one agent there are no reservations: cost must equal the Manhattan
    # distance (the unconstrained optimum) on an open grid.
    planner = PrioritizedAStar(config("prioritized_astar"))
    result = planner.plan(grid_from(["....."] * 5), [AgentTask((4, 0), (0, 4))])
    assert result.success
    assert result.cost == pytest.approx(8.0)
    assert result.paths[0][0] == (4, 0) and result.paths[0][-1] == (0, 4)


def test_maze01_two_head_on_passes_by_timing() -> None:
    # Both agents share the one corridor but their timings stagger naturally:
    # each still achieves its unconstrained shortest cost (33 + 33 = 66).
    grid, tasks = _scenario("maze01_two")
    result = PrioritizedAStar(config("prioritized_astar")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost == pytest.approx(66.0)
    assert max(len(p) - 1 for p in result.paths) == 33


def test_open01_cross_both_go_straight() -> None:
    # The crossing at (10,9) is timed apart without either agent slowing down:
    # both costs equal their Manhattan distance (16 + 17 = 33).
    grid, tasks = _scenario("open01_cross")
    result = PrioritizedAStar(config("prioritized_astar")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost == pytest.approx(33.0)
    assert max(len(p) - 1 for p in result.paths) == 17


def test_open01_swap_detours_around_the_moving_wall() -> None:
    # Head-on swap on row 10: agent 0's straight path is a moving wall; agent 1
    # cannot pass it head-on (parity rules out cost 15) and must detour via an
    # adjacent row — 14 + 16 = 30, which here equals the joint optimum.
    grid, tasks = _scenario("open01_swap")
    result = PrioritizedAStar(config("prioritized_astar")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost == pytest.approx(30.0)
    assert max(len(p) - 1 for p in result.paths) == 16


def test_head_on_corridor_swap_is_unsolvable() -> None:
    # One-wide corridor, agents swapping ends: no wait or detour exists, so the
    # second agent honestly fails — prioritized planning is not complete.
    planner = PrioritizedAStar(config("prioritized_astar"))
    grid = grid_from(["#####", ".....", "#####"])
    tasks = [AgentTask((1, 0), (1, 4)), AgentTask((1, 4), (1, 0))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []


def test_pocket_yield_makes_the_swap_solvable() -> None:
    # Same head-on swap, but one pocket cell at (1,4) beside the corridor: agent 1
    # ducks into it while agent 0 sweeps past, then continues. Agent 0 keeps its
    # straight path (cost 6); agent 1's detour is forced to cost exactly 9.
    planner = PrioritizedAStar(config("prioritized_astar"))
    grid = grid_from(["#######", "####.##", ".......", "#######"])
    tasks = [AgentTask((2, 0), (2, 6)), AgentTask((2, 6), (2, 0))]
    result = planner.plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert len(result.paths[0]) - 1 == 6
    assert len(result.paths[1]) - 1 == 9


def test_goal_stays_occupied_until_arrival() -> None:
    # Agent 0 passes through (0,2) at step 2 and parks at (0,4). Agent 1's goal
    # IS (0,2): arriving before step 2 would park it on top of agent 0's
    # pass-through, so the goal guard must force arrival after it (cost 3, not 1).
    planner = PrioritizedAStar(config("prioritized_astar"))
    grid = grid_from([".....", "##.##"])
    tasks = [AgentTask((0, 0), (0, 4)), AgentTask((1, 2), (0, 2))]
    result = planner.plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert len(result.paths[1]) - 1 == 3


def test_start_cell_occupied_fails_immediately() -> None:
    # An earlier path standing on the later agent's start at t=0 is an
    # unavoidable joint conflict — no search can undo it.
    planner = PrioritizedAStar(config("prioritized_astar"))
    grid = grid_from(["..", ".."])
    tasks = [AgentTask((0, 0), (1, 1)), AgentTask((0, 0), (1, 0))]
    result = planner.plan(grid, tasks)
    assert not result.success


def test_trace_events_and_metrics() -> None:
    buf = io.StringIO()
    grid = grid_from(["..."] * 3)
    tasks = [AgentTask((2, 0), (2, 2)), AgentTask((0, 2), (2, 0))]
    planner = PrioritizedAStar(config("prioritized_astar"))
    planner.plan(grid, tasks, TraceRecorder(buf))

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = {e["event"] for e in events}
    assert {"node_expanded", "path_found", "planning_finished"} <= kinds
    # Every expansion carries its agent index and space-time step.
    expansions = [e for e in events if e["event"] == "node_expanded"]
    assert all("agent" in e and "t" in e and len(e["state"]) == 2 for e in expansions)
    finished = events[-1]
    assert finished["event"] == "planning_finished" and finished["success"] is True
    assert set(finished["metrics"]) == {"expanded_nodes", "makespan", "sum_of_costs"}
