"""Joint-space A*: joint-optimal costs on every solvable scenario (including the
head-on corridor swap prioritized honestly fails), duplicate-start failure, and
the trace shape of joint-state expansion (no agent/t fields)."""

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
from mrmp.search import JointAStar


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
    assert isinstance(grid, OccupancyGrid2D)  # the only map type; narrow for the type checker
    tasks = [
        AgentTask(grid.world_to_cell(*spec.start), grid.world_to_cell(*spec.goal))
        for spec in scenario.agents
    ]
    return grid, tasks


def test_contract_matches_config() -> None:
    planner = JointAStar(config("joint_astar"))
    assert planner.name == config("joint_astar").algorithm == "joint_astar"
    assert planner.required_capabilities() == {Capability.DISCRETE_SPACE}


def test_single_agent_is_plain_optimal_a_star() -> None:
    # One agent: the joint state degenerates to a plain cell and the search must
    # return the Manhattan optimum, exactly like prioritized's single-agent case.
    planner = JointAStar(config("joint_astar"))
    result = planner.plan(grid_from(["....."] * 5), [AgentTask((4, 0), (0, 4))])
    assert result.success
    assert result.cost == pytest.approx(8.0)
    assert result.paths[0][0] == (4, 0) and result.paths[0][-1] == (0, 4)


def test_maze01_two_head_on_passes_by_timing() -> None:
    # The coupled optimum equals what prioritized achieved by luck of timing:
    # both agents still achieve their unconstrained shortest cost (33 + 33 = 66).
    grid, tasks = _scenario("maze01_two")
    result = JointAStar(config("joint_astar")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost == pytest.approx(66.0)
    assert max(len(p) - 1 for p in result.paths) == 33


def test_open01_cross_both_go_straight() -> None:
    # The crossing is timed apart without either agent slowing down: both costs
    # equal their Manhattan distance (16 + 17 = 33) — the joint optimum.
    grid, tasks = _scenario("open01_cross")
    result = JointAStar(config("joint_astar")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost == pytest.approx(33.0)
    assert max(len(p) - 1 for p in result.paths) == 17


def test_open01_swap_detours_around_the_moving_wall() -> None:
    # Head-on swap on row 10: parity rules out cost 15, so the detour costs
    # exactly 16 and the sum 30 is the joint optimum (prioritized hit it too —
    # here that is a theorem, not luck).
    grid, tasks = _scenario("open01_swap")
    result = JointAStar(config("joint_astar")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost == pytest.approx(30.0)
    assert max(len(p) - 1 for p in result.paths) == 16


def test_head_on_corridor_swap_is_unsolvable() -> None:
    # One-wide corridor, agents swapping ends: no wait or detour exists anywhere,
    # so the complete search exhausts its state space and honestly reports
    # failure — this is exactly where prioritized's incompleteness was a limit.
    planner = JointAStar(config("joint_astar"))
    grid = grid_from(["#####", ".....", "#####"])
    tasks = [AgentTask((1, 0), (1, 4)), AgentTask((1, 4), (1, 0))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []


def test_pocket_yield_makes_the_swap_solvable() -> None:
    # Same head-on swap with one pocket cell at (1,4): solvable, and the joint
    # optimum equals what prioritized achieved by priority luck — agent 0 keeps
    # its straight path (cost 6), agent 1 ducks through the pocket (cost 9).
    planner = JointAStar(config("joint_astar"))
    grid = grid_from(["#######", "####.##", ".......", "#######"])
    tasks = [AgentTask((2, 0), (2, 6)), AgentTask((2, 6), (2, 0))]
    result = planner.plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert len(result.paths[0]) - 1 == 6
    assert len(result.paths[1]) - 1 == 9


def test_parked_goal_blocks_the_only_lane() -> None:
    # Agent 0's goal is the corridor cell agent 1 must pass through to reach its
    # own goal. Stay-at-goal semantics are shared across both planners: once
    # agent 0 parks there forever, no joint plan exists — the complete search
    # exhausts and says so (after actually searching, unlike the duplicate start).
    planner = JointAStar(config("joint_astar"))
    grid = grid_from(["#####", ".....", "#####"])
    tasks = [AgentTask((1, 0), (1, 2)), AgentTask((1, 4), (1, 0))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.stats.expanded_nodes > 0


def test_duplicate_start_fails_immediately() -> None:
    # Two agents on one cell at t=0 is no joint state at all — the search does
    # not even expand.
    planner = JointAStar(config("joint_astar"))
    grid = grid_from(["..", ".."])
    tasks = [AgentTask((0, 0), (1, 1)), AgentTask((0, 0), (1, 0))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.stats.expanded_nodes == 0


def test_trace_events_are_joint_states_without_agent_or_step_fields() -> None:
    buf = io.StringIO()
    grid = grid_from(["..."] * 3)
    tasks = [AgentTask((2, 0), (2, 2)), AgentTask((0, 2), (2, 0))]
    planner = JointAStar(config("joint_astar"))
    planner.plan(grid, tasks, TraceRecorder(buf))

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = {e["event"] for e in events}
    assert {"node_expanded", "path_found", "planning_finished"} <= kinds
    # Joint-state expansions carry the flattened position tuple and NO agent/t —
    # a joint state has no single owning agent and no single timestep.
    expansions = [e for e in events if e["event"] == "node_expanded"]
    assert all("agent" not in e and "t" not in e for e in expansions)
    assert all(len(e["state"]) == 4 for e in expansions)
    finished = events[-1]
    assert finished["event"] == "planning_finished" and finished["success"] is True
    assert set(finished["metrics"]) == {"expanded_nodes", "makespan", "sum_of_costs"}
