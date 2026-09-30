"""Push and Rotate: the priority branch's decision procedure — solvable instances
resolve through push, swap (generalized to degree >= 3 junctions) or rotate (cycle
rotation), unsolvable ones (a head-on swap on a tree) fail honestly; single-move
validity of the emitted assignment sequence, and the trace shape of a search-free
planner. Where both algorithms solve the same instance, costs are pinned against
Push and Swap's numbers (the suboptimality trade differs per map)."""

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
from mrmp.search import PushAndRotate, PushAndSwap


def assert_single_moves(paths: list[list[Cell]], starts: list[Cell], goals: list[Cell]) -> None:
    """Push and Rotate emits full-horizon paths: every agent's path spans the whole
    makespan, at every step EXACTLY one agent moves (the assignment changes by one
    agent), always into a 4-adjacent cell. There is never simultaneous motion —
    this replaces the pairwise conflict checks of the simultaneous-move planners."""
    horizon = len(paths[0]) - 1
    assert all(len(p) == horizon + 1 for p in paths), "paths must span the full horizon"
    for k, path in enumerate(paths):
        assert path[0] == starts[k] and path[-1] == goals[k]
    for t in range(1, horizon + 1):
        movers = [k for k in range(len(paths)) if paths[k][t] != paths[k][t - 1]]
        assert len(movers) == 1, f"step {t} moved {len(movers)} agents at once"
        k = movers[0]
        prev, now = paths[k][t - 1], paths[k][t]
        assert abs(now[0] - prev[0]) + abs(now[1] - prev[1]) == 1, "move must be one grid step"


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
    planner = PushAndRotate(config("push_and_rotate"))
    assert planner.name == config("push_and_rotate").algorithm == "push_and_rotate"
    assert planner.required_capabilities() == {Capability.DISCRETE_SPACE}


def test_single_agent_walks_its_shortest_path() -> None:
    # One agent, nobody to push: the BFS shortest path walked straight through —
    # cost equals the Manhattan distance exactly like the prioritized baseline.
    planner = PushAndRotate(config("push_and_rotate"))
    result = planner.plan(grid_from(["....."] * 5), [AgentTask((4, 0), (0, 4))])
    assert result.success
    assert_single_moves(result.paths, [(4, 0)], [(0, 4)])
    assert result.cost == pytest.approx(8.0)


def test_already_at_goal_is_vacuously_done() -> None:
    # start == goal: the solve loop's walk never runs; cost 0, one-step path.
    planner = PushAndRotate(config("push_and_rotate"))
    result = planner.plan(grid_from(["..", ".."]), [AgentTask((0, 0), (0, 0))])
    assert result.success
    assert result.paths == [[(0, 0)]]
    assert result.cost == pytest.approx(0.0)


def test_duplicate_assignment_is_not_an_instance() -> None:
    # The paper's assignment is injective by definition (unique starts AND unique
    # targets). A violating input is not an instance of this problem at all — the
    # planner reports it honestly instead of running on a contradiction.
    planner = PushAndRotate(config("push_and_rotate"))
    grid = grid_from(["..", ".."])
    result = planner.plan(grid, [AgentTask((0, 0), (1, 1)), AgentTask((0, 0), (0, 1))])
    assert not result.success
    assert result.paths == []


def test_open01_cross_push_only_at_unconstrained_cost() -> None:
    # Crossing paths with nobody in the other's way long enough to matter: agent 0
    # walks its straight path (agent 1 is pushed out of row 10 first), then agent 1
    # walks down column 9 — both at exactly their Manhattan distance (16 + 17 = 33,
    # the same numbers Push and Swap reaches: no swap or rotate fires here).
    grid, tasks = _scenario("open01_cross")
    result = PushAndRotate(config("push_and_rotate")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_single_moves(result.paths, starts, goals)
    per_agent = [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in result.paths]
    assert per_agent == [16, 17]
    assert result.cost == pytest.approx(33.0)


def test_open01_swap_solved_by_a_real_swap() -> None:
    # Head-on swap on row 10 — the instance prioritized planning cannot solve at
    # any cost. Here agent 1 is chain-pushed off the row and the pair exchanges at
    # a junction; the generalized exchange costs LESS than Push and Swap's rigid
    # 2x2-block maneuver (18 + 18 = 36 vs 38, both above the joint optimum 30).
    grid, tasks = _scenario("open01_swap")
    result = PushAndRotate(config("push_and_rotate")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_single_moves(result.paths, starts, goals)
    per_agent = [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in result.paths]
    assert per_agent == [18, 18]
    assert result.cost == pytest.approx(36.0)


def test_maze01_two_solved_through_one_corridor_gap() -> None:
    # Both agents thread the single corridor gap in opposite directions (37 + 35 =
    # 72 — below Push and Swap's 74, above the joint optimum 66; completeness, not
    # speed). The subgraph decomposition merges along planks within m - 2 here.
    grid, tasks = _scenario("maze01_two")
    result = PushAndRotate(config("push_and_rotate")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_single_moves(result.paths, starts, goals)
    per_agent = [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in result.paths]
    assert per_agent == [37, 35]
    assert result.cost == pytest.approx(72.0)


def test_head_on_corridor_swap_is_honestly_unsolvable() -> None:
    # One-wide corridor, agents swapping ends: no subgraph hosts a junction, so no
    # exchange site exists anywhere and the rotate has no cycle to ride — the
    # decision procedure reports unsolvable (not a search failure), still reporting
    # the BFS expansions counted up to that verdict.
    planner = PushAndRotate(config("push_and_rotate"))
    grid, tasks = _scenario("corridor01_head_on")
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []
    assert result.stats.expanded_nodes == 14


def test_tee01_head_on_is_the_completeness_boundary() -> None:
    # The SAME head-on swap on the SAME corridor with one junction cell added at
    # (1,3): Push and Swap has no 2x2 block and fails honestly; Push and Rotate's
    # exchange fires on the degree-3 vertex and solves it at 8 + 8 = 16 moves.
    grid, tasks = _scenario("tee01_head_on")
    result = PushAndRotate(config("push_and_rotate")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_single_moves(result.paths, starts, goals)
    per_agent = [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in result.paths]
    assert per_agent == [8, 8]
    assert result.cost == pytest.approx(16.0)
    # ... and the incomplete branch really does give up on this exact instance.
    swap_result = PushAndSwap(config("push_and_swap")).plan(grid, tasks)
    assert not swap_result.success


def test_pocket01_swap_beats_the_rigid_block_swap() -> None:
    # The pocket map's head-on swap: 6 + 6 = 12 moves — the generalized exchange
    # beats Push and Swap's block-bound maneuver on the same instance (14).
    grid, tasks = _scenario("pocket01_swap")
    result = PushAndRotate(config("push_and_rotate")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_single_moves(result.paths, starts, goals)
    per_agent = [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in result.paths]
    assert per_agent == [6, 6]
    assert result.cost == pytest.approx(12.0)


def test_pocket01_rotate_rotation_beats_swap_only() -> None:
    # Three agents on the pocket map: agent 0 walks left through (1,3), and the
    # resolving agents' still-open trails form a cycle — ROTATE cascades every
    # rider one step forward instead of swapping pairs. The rotation cuts Push and
    # Swap's 47 moves (21 + 16 + 10) to 15 + 10 + 10 = 35.
    grid, tasks = _scenario("pocket01_rotate")
    result = PushAndRotate(config("push_and_rotate")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_single_moves(result.paths, starts, goals)
    per_agent = [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in result.paths]
    assert per_agent == [15, 10, 10]
    assert result.cost == pytest.approx(35.0)


def test_trace_events_and_metrics() -> None:
    # No search frontier exists here: the trace carries NO node_expanded events —
    # path_found (full-horizon, one event per agent) then planning_finished only.
    buf = io.StringIO()
    grid, tasks = _scenario("pocket01_swap")
    PushAndRotate(config("push_and_rotate")).plan(grid, tasks, TraceRecorder(buf))

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = {e["event"] for e in events}
    assert kinds == {"path_found", "planning_finished"}
    founds = [e for e in events if e["event"] == "path_found"]
    assert [e["agent"] for e in founds] == [0, 1]
    assert all(len(e["path"]) == 13 for e in founds)  # makespan 12 + the start step
    finished = events[-1]
    assert finished["event"] == "planning_finished" and finished["success"] is True
    assert set(finished["metrics"]) == {"expanded_nodes", "makespan", "sum_of_costs"}
    assert finished["metrics"]["makespan"] == 12.0
    assert finished["metrics"]["sum_of_costs"] == 12.0
    assert finished["metrics"]["expanded_nodes"] == 60.0
