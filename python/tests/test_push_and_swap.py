"""Push and Swap: push-only routing at unconstrained cost, swap-solved head-on
swaps (and the honest failure where no 2x2 block exists), single-move validity of
the emitted assignment sequence, and the trace shape of a search-free planner."""

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
from mrmp.search import PushAndSwap


def assert_single_moves(paths: list[list[Cell]], starts: list[Cell], goals: list[Cell]) -> None:
    """Push and Swap emits full-horizon paths: every agent's path spans the whole
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
    planner = PushAndSwap(config("push_and_swap"))
    assert planner.name == config("push_and_swap").algorithm == "push_and_swap"
    assert planner.required_capabilities() == {Capability.DISCRETE_SPACE}


def test_single_agent_walks_its_shortest_path() -> None:
    # One agent, nobody to push: the BFS shortest path walked straight through —
    # cost equals the Manhattan distance exactly like the prioritized baseline.
    planner = PushAndSwap(config("push_and_swap"))
    result = planner.plan(grid_from(["....."] * 5), [AgentTask((4, 0), (0, 4))])
    assert result.success
    assert_single_moves(result.paths, [(4, 0)], [(0, 4)])
    assert result.cost == pytest.approx(8.0)


def test_already_at_goal_is_vacuously_done() -> None:
    # start == goal: the while-loop body never runs; cost 0, one-step path.
    planner = PushAndSwap(config("push_and_swap"))
    result = planner.plan(grid_from(["..", ".."]), [AgentTask((0, 0), (0, 0))])
    assert result.success
    assert result.paths == [[(0, 0)]]
    assert result.cost == pytest.approx(0.0)


def test_duplicate_assignment_is_not_an_instance() -> None:
    # The paper's assignment is injective by definition (unique starts AND unique
    # targets). A violating input is not an instance of this problem at all — the
    # planner reports it honestly instead of running on a contradiction.
    planner = PushAndSwap(config("push_and_swap"))
    grid = grid_from(["..", ".."])
    result = planner.plan(grid, [AgentTask((0, 0), (1, 1)), AgentTask((0, 0), (0, 1))])
    assert not result.success
    assert result.paths == []


def test_open01_cross_push_only_at_unconstrained_cost() -> None:
    # Crossing paths with nobody in the other's way long enough to matter: agent 0
    # walks its straight path (agent 1 is pushed out of row 10 first), then agent 1
    # walks down column 9 — both at exactly their Manhattan distance (16 + 17 = 33).
    grid, tasks = _scenario("open01_cross")
    result = PushAndSwap(config("push_and_swap")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_single_moves(result.paths, starts, goals)
    assert result.cost == pytest.approx(33.0)
    per_agent = [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in result.paths]
    assert per_agent == [16, 17]


def test_open01_swap_solved_by_a_real_swap() -> None:
    # Head-on swap on row 10 — the instance prioritized planning cannot solve at
    # any cost. Here agent 1 is chain-pushed off the row and the pair exchanges at
    # a free 2x2 block; the maneuvers cost more than the joint optimum (30):
    # 18 + 20 = 38 moves total, honestly suboptimal.
    grid, tasks = _scenario("open01_swap")
    result = PushAndSwap(config("push_and_swap")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_single_moves(result.paths, starts, goals)
    per_agent = [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in result.paths]
    assert per_agent == [18, 20]
    assert result.cost == pytest.approx(38.0)


def test_maze01_two_solved_through_one_corridor_gap() -> None:
    # Both agents thread the single corridor gap in opposite directions (37 + 37 =
    # 74 — above the joint optimum 66; push and swap buys completeness, not speed).
    grid, tasks = _scenario("maze01_two")
    result = PushAndSwap(config("push_and_swap")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_single_moves(result.paths, starts, goals)
    assert result.cost == pytest.approx(74.0)


def test_head_on_corridor_swap_is_honestly_unsolvable() -> None:
    # One-wide corridor, agents swapping ends: a swap physically needs a free 2x2
    # block (grid graphs have no triangles), and a width-1 corridor has none — the
    # planner fails honestly even though n = |V| - 2 holds here.
    planner = PushAndSwap(config("push_and_swap"))
    grid = grid_from(["#######", "#.....#", "#######"])
    tasks = [AgentTask((1, 1), (1, 5)), AgentTask((1, 5), (1, 1))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []


def test_pocket_yield_makes_the_swap_executable() -> None:
    # The same head-on swap on a map with one pocket: the 2x2 block
    # {(0,3),(0,4),(1,3),(1,4)} hosts the exchange (8 + 6 = 14 moves).
    planner = PushAndSwap(config("push_and_swap"))
    grid = grid_from(["###..##", "#.....#", "#######"])
    tasks = [AgentTask((1, 1), (1, 5)), AgentTask((1, 5), (1, 1))]
    result = planner.plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_single_moves(result.paths, starts, goals)
    per_agent = [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in result.paths]
    assert per_agent == [8, 6]


def test_refilled_w2_fails_the_attempt_not_the_planner() -> None:
    # Three agents on the pocket map: a third body makes a new case reachable —
    # clearing w4 steps its occupant into the just-cleared w2, refilling it. That
    # attempt now fails like any other clear failure (the old code moved into the
    # occupied cell and tripped the move invariant). The planner keeps exploring
    # candidates and solves: 21 + 16 + 10 = 47 moves, honestly suboptimal.
    planner = PushAndSwap(config("push_and_swap"))
    grid = grid_from(["###..##", "#.....#", "#######"])
    tasks = [
        AgentTask((1, 5), (1, 2)),
        AgentTask((1, 3), (1, 1)),
        AgentTask((1, 1), (1, 3)),
    ]
    result = planner.plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_single_moves(result.paths, starts, goals)
    per_agent = [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in result.paths]
    assert per_agent == [21, 16, 10]


def test_trace_events_and_metrics() -> None:
    # No search frontier exists here: the trace carries NO node_expanded events —
    # path_found (full-horizon, one event per agent) then planning_finished only.
    buf = io.StringIO()
    grid = grid_from(["###..##", "#.....#", "#######"])
    tasks = [AgentTask((1, 1), (1, 5)), AgentTask((1, 5), (1, 1))]
    PushAndSwap(config("push_and_swap")).plan(grid, tasks, TraceRecorder(buf))

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = {e["event"] for e in events}
    assert kinds == {"path_found", "planning_finished"}
    founds = [e for e in events if e["event"] == "path_found"]
    assert [e["agent"] for e in founds] == [0, 1]
    assert all(len(e["path"]) == 15 for e in founds)  # makespan 14 + the start step
    finished = events[-1]
    assert finished["event"] == "planning_finished" and finished["success"] is True
    assert set(finished["metrics"]) == {"expanded_nodes", "makespan", "sum_of_costs"}
    assert finished["metrics"]["makespan"] == 14.0
    assert finished["metrics"]["sum_of_costs"] == 14.0
