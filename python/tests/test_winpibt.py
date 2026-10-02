"""winPIBT: PIBT's priority inheritance generalized along the time window — every
agent holds a provisional space-time path, secures its steps one by one in priority
order, and pulls parked occupants forward (retroactively) before stepping onto their
cell. The pinned numbers ARE the algorithm and the window IS the axis: at w = 1 the
negotiation is per-cell again and reproduces PIBT's pinned totals on every shared
scenario; at w = 2 lookahead already buys a step PIBT could not get; at w ≥ 3 the
greedy reservation of the highest-priority agent walls off the pocket entrance and the
swap that PIBT negotiates becomes an honest deadlock — prioritized planning's failure
mode arriving continuously as the window grows. Head-on width-1 corridors deadlock at
every window (an edge on no cycle defeats every member of this branch)."""

from __future__ import annotations

import io
import json
from pathlib import Path

import pytest
from conftest import REPO_ROOT, config, grid_from, write_config

from mrmp.core.capabilities import Capability
from mrmp.core.params import ParamSet
from mrmp.core.trace import TraceRecorder
from mrmp.core.types import AgentTask, Cell
from mrmp.decentralized import Winpibt
from mrmp.maps.loader import load_map, load_scenario
from mrmp.maps.occupancy_grid import OccupancyGrid2D


def assert_simultaneous(paths: list[list[Cell]], starts: list[Cell], goals: list[Cell]) -> None:
    """Same full-horizon contract as the PIBT page: every path spans the makespan,
    every step stays or steps one cell, and across every pair no vertex conflict and
    no swap — secured reservations plus inheritance make both structurally impossible."""
    horizon = len(paths[0]) - 1
    assert all(len(p) == horizon + 1 for p in paths), "paths must span the full horizon"
    for k, path in enumerate(paths):
        assert path[0] == starts[k] and path[-1] == goals[k]
    for t in range(1, horizon + 1):
        for path in paths:
            prev, now = path[t - 1], path[t]
            assert abs(now[0] - prev[0]) + abs(now[1] - prev[1]) <= 1, "at most one cell per step"
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            for t in range(1, horizon + 1):
                a_now, a_prev = paths[i][t], paths[i][t - 1]
                b_now, b_prev = paths[j][t], paths[j][t - 1]
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


def _per_agent(paths: list[list[Cell]]) -> list[int]:
    return [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in paths]


def _params(window: int, max_steps: int) -> list[dict[str, object]]:
    """A two-param config decl so tests can pin a window/budget without editing the
    shipped configs/decentralized/winpibt.yaml."""
    return [
        {"name": "window", "type": "int", "default": window, "min": 1,
         "description": "test window"},
        {"name": "max_steps", "type": "int", "default": max_steps, "min": 1,
         "description": "test budget"},
    ]


def test_contract_matches_config() -> None:
    planner = Winpibt(config("winpibt"))
    assert planner.name == config("winpibt").algorithm == "winpibt"
    assert planner.required_capabilities() == {Capability.DISCRETE_SPACE}


def test_single_agent_walks_the_gradient_pinned_by_tie_breaks() -> None:
    # One agent, nobody to inherit from: the ideal path is just the pinned BFS parent
    # chain (fixed neighbor order picks column 0 first, then along row 0 — landing on
    # the same route PIBT's candidate sort produced on this grid). The call count pins
    # the window: at w = 2 one winpibt() invocation every second round extends to t + 2.
    planner = Winpibt(config("winpibt"))
    result = planner.plan(grid_from(["....."] * 5), [AgentTask((4, 0), (0, 4))])
    assert result.success
    assert result.paths == [
        [(4, 0), (3, 0), (2, 0), (1, 0), (0, 0), (0, 1), (0, 2), (0, 3), (0, 4)]
    ]
    assert result.cost == pytest.approx(8.0)
    # Rounds t = 0, 2, 4, 6 extend; the goal step lands at t = 8 and the loop stops.
    assert result.stats.expanded_nodes == 4


def test_already_at_goal_is_vacuously_done() -> None:
    # start == goal: the round loop never runs; one-step path, zero cost, zero calls.
    planner = Winpibt(config("winpibt"))
    result = planner.plan(grid_from(["..", ".."]), [AgentTask((0, 0), (0, 0))])
    assert result.success
    assert result.paths == [[(0, 0)]]
    assert result.cost == pytest.approx(0.0)
    assert result.stats.expanded_nodes == 0


def test_duplicate_assignment_is_not_an_instance() -> None:
    # Same well-formedness rule as the PIBT page: unique starts AND unique goals on
    # passable cells, or it is not an instance of this problem at all.
    planner = Winpibt(config("winpibt"))
    grid = grid_from(["..", ".."])
    result = planner.plan(grid, [AgentTask((0, 0), (1, 1)), AgentTask((0, 0), (0, 1))])
    assert not result.success
    assert result.paths == []


def test_open01_cross_the_window_changes_nothing_here() -> None:
    # The crossing where PIBT pinned [16, 17]: agent 1 yields at the crossing here too.
    # Nobody ever needs more lookahead on this map, so every window from 1 up pins the
    # same totals — the window is inert when negotiation alone suffices.
    grid, tasks = _scenario("open01_cross")
    result = Winpibt(config("winpibt")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_simultaneous(result.paths, starts, goals)
    assert _per_agent(result.paths) == [16, 17]
    assert result.cost == pytest.approx(33.0)


def test_open01_swap_the_window_buys_one_step(tmp_path: Path) -> None:
    # The head-on swap on row 10. At w = 1 the negotiation is per-cell and replays
    # PIBT's run exactly (makespan 17, same [14, 16] split). At the default w = 2 agent
    # 1 sees agent 0's reservation arriving two steps ahead and ducks into row 9 one
    # step earlier — same 30 moves, makespan 16. The window buys foresight, not speed:
    # the cost total is identical, only the schedule tightens.
    grid, tasks = _scenario("open01_swap")
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]

    wide = Winpibt(config("winpibt")).plan(grid, tasks)
    assert wide.success
    assert_simultaneous(wide.paths, starts, goals)
    assert _per_agent(wide.paths) == [14, 16]
    assert wide.cost == pytest.approx(30.0)
    assert max(len(p) for p in wide.paths) - 1 == 16

    narrow = write_config(
        tmp_path / "winpibt_w1.yaml", "winpibt", _params(1, 500), section="decentralized"
    )
    per_cell = Winpibt(ParamSet.from_yaml(narrow)).plan(grid, tasks)
    assert per_cell.success
    assert_simultaneous(per_cell.paths, starts, goals)
    assert _per_agent(per_cell.paths) == [14, 16]  # PIBT's pinned split on the same map
    assert max(len(p) for p in per_cell.paths) - 1 == 17  # ... one step slower


def test_pocket_yield_makes_the_swap_executable() -> None:
    # The same head-on swap on a map WITH a pocket, at the default window: agent 1
    # reaches the pocket entrance (1,4) before agent 0's shorter window reserves it,
    # ducks into (0,4), lets agent 0 pass, and walks out the far side — [4, 6], the
    # same totals PIBT pinned. At w = 1 the per-cell game reproduces those totals too;
    # only the route inside the pocket differs ((1,4) exit instead of (0,3), because
    # winPIBT's static-route tie-break is its own pinned convention, not PIBT's sort).
    grid, tasks = _scenario("pocket01_swap")
    result = Winpibt(config("winpibt")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_simultaneous(result.paths, starts, goals)
    assert result.paths[0] == [(1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 5), (1, 5), (1, 5)]
    assert result.paths[1] == [(1, 5), (1, 4), (1, 4), (0, 4), (1, 4), (1, 3), (1, 2), (1, 1)]
    assert _per_agent(result.paths) == [4, 6]
    assert result.cost == pytest.approx(10.0)


def test_the_window_is_the_knife_edge(tmp_path: Path) -> None:
    # The pocket scenario at w = 3: agent 0 (higher priority) extends its reservation
    # three cells deep on the first round — (1,2),(1,3),(1,4) all secured by step 3 —
    # and the pocket entrance is gone before agent 1 can reach it. Agent 1 stays pinned
    # at (1,5); every later inheritance fails because vacating means swapping with a
    # move that is already secured. Honest deadlock: the greedy window IS prioritized
    # planning, and this is prioritized planning's failure on a tight swap. Same map,
    # same priorities as the passing test above — only the window moved.
    grid, tasks = _scenario("pocket01_swap")
    params = write_config(
        tmp_path / "winpibt.yaml", "winpibt", _params(3, 500), section="decentralized"
    )
    result = Winpibt(ParamSet.from_yaml(params)).plan(grid, tasks)
    assert not result.success
    assert result.paths == []


def test_head_on_corridor_swap_is_honestly_unsolvable() -> None:
    # One-wide corridor, agents swapping ends: no cell has two free neighbors at all,
    # so the retreating agent is always cornered against the wall and every claim on
    # its cell backtracks. Honest failure at every window — the same edge-on-no-cycle
    # condition that defeats PIBT; widening the window changes greediness, not topology.
    planner = Winpibt(config("winpibt"))
    grid = grid_from(["#######", "#.....#", "#######"])
    tasks = [AgentTask((1, 1), (1, 5)), AgentTask((1, 5), (1, 1))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []


def test_budget_exhaustion_is_honest_and_counted(tmp_path: Path) -> None:
    # The same deadlock at a tiny budget: failure after exactly max_steps rounds. At
    # w = 2 only rounds t ≡ 0 (mod 2) extend anything: round 0 fires both top-level
    # calls, and from round 2 on the higher-priority agent's extension drags the other
    # along through inheritance calls that fail and backtrack — 2 + 3 + 3 = 8 pinned
    # invocations before the budget cuts the run off.
    path = write_config(
        tmp_path / "winpibt.yaml", "winpibt", _params(2, 5), section="decentralized"
    )
    planner = Winpibt(ParamSet.from_yaml(path))
    grid = grid_from(["#######", "#.....#", "#######"])
    tasks = [AgentTask((1, 1), (1, 5)), AgentTask((1, 5), (1, 1))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []
    assert result.stats.expanded_nodes == 8


def test_trace_events_and_metrics() -> None:
    # No search frontier exists here either: the trace carries NO node_expanded events —
    # path_found (full-horizon, one event per agent in index order) then
    # planning_finished only. The execution replay IS the demo for this branch.
    buf = io.StringIO()
    grid = grid_from(["###..##", "#.....#", "#######"])
    tasks = [AgentTask((1, 1), (1, 5)), AgentTask((1, 5), (1, 1))]
    Winpibt(config("winpibt")).plan(grid, tasks, TraceRecorder(buf))

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = {e["event"] for e in events}
    assert kinds == {"path_found", "planning_finished"}
    founds = [e for e in events if e["event"] == "path_found"]
    assert [e["agent"] for e in founds] == [0, 1]
    assert all(len(e["path"]) == 8 for e in founds)  # makespan 7 + the start step
    finished = events[-1]
    assert finished["event"] == "planning_finished" and finished["success"] is True
    assert set(finished["metrics"]) == {"expanded_nodes", "makespan", "sum_of_costs"}
    assert finished["metrics"]["makespan"] == 7.0
    assert finished["metrics"]["sum_of_costs"] == 10.0
    # one top-level call per extension round (t = 0, 2, 4, 6) and no inheritance here:
    # the ducking agent moves before its cell is ever claimed
    assert finished["metrics"]["expanded_nodes"] == 9.0
