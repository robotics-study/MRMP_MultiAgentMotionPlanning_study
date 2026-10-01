"""PIBT: per-timestep priority-inheritance decisions on a grid — simultaneous motion
with no vertex conflict and no swap by construction, honest deadlock (honest failure)
wherever an edge lies on no cycle, and the pinned determinism of everything the paper
leaves free (ε = index order, candidate order distance → occupancy → row-major).

The pinned numbers ARE the algorithm: open01_cross shows pure gradient descent with
yielding (33 moves — the same total the offline planners needed), open01_swap shows
negotiated yielding hitting the joint optimum 30 that the rigid push/swap primitives
paid 38 and 36 for, pocket01_swap shows the duck-into-the-pocket yield, and both
head-on scenarios deadlock honestly — a width-1 corridor edge lies on no cycle, so the
blocked agent has nowhere to vacate to (the exact condition Push and Rotate's rotate
primitive existed to repair; PIBT has no such primitive)."""

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
from mrmp.decentralized import Pibt
from mrmp.maps.loader import load_map, load_scenario
from mrmp.maps.occupancy_grid import OccupancyGrid2D


def assert_simultaneous(paths: list[list[Cell]], starts: list[Cell], goals: list[Cell]) -> None:
    """PIBT emits full-horizon paths: every agent's path spans the whole makespan,
    every step every agent either stays or steps one cell — and across every pair,
    no vertex conflict (same cell at one step) and no swap (pair exchanges cells),
    which is exactly what inheritance + claim-exclusion make structurally impossible."""
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


def test_contract_matches_config() -> None:
    planner = Pibt(config("pibt"))
    assert planner.name == config("pibt").algorithm == "pibt"
    assert planner.required_capabilities() == {Capability.DISCRETE_SPACE}


def test_single_agent_walks_the_gradient_pinned_by_tie_breaks() -> None:
    # One agent, nobody to inherit from: pure gradient descent. The path itself pins
    # the tie-break — distance-to-goal alone admits any monotone 8-move route, and
    # row-major order on equal distances picks straight up column 0 first, then along
    # row 0 (a cell at row 0 always sorts before a same-distance cell at row > 0).
    planner = Pibt(config("pibt"))
    result = planner.plan(grid_from(["....."] * 5), [AgentTask((4, 0), (0, 4))])
    assert result.success
    assert result.paths == [
        [(4, 0), (3, 0), (2, 0), (1, 0), (0, 0), (0, 1), (0, 2), (0, 3), (0, 4)]
    ]
    assert result.cost == pytest.approx(8.0)
    # One decision invocation per timestep: the single agent never inherits anyone.
    assert result.stats.expanded_nodes == 8


def test_already_at_goal_is_vacuously_done() -> None:
    # start == goal: the timestep loop never runs; one-step path, zero cost, and
    # zero decision invocations (the loop body is where every call lives).
    planner = Pibt(config("pibt"))
    result = planner.plan(grid_from(["..", ".."]), [AgentTask((0, 0), (0, 0))])
    assert result.success
    assert result.paths == [[(0, 0)]]
    assert result.cost == pytest.approx(0.0)
    assert result.stats.expanded_nodes == 0


def test_duplicate_assignment_is_not_an_instance() -> None:
    # The paper's instance is well-formed by definition (unique starts AND unique
    # goals on passable cells). A violating input is not an instance of this problem
    # at all — the planner reports it honestly instead of running on a contradiction.
    planner = Pibt(config("pibt"))
    grid = grid_from(["..", ".."])
    result = planner.plan(grid, [AgentTask((0, 0), (1, 1)), AgentTask((0, 0), (0, 1))])
    assert not result.success
    assert result.paths == []


def test_open01_cross_pure_gradient_with_yielding() -> None:
    # Crossing paths: both agents just descend their distance gradient; the pinned
    # ε order (agent 0 highest) makes agent 1 yield at the crossing instead of
    # waiting for a planned push. The TOTAL equals the offline planners' unconstrained
    # cost (16 + 17 = 33), but nobody was ever pushed — every move was self-chosen.
    grid, tasks = _scenario("open01_cross")
    result = Pibt(config("pibt")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_simultaneous(result.paths, starts, goals)
    per_agent = [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in result.paths]
    assert per_agent == [16, 17]
    assert result.cost == pytest.approx(33.0)


def test_open01_swap_yielding_hits_the_joint_optimum() -> None:
    # Head-on swap on row 10 — the instance prioritized planning cannot solve at any
    # cost and Push and Swap paid 38 (rigid chain-push) / Rotate 36 for. PIBT needs no
    # primitive: agent 1 simply keeps walking while agent 0's higher priority makes it
    # flow around, landing on exactly the joint optimum 14 + 16 = 30.
    grid, tasks = _scenario("open01_swap")
    result = Pibt(config("pibt")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_simultaneous(result.paths, starts, goals)
    per_agent = [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in result.paths]
    assert per_agent == [14, 16]
    assert result.cost == pytest.approx(30.0)


def test_pocket_yield_makes_the_swap_executable() -> None:
    # The same head-on swap on a map WITH a pocket: agent 1 inherits the claim and
    # ducks into (0,4), lets agent 0 pass, then walks out the far side — 4 + 6 = 10
    # moves. The pocket edge lies on a cycle; that is all completeness ever asks for.
    grid, tasks = _scenario("pocket01_swap")
    result = Pibt(config("pibt")).plan(grid, tasks)
    assert result.success
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    assert_simultaneous(result.paths, starts, goals)
    per_agent = [sum(1 for t in range(1, len(p)) if p[t] != p[t - 1]) for p in result.paths]
    assert per_agent == [4, 6]
    assert result.cost == pytest.approx(10.0)


def test_tee_junction_head_on_deadlocks_honestly() -> None:
    # The SAME coordinates where Push and Rotate's rotate primitive paid 8 + 8 = 16:
    # a single dead-end stub at (0,3). A retreating agent can only vacate into a cell
    # adjacent to the one it holds — when its back is the corridor end, the stub is
    # not adjacent to where the inheritance chain reaches it, so the claim backtracks
    # and both agents freeze. The edge on no cycle defeats PIBT exactly where Rotate
    # had a repair; PIBT has none, and the budget turns deadlock into honest failure.
    planner = Pibt(config("pibt"))
    grid = grid_from(["###.###", "#.....#", "#######"])
    tasks = [AgentTask((1, 1), (1, 5)), AgentTask((1, 5), (1, 1))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []


def test_head_on_corridor_swap_is_honestly_unsolvable() -> None:
    # One-wide corridor, agents swapping ends: no cell has two free neighbors at all,
    # so the retreating agent is always cornered against the wall and every claim on
    # its cell backtracks. Honest failure — a budget exhaustion, never a proof of
    # unsolvability (the same convention as CBS's tree budget).
    planner = Pibt(config("pibt"))
    grid = grid_from(["#######", "#.....#", "#######"])
    tasks = [AgentTask((1, 1), (1, 5)), AgentTask((1, 5), (1, 1))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []


def test_budget_exhaustion_is_honest_and_counted(tmp_path: Path) -> None:
    # Same deadlock as the corridor scenario at a tiny budget: failure after exactly
    # max_steps timesteps × 2 agents = 10 decision invocations, reported honestly.
    path = write_config(
        tmp_path / "pibt.yaml",
        "pibt",
        [{"name": "max_steps", "type": "int", "default": 5,
          "min": 1, "description": "test budget"}],
        section="decentralized",
    )
    planner = Pibt(ParamSet.from_yaml(path))
    grid = grid_from(["#######", "#.....#", "#######"])
    tasks = [AgentTask((1, 1), (1, 5)), AgentTask((1, 5), (1, 1))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []
    assert result.stats.expanded_nodes == 10


def test_trace_events_and_metrics() -> None:
    # No search frontier exists here: the trace carries NO node_expanded events —
    # path_found (full-horizon, one event per agent in index order) then
    # planning_finished only. The execution replay IS the demo for this branch.
    buf = io.StringIO()
    grid = grid_from(["###..##", "#.....#", "#######"])
    tasks = [AgentTask((1, 1), (1, 5)), AgentTask((1, 5), (1, 1))]
    Pibt(config("pibt")).plan(grid, tasks, TraceRecorder(buf))

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
    # one top-level decision per agent per step, no inheritance chain here: 2 × 7
    assert finished["metrics"]["expanded_nodes"] == 14.0
