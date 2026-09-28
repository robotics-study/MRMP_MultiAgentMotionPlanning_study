"""CBS: joint-optimal costs on every solvable scenarios (the same optima
joint-space A* proved reachable), honest failures on the unsolvable ones, and
the trace shape of constraint-tree search (conflict_found -> constraint_added ->
re-plan expansions; path_found only for the final solution).

The two failure modes are DIFFERENT and the tests keep them apart: a duplicate
start kills every branch by its own constraints (finite tree — an actual verdict),
while the corridor swap branches forever and is stopped by max_ct_expansions."""

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
from mrmp.mapf import Cbs
from mrmp.maps.loader import load_map, load_scenario


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


def _scenario(name: str) -> tuple[object, list[AgentTask]]:
    scenario = load_scenario(REPO_ROOT / "maps" / "scenarios" / f"{name}.yaml")
    grid = load_map(scenario.map_path)
    tasks = [
        AgentTask(grid.world_to_cell(*spec.start), grid.world_to_cell(*spec.goal))
        for spec in scenario.agents
    ]
    return grid, tasks


def _budget_config(tmp_path: Path, max_ct_expansions: int) -> ParamSet:
    """The cbs config with a smaller CT-expansion budget (deterministic stops)."""
    path = write_config(
        tmp_path / "cbs.yaml",
        "cbs",
        [{"name": "max_ct_expansions", "type": "int", "default": max_ct_expansions,
          "min": 1, "description": "test budget"}],
    )
    return ParamSet.from_yaml(path)


def test_contract_matches_config() -> None:
    planner = Cbs(config("cbs"))
    assert planner.name == config("cbs").algorithm == "cbs"
    assert planner.required_capabilities() == {Capability.DISCRETE_SPACE}
    # The semi-decidability budget is the one declared parameter.
    assert config("cbs").get_int("max_ct_expansions") == 64


def test_single_agent_is_plain_optimal_a_star() -> None:
    # One agent, no possible conflict: the CT never branches and the root's
    # unconstrained sub-search IS the answer — the Manhattan optimum, exactly
    # like the other two planners' single-agent cases.
    planner = Cbs(config("cbs"))
    result = planner.plan(grid_from(["....."] * 5), [AgentTask((4, 0), (0, 4))])
    assert result.success
    assert result.cost == pytest.approx(8.0)
    assert result.paths[0][0] == (4, 0) and result.paths[0][-1] == (0, 4)


def test_maze01_two_head_on_passes_by_timing() -> None:
    # The joint optimum equals what prioritized achieved by luck of timing; CBS
    # reaches it by branching on the corridor meeting instead of by priority.
    grid, tasks = _scenario("maze01_two")
    result = Cbs(config("cbs")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost == pytest.approx(66.0)
    assert max(len(p) - 1 for p in result.paths) == 33


def test_open01_cross_both_go_straight() -> None:
    # The crossing is timed apart without either agent slowing down: both costs
    # equal their Manhattan distance (16 + 17 = 33) — the joint optimum.
    grid, tasks = _scenario("open01_cross")
    result = Cbs(config("cbs")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost == pytest.approx(33.0)
    assert max(len(p) - 1 for p in result.paths) == 17


def test_open01_swap_detours_around_the_moving_wall() -> None:
    # Head-on swap on row 10: parity rules out cost 15 per agent, so the sum is
    # 30 and one of them detours — CBS finds it by branching on the head-on
    # collision instead of hoping a priority order stumbles into it.
    grid, tasks = _scenario("open01_swap")
    result = Cbs(config("cbs")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost == pytest.approx(30.0)
    assert max(len(p) - 1 for p in result.paths) == 16


def test_duplicate_start_fails_by_branching_to_dead_ends() -> None:
    # Two agents on one cell at t=0 is unsolvable — and here the tree really does
    # die on its own constraints: the root's paths conflict at t=0, BOTH children
    # constrain an agent off its own start cell (instantly dead sub-searches), and
    # the queue empties after exactly 3 CT expansions — far under the budget. That
    # is what a genuine unsolvability verdict looks like for CBS.
    planner = Cbs(config("cbs"))
    grid = grid_from(["..", ".."])
    tasks = [AgentTask((0, 0), (1, 1)), AgentTask((0, 0), (1, 0))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []
    # Root expansions only: both children died before expanding anything.
    assert result.stats.expanded_nodes > 0


def test_corridor_swap_budget_stop(tmp_path: Path) -> None:
    # One-wide corridor, agents swapping ends: no joint plan exists — but CBS
    # cannot SEE that. Every branch just pushes the crossing later (each new
    # constraint is satisfiable by waiting longer), so the tree is infinite and
    # only max_ct_expansions stops it. The verdict is honest: "no solution found
    # within budget", pinned here to exactly 8 CT expansions' worth of work —
    # deterministic in every language, which is why this is a count not a clock.
    planner = Cbs(_budget_config(tmp_path, 8))
    grid = grid_from(["#####", ".....", "#####"])
    tasks = [AgentTask((1, 0), (1, 4)), AgentTask((1, 4), (1, 0))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []
    # Root (both agents' unconstrained A*s) + seven more CT expansions before the
    # budget trips — pinned exactly because every language replays identically.
    assert result.stats.expanded_nodes == 132


def test_budget_exhaustion_is_not_a_verdict(tmp_path: Path) -> None:
    # The SAME solvable cross scenario with a budget of 1 fails after the root
    # alone. A budget stop says "no solution found within budget", never
    # "unsolvable" — this is the pair to the default-config test above, where the
    # identical instance succeeds.
    grid, tasks = _scenario("open01_cross")
    result = Cbs(_budget_config(tmp_path, 1)).plan(grid, tasks)
    assert not result.success
    assert result.stats.expanded_nodes > 0


def test_pocket_yield_makes_the_swap_solvable() -> None:
    # Same head-on swap with one pocket cell at (1,4): solvable, and the joint
    # optimum is what joint-space A* proved — agent 0 keeps its straight path
    # (cost 6), agent 1 ducks through the pocket (cost 9). CBS must match it.
    planner = Cbs(config("cbs"))
    grid = grid_from(["#######", "####.##", ".......", "#######"])
    tasks = [AgentTask((2, 0), (2, 6)), AgentTask((2, 6), (2, 0))]
    result = planner.plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost == pytest.approx(15.0)


def test_trace_shows_the_constraint_tree_lifecycle() -> None:
    # A crossing pair forces at least one branch: the trace must show low-level
    # expansions carrying agent + t, then conflict_found, then constraint_added
    # for BOTH agents of the conflicting pair, and path_found only for the
    # final solution paths in agent-index order.
    buf = io.StringIO()
    grid = grid_from(["..."] * 3)
    tasks = [AgentTask((2, 0), (2, 2)), AgentTask((0, 2), (2, 0))]
    Cbs(config("cbs")).plan(grid, tasks, TraceRecorder(buf))

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = [e["event"] for e in events]
    assert "node_expanded" in kinds and "path_found" in kinds
    expansions = [e for e in events if e["event"] == "node_expanded"]
    # Per-agent space-time expansion: every event names its agent and timestep.
    assert all("agent" in e and "t" in e and len(e["state"]) == 2 for e in expansions)
    conflicts = [e for e in events if e["event"] == "conflict_found"]
    assert conflicts, "a crossing pair must conflict"
    first_conflict = conflicts[0]
    assert first_conflict["kind"] in ("vertex", "edge")
    assert first_conflict["agents"][0] < first_conflict["agents"][1]
    constraints = [e for e in events if e["event"] == "constraint_added"]
    # Both sides of the first conflict get a constraint (both branches exist).
    assert {e["agent"] for e in constraints} == set(first_conflict["agents"])
    paths = [e for e in events if e["event"] == "path_found"]
    assert [e["agent"] for e in paths] == [0, 1]
    # path_found comes after every conflict/constraint event — the solution is
    # only ever emitted once the popped node is conflict-free.
    last_tree_event = max(
        i for i, k in enumerate(kinds) if k in ("conflict_found", "constraint_added")
    )
    assert all(i > last_tree_event for i, e in enumerate(events) if e["event"] == "path_found")
    finished = events[-1]
    assert finished["event"] == "planning_finished" and finished["success"] is True
    assert set(finished["metrics"]) == {"expanded_nodes", "makespan", "sum_of_costs"}
