"""dRRT*: seeded determinism (the PRNG stream — roadmap samples first, per-iteration
bias/coordinate/pick draws after — is part of the algorithm's identity), valid
collision-free joint paths on both disc scenarios, ANYTIME MONOTONICITY (a longer
budget never costs more: growth + rewiring only improve the incumbent), the same
instance verdicts as dRRT (a start/goal disc overlapping an obstacle cell, or two
discs overlapping at BOTH starts or BOTH goals — zero expansions), and an honest
budget failure that is NOT a verdict (the roadmap's component never reaches the
goal vertex; more samples might change that).

The validity check is the shared joint-edge mirror of the planner's collision
semantics (conftest.assert_joint_valid): a joint edge is legal iff the
moving-pair distance stays >= r_i + r_j for every pair, and every waypoint must
itself be free for its disc. Waiting is native here: a waypoint equal to the
previous one is a wait, and trimmed paths clamp past their end."""

from __future__ import annotations

import io
import json
from pathlib import Path

import pytest
import yaml
from conftest import (
    REPO_ROOT,
    assert_joint_valid,
    config,
    grid_from,
    write_config,
)

from mrmp.core.capabilities import Capability
from mrmp.core.params import ParamSet
from mrmp.core.trace import TraceRecorder
from mrmp.core.types import ContinuousTask
from mrmp.maps.loader import load_map, load_scenario
from mrmp.maps.occupancy_grid import OccupancyGrid2D
from mrmp.sampling import DrrtStar


def _scenario(name: str) -> tuple[OccupancyGrid2D, list[ContinuousTask]]:
    scenario = load_scenario(REPO_ROOT / "maps" / "scenarios" / f"{name}.yaml")
    grid = load_map(scenario.map_path)
    assert isinstance(grid, OccupancyGrid2D)  # the only map type; narrow for the type checker
    tasks = [
        ContinuousTask(start=spec.start, goal=spec.goal, radius=spec.radius)
        for spec in scenario.agents
    ]
    return grid, tasks


def _with_params(tmp_path: Path, **overrides: object) -> ParamSet:
    """The declared config with selected defaults overridden (same declarations,
    different defaults — the loader validates them like the real file)."""
    doc = yaml.safe_load(
        next((REPO_ROOT / "configs").rglob("drrt_star.yaml")).read_text(encoding="utf-8")
    )
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    return ParamSet.from_yaml(
        write_config(tmp_path / "drrt_star.yaml", "drrt_star", doc["params"], section="sampling")
    )


def test_contract_matches_config() -> None:
    planner = DrrtStar(config("drrt_star"))
    assert planner.name == config("drrt_star").algorithm == "drrt_star"
    assert planner.required_capabilities() == {Capability.CONTINUOUS_SPACE}


def test_same_seed_replays_byte_identical_plans() -> None:
    # The MINSTD stream (roadmap samples first, tree-phase draws after) is part of
    # the algorithm's identity: all three language engines must replay identical
    # roadmaps and trees from one seed. Same-seed reruns are the first line.
    grid, tasks = _scenario("open01_cross_discs")
    a = DrrtStar(config("drrt_star")).plan(grid, tasks)
    b = DrrtStar(config("drrt_star")).plan(grid, tasks)
    assert a.success and b.success
    assert a.paths == b.paths
    assert a.cost == pytest.approx(b.cost)
    assert a.makespan == pytest.approx(b.makespan)
    assert a.stats.expanded_nodes == b.stats.expanded_nodes


def test_open01_cross_discs_solves_with_valid_joint_paths() -> None:
    # The crossing at (4.75, 4.75): every tensor edge that would overlap the two
    # discs is rejected by the moving-pair check, so the tree only grows through
    # safe simultaneous motion — and sequencing (one robot waits while the other
    # crosses) is native: a self-loop edge IS a wait here.
    grid, tasks = _scenario("open01_cross_discs")
    result = DrrtStar(config("drrt_star")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths, [t.radius for t in tasks], grid)
    # Each path starts at its start point and ends at its goal point.
    for i, task in enumerate(tasks):
        assert result.paths[i][0] == task.start
        assert result.paths[i][-1] == task.goal


def test_open01_swap_discs_solves_with_valid_joint_paths() -> None:
    # Head-on swap on the open plane: two discs cannot pass one another on one
    # line without one of them deviating, so the sampled roadmaps must supply a
    # detour and the tree search sequences who moves when.
    grid, tasks = _scenario("open01_swap_discs")
    result = DrrtStar(config("drrt_star")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths, [t.radius for t in tasks], grid)


def test_longer_budget_never_costs_more(tmp_path: Path) -> None:
    # The anytime contract: the loop never stops on first success — growth and
    # rewiring only ever replace the incumbent with a strictly cheaper chain. Same
    # seed, same samples; the full budget must not cost more than 20 iterations
    # (and on these scenarios it measurably costs LESS).
    grid, tasks = _scenario("open01_cross_discs")
    short = DrrtStar(_with_params(tmp_path, max_iterations=20)).plan(grid, tasks)
    long_ = DrrtStar(config("drrt_star")).plan(grid, tasks)
    assert short.success and long_.success
    assert long_.cost <= short.cost + 1e-9


def test_goal_on_obstacle_fails_immediately_as_an_instance_verdict() -> None:
    # A goal whose disc overlaps an obstacle cell admits no valid configuration —
    # a verdict on the instance itself at zero expansions (unlike budget
    # exhaustion, which is never a verdict).
    planner = DrrtStar(config("drrt_star"))
    grid = grid_from(["#."])
    tasks = [ContinuousTask(start=(1.5, 0.5), goal=(0.5, 0.5), radius=0.4)]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []
    assert result.stats.expanded_nodes == 0


def test_overlapping_start_discs_fail_immediately() -> None:
    # Two start discs overlapping each other: no valid initial configuration
    # exists at all — the same kind of instance verdict, still zero expansions.
    planner = DrrtStar(config("drrt_star"))
    grid = grid_from(["#.#"])
    tasks = [
        ContinuousTask(start=(1.5, 0.5), goal=(3.5, 0.5), radius=0.4),
        ContinuousTask(start=(1.5, 0.5), goal=(1.5, 0.5), radius=0.4),
    ]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.stats.expanded_nodes == 0


def test_overlapping_goal_discs_fail_immediately() -> None:
    # The mirror verdict: two GOAL discs overlapping each other. Every plan must
    # hold both discs at their goals simultaneously, so no valid final
    # configuration exists — an instance verdict at zero expansions, not a budget
    # failure that depends on the seed. (A start overlapping the OTHER robot's
    # goal is deliberately NOT a verdict: i can vacate before j arrives.)
    planner = DrrtStar(config("drrt_star"))
    grid = grid_from(["#.", "#.", "#."])
    tasks = [
        ContinuousTask(start=(1.5, 2.5), goal=(1.5, 0.4), radius=0.4),
        ContinuousTask(start=(1.5, 1.5), goal=(1.5, 0.9), radius=0.4),
    ]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.stats.expanded_nodes == 0


def test_unreachable_goal_is_a_budget_failure_not_a_verdict(tmp_path: Path) -> None:
    # The wall separates start from goal for the single agent — but free_point
    # holds at both endpoints, so this is NOT an instance verdict: every roadmap
    # edge crossing the wall is segment_blocked, the tree grows on the start's
    # side and the goal vertex never enters T. Honest "no solution found within
    # budget" — with a grown tree (expanded_nodes > 0). The all-inf H argument
    # also runs here: every guided pick resolves to its lowest-index neighbor.
    planner = DrrtStar(_with_params(tmp_path, samples_per_robot=8, max_iterations=20))
    grid = grid_from(["#.#.#"])
    tasks = [ContinuousTask(start=(1.5, 0.5), goal=(3.5, 0.5), radius=0.4)]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []
    assert result.stats.expanded_nodes > 0


def test_trace_events_are_roadmaps_then_joint_states() -> None:
    buf = io.StringIO()
    grid, tasks = _scenario("open01_cross_discs")
    DrrtStar(config("drrt_star")).plan(grid, tasks, TraceRecorder(buf))

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = [ev["event"] for ev in events]
    # One roadmap_built per agent first (agent order), then joint-state expansions,
    # then one path_found per agent, then the finish marker.
    assert kinds[0] == "roadmap_built" and kinds[1] == "roadmap_built"
    roadmaps = [ev for ev in events if ev["event"] == "roadmap_built"]
    assert [ev["agent"] for ev in roadmaps] == [0, 1]
    for k, ev in enumerate(roadmaps):
        verts = ev["vertices"]
        # Vertices in insertion order: start first, then the goal vertex.
        assert tuple(verts[0]) == tasks[k].start
        assert tuple(verts[1]) == tasks[k].goal
        edges = ev["edges"]
        assert all(i < j for i, j in edges)  # index pairs ordered i<j...
        assert edges == sorted(edges)  # ...and the list ordered by (min, max)
    expanded = [ev for ev in events if ev["event"] == "node_expanded"]
    assert all(len(ev["state"]) == 2 * len(tasks) for ev in expanded)  # joint states
    paths = [ev for ev in events if ev["event"] == "path_found"]
    assert [ev["agent"] for ev in paths] == [0, 1]
    assert events[-1]["event"] == "planning_finished" and events[-1]["success"]
