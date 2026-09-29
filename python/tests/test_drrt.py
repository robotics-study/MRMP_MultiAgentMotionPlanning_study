"""dRRT: seeded determinism (the PRNG stream — roadmap samples first, joint samples
after — is part of the algorithm's identity), valid collision-free joint paths on
both disc scenarios, and the two FAILURE verdicts dRRT gives — an instant instance
verdict (a start/goal disc overlapping an obstacle cell, or two start discs
overlapping each other: no valid initial configuration exists at all), and an
honest budget failure that is NOT a verdict (the roadmap's individual component
never reaches the goal vertex; more samples might change that, so dRRT can only
say "no solution found within budget").

The validity check mirrors the planner's own collision semantics: a joint edge is
legal iff the moving-pair distance stays >= r_i + r_j for every pair (strict
overlap is collision), and every waypoint must itself be free for its disc."""

from __future__ import annotations

import io
import json
from pathlib import Path

import pytest
import yaml
from conftest import REPO_ROOT, config, grid_from, write_config

from mrmp.core.capabilities import Capability
from mrmp.core.geometry import moving_pair_distance
from mrmp.core.params import ParamSet
from mrmp.core.trace import TraceRecorder
from mrmp.core.types import ContinuousTask, Point
from mrmp.maps.loader import load_map, load_scenario
from mrmp.maps.occupancy_grid import OccupancyGrid2D
from mrmp.sampling import Drrt


def _scenario(name: str) -> tuple[OccupancyGrid2D, list[ContinuousTask]]:
    scenario = load_scenario(REPO_ROOT / "maps" / "scenarios" / f"{name}.yaml")
    grid = load_map(scenario.map_path)
    assert isinstance(grid, OccupancyGrid2D)  # the only map type; narrow for the type checker
    tasks = [
        ContinuousTask(start=spec.start, goal=spec.goal, radius=spec.radius)
        for spec in scenario.agents
    ]
    return grid, tasks


def assert_joint_valid(
    paths: list[list[Point]], radii: list[float], grid: OccupancyGrid2D
) -> None:
    """Every waypoint free for its own disc, and every simultaneous motion segment
    pair apart by at least the radius sum (strict overlap is collision)."""
    horizon = max(len(p) - 1 for p in paths)
    for i, path in enumerate(paths):
        for point in path:
            assert grid.free_point(point, radii[i]), f"agent {i} waypoint on an obstacle"
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            for t in range(1, horizon + 1):
                a_prev = paths[i][t - 1] if t - 1 < len(paths[i]) else paths[i][-1]
                a_now = paths[i][t] if t < len(paths[i]) else paths[i][-1]
                b_prev = paths[j][t - 1] if t - 1 < len(paths[j]) else paths[j][-1]
                b_now = paths[j][t] if t < len(paths[j]) else paths[j][-1]
                d = moving_pair_distance(a_prev, a_now, b_prev, b_now)
                assert d >= radii[i] + radii[j], f"discs overlap during step {t}"


def _with_params(tmp_path: Path, **overrides: object) -> ParamSet:
    """The declared config with selected defaults overridden (same declarations,
    different defaults — the loader validates them like the real file)."""
    doc = yaml.safe_load(
        next((REPO_ROOT / "configs").rglob("drrt.yaml")).read_text(encoding="utf-8")
    )
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    return ParamSet.from_yaml(
        write_config(tmp_path / "drrt.yaml", "drrt", doc["params"], section="sampling")
    )


def test_contract_matches_config() -> None:
    planner = Drrt(config("drrt"))
    assert planner.name == config("drrt").algorithm == "drrt"
    assert planner.required_capabilities() == {Capability.CONTINUOUS_SPACE}


def test_same_seed_replays_byte_identical_plans() -> None:
    # The MINSTD stream (roadmap samples first, joint samples after) is part of
    # the algorithm's identity: all three language engines must replay identical
    # roadmaps and trees from one seed. Same-seed reruns are the first line.
    grid, tasks = _scenario("open01_cross_discs")
    a = Drrt(config("drrt")).plan(grid, tasks)
    b = Drrt(config("drrt")).plan(grid, tasks)
    assert a.success and b.success
    assert a.paths == b.paths
    assert a.cost == pytest.approx(b.cost)
    assert a.stats.expanded_nodes == b.stats.expanded_nodes


def test_open01_cross_discs_solves_with_valid_joint_paths() -> None:
    # The crossing at (4.75, 4.75): every tensor edge that would overlap the two
    # discs is rejected by the moving-pair check, so the tree only grows through
    # safe simultaneous motion and the prioritized connector sequences the rest.
    grid, tasks = _scenario("open01_cross_discs")
    result = Drrt(config("drrt")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths, [t.radius for t in tasks], grid)
    # Each path starts at its start point and ends at its goal point.
    for i, task in enumerate(tasks):
        assert result.paths[i][0] == task.start
        assert result.paths[i][-1] == task.goal


def test_open01_swap_discs_solves_with_valid_joint_paths() -> None:
    # Head-on swap on the open plane: no timing alone separates two discs on one
    # line, so the accepted tree edges route around each other through sampled
    # vertices and/or the priority DAG sequences the pass.
    grid, tasks = _scenario("open01_swap_discs")
    result = Drrt(config("drrt")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths, [t.radius for t in tasks], grid)


def test_goal_on_obstacle_fails_immediately_as_an_instance_verdict() -> None:
    # A goal whose disc overlaps an obstacle cell admits no valid configuration —
    # a verdict on the instance itself at zero expansions (unlike budget
    # exhaustion, which is never a verdict).
    planner = Drrt(config("drrt"))
    grid = grid_from(["#."])
    tasks = [ContinuousTask(start=(1.5, 0.5), goal=(0.5, 0.5), radius=0.4)]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []
    assert result.stats.expanded_nodes == 0


def test_overlapping_start_discs_fail_immediately() -> None:
    # Two start discs overlapping each other: no valid initial configuration
    # exists at all — the same kind of instance verdict, still zero expansions.
    planner = Drrt(config("drrt"))
    grid = grid_from(["#.#"])
    tasks = [
        ContinuousTask(start=(1.5, 0.5), goal=(3.5, 0.5), radius=0.4),
        ContinuousTask(start=(1.5, 0.5), goal=(1.5, 0.5), radius=0.4),
    ]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.stats.expanded_nodes == 0


def test_unreachable_goal_is_a_budget_failure_not_a_verdict(tmp_path) -> None:
    # The wall separates start from goal for the single agent — but free_point
    # holds at both endpoints, so this is NOT an instance verdict: every roadmap
    # edge crossing the wall is segment_blocked, the tree grows on the start's
    # side and CONNECT_TO_TARGET never reaches vertex 1. Honest "no solution
    # found within budget" — with a grown tree (expanded_nodes > 0).
    planner = Drrt(_with_params(tmp_path, samples_per_robot=8, max_rounds=2))
    grid = grid_from(["#.#.#"])
    tasks = [ContinuousTask(start=(1.5, 0.5), goal=(3.5, 0.5), radius=0.4)]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []
    assert result.stats.expanded_nodes > 0


def test_trace_events_are_roadmaps_then_joint_states() -> None:
    buf = io.StringIO()
    grid, tasks = _scenario("open01_cross_discs")
    Drrt(config("drrt")).plan(grid, tasks, TraceRecorder(buf))

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
