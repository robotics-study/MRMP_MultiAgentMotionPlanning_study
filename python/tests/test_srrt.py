"""sRRT: seeded determinism (the PRNG is part of the algorithm's identity), valid
conflict-free joint paths on every scenario, honest sub-optimality where coupling
forces detours above the coupled optimum, and the two FAILURE verdicts sRRT gives
— an instant instance verdict when a goal is unreachable from its start (the
individual policy cannot even be defined there), and an honest budget failure
that is NOT a verdict on the instance.

The search branch proves 66/33/30 are the optima on these scenarios; sRRT may
only match or exceed them. Where the two individual policies happen to pass each
other time-offset (open01_cross, maze01_two), no collision ever fires, sRRT never
couples, and the result is exactly the sum of the individually-optimal paths;
where a head-on meeting is unavoidable (open01_swap), coupling fires and the
sample-driven coupled phase lands above the coupled optimum — the honest price
of sampling (sRRT's completeness is unproven, its optimality nonexistent)."""

from __future__ import annotations

import io
import json
from pathlib import Path

import pytest
import yaml
from conftest import REPO_ROOT, config, grid_from, write_config

from mrmp.core.capabilities import Capability
from mrmp.core.params import ParamSet
from mrmp.core.trace import TraceRecorder
from mrmp.core.types import AgentTask, Cell
from mrmp.maps.loader import load_map, load_scenario
from mrmp.maps.occupancy_grid import OccupancyGrid2D
from mrmp.sampling import Srrt


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


def _with_params(tmp_path: Path, **overrides: object) -> ParamSet:
    """The declared config with selected defaults overridden (same declarations,
    different defaults — the loader validates them like the real file)."""
    doc = yaml.safe_load(
        next((REPO_ROOT / "configs").rglob("srrt.yaml")).read_text(encoding="utf-8")
    )
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    return ParamSet.from_yaml(
        write_config(tmp_path / "srrt.yaml", "srrt", doc["params"], section="sampling")
    )


def test_contract_matches_config() -> None:
    planner = Srrt(config("srrt"))
    assert planner.name == config("srrt").algorithm == "srrt"
    assert planner.required_capabilities() == {Capability.DISCRETE_SPACE}


def test_single_agent_follows_its_bfs_policy_on_every_seed(
    tmp_path,
) -> None:
    # One agent: no robot-robot collision can ever fire, so the collision set
    # stays empty forever and every expansion just takes one more policy step —
    # the tree degenerates to a single line along the BFS-tree policy from start
    # to goal. Sampling only decides WHEN the line extends, never where; cost is
    # the individually-optimal Manhattan length on an open grid, seed-independently.
    grid = grid_from(["....."] * 5)
    for seed in (42, 7, 123):
        planner = Srrt(_with_params(tmp_path, seed=seed))
        result = planner.plan(grid, [AgentTask((4, 0), (0, 4))])
        assert result.success
        assert result.cost == pytest.approx(8.0)
        assert result.paths[0][0] == (4, 0) and result.paths[0][-1] == (0, 4)


def test_same_seed_replays_byte_identical_plans() -> None:
    # The MINSTD stream is part of the algorithm's identity: all three language
    # engines must replay identical trees from one seed. Same-seed reruns are the
    # first line of that contract.
    grid, tasks = _scenario("open01_swap")
    a = Srrt(config("srrt")).plan(grid, tasks)
    b = Srrt(config("srrt")).plan(grid, tasks)
    assert a.success and b.success
    assert a.paths == b.paths
    assert a.cost == pytest.approx(b.cost)
    assert a.stats.expanded_nodes == b.stats.expanded_nodes


def test_open01_cross_stays_decoupled_at_the_tight_bound() -> None:
    # The crossing is timed apart by the policies themselves: both BFS policy
    # paths pass the crossing cell at different steps, so no collision ever
    # fires, the collision sets stay empty (pure decoupled execution), and each
    # agent's path is its individually-optimal one — 16 + 17 = 33, exactly what
    # joint-space A* proves optimal. Where the metric's lower bound is tight,
    # sRRT lands on it by construction, not by luck of sampling.
    grid, tasks = _scenario("open01_cross")
    result = Srrt(config("srrt")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost == pytest.approx(33.0)
    assert max(len(p) - 1 for p in result.paths) == 17


def test_open01_swap_couples_and_solves_honestly_suboptimally() -> None:
    # Head-on along one open row: no timing offset can save two policies walking
    # into each other, so the first head-on step fires an informative collision,
    # both robots join every ancestor's collision set, and from there sRRT IS a
    # plain joint-space RRT steered by samples. The search branch proves 30 is
    # optimal; this seeded run lands above it — the honest price of sampling with
    # an unproven guarantee (the paper leaves even probabilistic completeness
    # unproven).
    grid, tasks = _scenario("open01_swap")
    result = Srrt(config("srrt")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost > 30.0


def test_maze01_two_passes_the_corridor_by_timing_not_coupling() -> None:
    # The corridor swap the search branch solves at cost 66: here each agent's
    # individually-optimal policy path (33 steps each) passes the shared corridor
    # one step apart, so no collision ever fires and sRRT never couples — the
    # result is exactly the sum of individual optima, which happens to EQUAL the
    # coupled optimum. That is timing luck on this map geometry, not a property
    # of sRRT: the algorithm cannot promise it on other instances.
    grid, tasks = _scenario("maze01_two")
    result = Srrt(config("srrt")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost == pytest.approx(66.0)


def test_unreachable_goal_fails_immediately_as_an_instance_verdict() -> None:
    # A goal whose free component does not contain the start makes the individual
    # policy undefined at the start cell — no collision-free joint trajectory can
    # exist for ANY sample, so sRRT stops with a verdict on the instance itself
    # (unlike MA-RRT*, which can only ever say "budget exhausted").
    planner = Srrt(config("srrt"))
    grid = grid_from(["######", "#.#..#", "######"])
    result = planner.plan(grid, [AgentTask((1, 1), (1, 4))])
    assert not result.success
    assert result.paths == []
    assert result.stats.expanded_nodes == 0


def test_budget_exhaustion_is_not_an_instance_verdict(tmp_path) -> None:
    # The same solvable instance with a tiny budget fails too — but as "no
    # solution found within budget", not an instance verdict (the tree grew, the
    # policies exist; the goal tuple just never became a vertex in 3 draws).
    grid, tasks = _scenario("open01_swap")
    planner = Srrt(_with_params(tmp_path, max_iterations=3))
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []
    assert result.stats.expanded_nodes > 0


def test_duplicate_start_fails_immediately() -> None:
    # Two agents on one cell at t=0 is no joint state at all — the tree never
    # grows past a root that cannot exist (same honest verdict as the search side).
    planner = Srrt(config("srrt"))
    grid = grid_from(["..", ".."])
    tasks = [AgentTask((0, 0), (1, 1)), AgentTask((0, 0), (1, 0))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.stats.expanded_nodes == 0


def test_trace_events_are_joint_states_without_cost_agent_or_step_fields() -> None:
    buf = io.StringIO()
    grid = grid_from(["..."] * 3)
    tasks = [AgentTask((2, 0), (2, 2)), AgentTask((0, 2), (2, 0))]
    planner = Srrt(config("srrt"))
    planner.plan(grid, tasks, TraceRecorder(buf))

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = {e["event"] for e in events}
    assert {"node_expanded", "path_found", "planning_finished"} <= kinds
    # Joint-state expansions carry the flattened position tuple and NO cost,
    # agent or t — sRRT nodes are joint states with no g-value (no optimality to
    # count toward), no owning agent, and no single timestep. The root (the start
    # tuple) is the first event.
    expansions = [e for e in events if e["event"] == "node_expanded"]
    assert all("cost" not in e and "agent" not in e and "t" not in e for e in expansions)
    assert all(len(e["state"]) == 4 for e in expansions)
    assert expansions[0]["state"] == [2, 0, 0, 2]
    finished = events[-1]
    assert finished["event"] == "planning_finished" and finished["success"] is True
    assert set(finished["metrics"]) == {"expanded_nodes", "makespan", "sum_of_costs"}

