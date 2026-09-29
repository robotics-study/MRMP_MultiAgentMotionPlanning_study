"""MA-RRT*: seeded determinism (the PRNG is part of the algorithm's identity),
valid conflict-free joint paths on every scenario the sampler can solve within
budget, honest sub-optimality where finite budgets cannot converge, honest
failure when the budget dies before the goal tuple becomes a tree vertex, and
the trace shape of tree growth (joint-state node_expanded without agent/t).

The search branch proves 66/33/30 are the optima on these scenarios; MA-RRT*
may only match or exceed them. Matching is convergence (open01_cross's lower
bound is tight, so its first solution is already optimal); exceeding is the
honest price of sampling under a finite iteration budget."""

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
from mrmp.sampling import MaRrtStar


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
        next((REPO_ROOT / "configs").rglob("ma_rrt_star.yaml")).read_text(encoding="utf-8")
    )
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    return ParamSet.from_yaml(
        write_config(
            tmp_path / "ma_rrt_star.yaml", "ma_rrt_star", doc["params"], section="sampling"
        )
    )


def test_contract_matches_config() -> None:
    planner = MaRrtStar(config("ma_rrt_star"))
    assert planner.name == config("ma_rrt_star").algorithm == "ma_rrt_star"
    assert planner.required_capabilities() == {Capability.DISCRETE_SPACE}


def test_single_agent_converges_to_the_manhattan_optimal_on_every_seed(
    tmp_path,
) -> None:
    # One agent on an open grid: every greedy segment is a Manhattan-shortest
    # straight walk, so the first goal-biased sample already yields cost 8 and
    # informed pruning (cost + admissible bound > incumbent) freezes the tree at
    # that optimum — seed-independence IS the convergence property here.
    grid = grid_from(["....."] * 5)
    for seed in (42, 7, 123):
        planner = MaRrtStar(_with_params(tmp_path, seed=seed))
        result = planner.plan(grid, [AgentTask((4, 0), (0, 4))])
        assert result.success
        assert result.cost == pytest.approx(8.0)
        assert result.paths[0][0] == (4, 0) and result.paths[0][-1] == (0, 4)


def test_same_seed_replays_byte_identical_plans() -> None:
    # The MINSTD stream is part of the algorithm's identity: all three language
    # engines must replay identical trees from one seed. Same-seed reruns are the
    # first line of that contract.
    grid, tasks = _scenario("open01_swap")
    a = MaRrtStar(config("ma_rrt_star")).plan(grid, tasks)
    b = MaRrtStar(config("ma_rrt_star")).plan(grid, tasks)
    assert a.success and b.success
    assert a.paths == b.paths
    assert a.cost == pytest.approx(b.cost)
    assert a.stats.expanded_nodes == b.stats.expanded_nodes


def test_open01_cross_hits_the_joint_optimum() -> None:
    # The crossing is timed apart without either agent slowing down: both greedy
    # walks are straight and pass the crossing at different steps, so the first
    # goal-tuple sample already yields 16 + 17 = 33 — exactly what joint-space
    # A* proves optimal. Where the metric's lower bound is tight, sampling lands
    # on the optimum immediately.
    grid, tasks = _scenario("open01_cross")
    result = MaRrtStar(config("ma_rrt_star")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost == pytest.approx(33.0)
    assert max(len(p) - 1 for p in result.paths) == 17


def test_open01_swap_solves_honestly_suboptimally() -> None:
    # Head-on swap: greedy walks collide head-on, so the tree must grow through
    # intermediate joint states before any chain reaches the goal tuple. The
    # search branch proves 30 is optimal; a seeded sampler at its default budget
    # lands above it — the honest price of sampling with a finite iteration
    # budget (asymptotic optimality is a limit statement, not a budget one).
    grid, tasks = _scenario("open01_swap")
    result = MaRrtStar(config("ma_rrt_star")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost > 30.0


def test_maze01_two_finds_a_corridor_pass_by() -> None:
    # The corridor swap the search branch solves at cost 66: MA-RRT* finds A
    # pass-by solution within budget on the default seed (the paper's own honest
    # finding is that sampling scales worse in tight mazes — the cost here sits
    # far above the coupled optimum, and that is the point of the branch).
    grid, tasks = _scenario("maze01_two")
    result = MaRrtStar(config("ma_rrt_star")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths)
    assert result.cost > 66.0


def test_walled_off_goal_fails_honestly_on_budget() -> None:
    # A goal behind a wall is unreachable by any greedy walk: the tree grows and
    # grows but the goal tuple never becomes a vertex, so the budget dies first.
    # That is "no solution found within budget" — NOT a verdict on the instance
    # (MA-RRT* is only probabilistically complete; unlike joint-space A*, an
    # unsolvable instance never gets a closed-set verdict).
    planner = MaRrtStar(config("ma_rrt_star"))
    grid = grid_from(["######", "#.#..#", "######"])
    result = planner.plan(grid, [AgentTask((1, 1), (1, 4))])
    assert not result.success
    assert result.paths == []
    assert result.stats.expanded_nodes > 0


def test_duplicate_start_fails_immediately() -> None:
    # Two agents on one cell at t=0 is no joint state at all — the tree never
    # grows past a root that cannot exist (same honest verdict as the search side).
    planner = MaRrtStar(config("ma_rrt_star"))
    grid = grid_from(["..", ".."])
    tasks = [AgentTask((0, 0), (1, 1)), AgentTask((0, 0), (1, 0))]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.stats.expanded_nodes == 0


def test_trace_events_are_joint_states_without_agent_or_step_fields() -> None:
    buf = io.StringIO()
    grid = grid_from(["..."] * 3)
    tasks = [AgentTask((2, 0), (2, 2)), AgentTask((0, 2), (2, 0))]
    planner = MaRrtStar(config("ma_rrt_star"))
    planner.plan(grid, tasks, TraceRecorder(buf))

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = {e["event"] for e in events}
    assert {"node_expanded", "path_found", "planning_finished"} <= kinds
    # Joint-state expansions carry the flattened position tuple and NO agent/t —
    # a joint state has no single owning agent and no single timestep. The root
    # (the start tuple at cost 0) is the first event.
    expansions = [e for e in events if e["event"] == "node_expanded"]
    assert all("agent" not in e and "t" not in e for e in expansions)
    assert all(len(e["state"]) == 4 for e in expansions)
    assert expansions[0]["state"] == [2, 0, 0, 2] and expansions[0]["cost"] == 0.0
    finished = events[-1]
    assert finished["event"] == "planning_finished" and finished["success"] is True
    assert set(finished["metrics"]) == {"expanded_nodes", "makespan", "sum_of_costs"}
