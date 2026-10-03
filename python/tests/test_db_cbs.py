"""db-CBS: CBS whose low level has momentum. The pins below ARE the lattice — at
delta < 1 (floor 0) a move is exactly x' = x + v' with |v' - v| <= 1 per axis, so a
vmax=1 agent moves like the search branch's 4-connected... no: like its 8-connected
cousin (every axis independently steps), while a vmax=2 agent visibly ramps (the
first step off rest is ONE cell, never two). At delta >= 1 physics negotiates: the
landing may miss the ideal landing by one Manhattan unit and velocity jumps by up to
one — the pinned single-agent path moves two cells on its very first step.

The branch's conflict semantics differ from the search branch on purpose: only
co-presence at a sampled step is a conflict (kind "vertex" always; there is no edge
kind here because point robots sampled at integer steps pass through each other —
the corridor swap that CBS honestly fails IS SOLVABLE here and pinned so below).
The ladder [1.5, 0.5] runs two independent fresh trees and keeps the last rung that
solved; expanded_nodes accumulates across rungs, which is why the default-config
pins are larger than single-rung pins on the same scenario."""

from __future__ import annotations

import io
import json
import tempfile
from pathlib import Path

import pytest
from conftest import REPO_ROOT, config, grid_from, write_config

from mrmp.core.capabilities import Capability
from mrmp.core.params import ParamError, ParamSet
from mrmp.core.trace import TraceRecorder
from mrmp.core.types import AgentTask, Cell
from mrmp.kinodynamic import DbCbs
from mrmp.maps.loader import load_map, load_scenario
from mrmp.maps.occupancy_grid import OccupancyGrid2D


def _occ(path: list[Cell], t: int) -> Cell:
    """Stay-at-goal occupancy of a finished path (same rule as the planner)."""
    return path[t] if t < len(path) else path[-1]


def assert_joint_valid(paths: list[list[Cell]], vmaxes: list[int]) -> None:
    """Every step's displacement is within that agent's integer velocity bound and
    changed by at most 1 per axis since the previous step (the EXACT double
    integrator — these pins are all read out of delta < 1 rungs), and no two agents
    ever co-occupy a cell at the same step. Swaps across a step are NOT checked:
    this branch has no edge conflicts by design."""
    horizon = max(len(p) - 1 for p in paths)
    for k, path in enumerate(paths):
        v_prev: Cell | None = None
        for t in range(1, len(path)):
            v: Cell = (path[t][0] - path[t - 1][0], path[t][1] - path[t - 1][1])
            assert max(abs(v[0]), abs(v[1])) <= vmaxes[k], f"agent {k} step {t}: |v| > vmax"
            if v_prev is not None:
                dv = (abs(v[0] - v_prev[0]), abs(v[1] - v_prev[1]))
                assert max(dv) <= 1, f"agent {k} step {t}: acceleration exceeds 1"
            v_prev = v
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            for t in range(horizon + 1):
                assert _occ(paths[i], t) != _occ(paths[j], t), f"co-presence at step {t}"


def _swap_at(paths: list[list[Cell]]) -> bool:
    """True iff some pair swaps cells across a step — legal here, impossible for
    the search branch. The corridor pin below asserts this is what makes it solvable."""
    horizon = max(len(p) - 1 for p in paths)
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            for t in range(1, horizon + 1):
                if (_occ(paths[i], t) == _occ(paths[j], t - 1)
                        and _occ(paths[j], t) == _occ(paths[i], t - 1)):
                    return True
    return False


def _scenario(name: str) -> tuple[OccupancyGrid2D, list[AgentTask]]:
    scenario = load_scenario(REPO_ROOT / "maps" / "scenarios" / f"{name}.yaml")
    grid = load_map(scenario.map_path)
    assert isinstance(grid, OccupancyGrid2D)  # the only map type; narrow for the type checker
    tasks = [
        AgentTask(grid.world_to_cell(*spec.start), grid.world_to_cell(*spec.goal), vmax=spec.vmax)
        for spec in scenario.agents
    ]
    return grid, tasks


def _params(tmp_path: Path, delta_start: float, delta_end: float, max_ct: int = 256) -> ParamSet:
    """A three-param config decl so tests can pin a rung/budget without editing the
    shipped configs/kinodynamic/db_cbs.yaml."""
    path = write_config(
        tmp_path / "db_cbs.yaml",
        "db_cbs",
        [
            {"name": "delta_start", "type": "float", "default": delta_start, "min": 0.5,
             "max": 1.5, "description": "test ladder start"},
            {"name": "delta_end", "type": "float", "default": delta_end, "min": 0.5,
             "max": 1.5, "description": "test ladder end"},
            {"name": "max_ct_expansions", "type": "int", "default": max_ct, "min": 1,
             "description": "test budget"},
        ],
        section="kinodynamic",
    )
    return ParamSet.from_yaml(path)


def test_contract_matches_config() -> None:
    planner = DbCbs(config("db_cbs"))
    assert planner.name == config("db_cbs").algorithm == "db_cbs"
    assert planner.required_capabilities() == {Capability.DISCRETE_SPACE}
    # The ladder is the axis (loose -> exact) and the budget is the honest stop.
    assert config("db_cbs").get_float("delta_start") == 1.5
    assert config("db_cbs").get_float("delta_end") == 0.5
    assert config("db_cbs").get_int("max_ct_expansions") == 256


def test_ladder_only_tightens_raises(tmp_path: Path) -> None:
    # A widening bound is not what the paper's loop does. Each parameter alone
    # validates fine (both are floats in [0.5, 1.5]); only plan() can see the pair.
    planner = DbCbs(_params(tmp_path, delta_start=0.5, delta_end=1.5))
    grid = grid_from(["..", ".."])
    with pytest.raises(ParamError):
        planner.plan(grid, [AgentTask((0, 0), (1, 1), vmax=1.0)])


def test_non_integer_vmax_raises() -> None:
    # The velocity lattice quantizes integers only — a fractional limit is not a
    # finer model here, it is a different model the lattice cannot express.
    planner = DbCbs(config("db_cbs"))
    grid = grid_from(["..", ".."])
    with pytest.raises(ParamError):
        planner.plan(grid, [AgentTask((0, 0), (1, 1), vmax=0.25)])


def _params_of(delta_start: float, delta_end: float, max_ct: int = 256) -> ParamSet:
    """The shipped config's shape with the ladder overridden — for tests that pin a
    single rung without claiming a tmp_path fixture argument."""
    return _params(Path(tempfile.mkdtemp()), delta_start, delta_end, max_ct)


def test_single_agent_exact_rung_is_the_double_integrator() -> None:
    # One agent, no conflicts: the root's state-space A* IS the answer. At delta < 1
    # a move is exactly x' = x + v' with |v' - v| <= 1 per axis — at vmax=1 every
    # 8-neighbor move plus wait is legal (acceleration vacuous), and the pinned path
    # is the pure diagonal. expanded_nodes counts every state-space pop (no CT
    # branching here — the root's sub-search IS the whole run).
    planner = DbCbs(_params_of(0.5, 0.5))
    result = planner.plan(grid_from(["....."] * 5), [AgentTask((4, 0), (0, 4), vmax=1.0)])
    assert result.success
    assert result.paths[0] == [(4, 0), (3, 1), (2, 2), (1, 3), (0, 4)]
    assert result.cost == pytest.approx(4.0)
    assert result.stats.expanded_nodes == 5


def test_single_agent_vmax_2_ramps_from_rest() -> None:
    # vmax=2 on the same straight shot: acceleration is still 1 per axis, so the
    # first step off rest covers ONE cell and only later steps may cover two —
    # velocity ramps, it never jumps. Cruise speed 2 still wins the trip (cost 3).
    planner = DbCbs(_params_of(0.5, 0.5))
    result = planner.plan(grid_from(["....."] * 5), [AgentTask((4, 0), (0, 4), vmax=2.0)])
    assert result.success
    # First step one cell per axis (ramp), then the velocity is free to cover two.
    assert result.paths[0] == [(4, 0), (3, 1), (1, 2), (0, 4)]
    assert result.cost == pytest.approx(3.0)
    assert result.stats.expanded_nodes == 10


def test_loose_rung_makes_physics_negotiable() -> None:
    # Same instance at delta >= 1: the chaining gap covers one extra cell per axis,
    # so the very first step already covers two cells and the trip costs one step
    # less. The ladder is not a refinement of the exact model — it is a DIFFERENT
    # (negotiable) physics, which is why rungs are independent attempts.
    planner = DbCbs(_params_of(1.5, 1.5))
    result = planner.plan(grid_from(["....."] * 5), [AgentTask((4, 0), (0, 4), vmax=1.0)])
    assert result.success
    assert result.paths[0] == [(4, 0), (2, 0), (1, 2), (0, 4)]
    assert result.cost == pytest.approx(3.0)
    assert result.stats.expanded_nodes == 94


def test_open01_cross_momentum_times_the_crossing() -> None:
    # The textbook cross on the timed scenario: at vmax=1/vmax=2 with exact physics
    # (the ladder's last rung) the two individually-optimal state-paths already miss
    # each other in time-space — NO conflict fires, and agent 1's vmax=2 ramp is
    # visible in its pinned path (one cell out of rest, then two). The constraint
    # tree never branches here; this scenario pins the motion model itself.
    grid, tasks = _scenario("open01_cross_timed")
    result = DbCbs(config("db_cbs")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths, [1, 2])
    assert result.paths[0] == [(10, 1), (9, 2), (8, 3), (8, 4), (9, 5), (9, 6), (9, 7),
                               (9, 8), (8, 9), (8, 10), (8, 11), (9, 12), (10, 13),
                               (10, 14), (10, 15), (10, 16), (10, 17)]
    assert result.paths[1] == [(1, 9), (2, 9), (4, 9), (6, 9), (8, 9), (10, 8),
                               (12, 7), (14, 7), (16, 8), (18, 9)]
    assert result.cost == pytest.approx(25.0)
    assert max(len(p) - 1 for p in result.paths) == 16
    # Both rungs' expansions accumulate; the exact rung's answer stands alone too.
    assert result.stats.expanded_nodes == 604


def test_open01_swap_passes_through_instead_of_detouring() -> None:
    # The head-on swap the search branch must detour (CBS pays 30 for it): here a
    # swap across a step is NOT a conflict — sampled point robots pass through each
    # other — so both agents keep their individually-optimal paths and the crossing
    # needs no wait at all. Cost 28 = 14 + 14, below CBS's 30: the pair slides past
    # on diagonally-offset lanes without ever sharing a cell at one step.
    grid, tasks = _scenario("open01_swap_timed")
    result = DbCbs(config("db_cbs")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths, [1, 1])
    assert result.cost == pytest.approx(28.0)
    assert max(len(p) - 1 for p in result.paths) == 14


def test_pocket_swap_pins_both_paths() -> None:
    # The pocket scenario under velocity: agent 1 ducks through the pocket cells —
    # pinned field-for-field so every language replays the same choice.
    grid, tasks = _scenario("pocket01_swap_timed")
    result = DbCbs(config("db_cbs")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths, [1, 1])
    assert result.paths[0] == [(1, 1), (1, 2), (1, 3), (1, 4), (1, 5)]
    assert result.paths[1] == [(1, 5), (0, 4), (0, 3), (1, 2), (1, 1)]
    assert result.cost == pytest.approx(8.0)
    assert max(len(p) - 1 for p in result.paths) == 4


def test_corridor_swap_is_solvable_here() -> None:
    # The width-1 corridor the search branch honestly FAILS (a swap is unavoidable
    # there and CBS forbids edge conflicts): with vertex-only conflicts the pair
    # simply passes through — one waits, they cross without ever sharing a step.
    grid, tasks = _scenario("corridor01_head_on_timed")
    result = DbCbs(config("db_cbs")).plan(grid, tasks)
    assert result.success
    assert_joint_valid(result.paths, [1, 1])
    assert _swap_at(result.paths)
    assert result.cost == pytest.approx(9.0)
    assert max(len(p) - 1 for p in result.paths) == 5


def test_ladder_rungs_are_independent_fresh_trees(tmp_path: Path) -> None:
    # The ladder runs one fresh CT per rung and keeps the LAST rung that solved —
    # on the cross scenario both rungs solve to the SAME paths (the exact rung's
    # answer is what the default config reports), while expanded_nodes honestly
    # accumulates across both trees.
    grid, tasks = _scenario("open01_cross_timed")
    ladder = DbCbs(config("db_cbs")).plan(grid, tasks)
    exact_only = DbCbs(_params(tmp_path, 0.5, 0.5)).plan(grid, tasks)
    assert ladder.paths == exact_only.paths
    assert ladder.stats.expanded_nodes > exact_only.stats.expanded_nodes


def test_duplicate_start_fails_by_branching_to_dead_ends() -> None:
    # Two agents on one cell at t=0 is unsolvable — and the tree really does die:
    # both children constrain an agent off its own start cell (instantly dead
    # sub-searches) and the queue empties. An honest verdict, not a budget stop;
    # expanded 8 = the two root sub-searches across both rungs, nothing more.
    planner = DbCbs(config("db_cbs"))
    grid = grid_from(["..", ".."])
    tasks = [AgentTask((0, 0), (1, 1), vmax=1.0), AgentTask((0, 0), (0, 1), vmax=1.0)]
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.paths == []
    assert result.stats.expanded_nodes == 8


def test_budget_exhaustion_is_not_a_verdict(tmp_path: Path) -> None:
    # The SAME solvable swap with a budget of 1 fails after the root alone (the CT
    # loop never pops): "no solution found within budget", pinned exactly because
    # every language replays identically.
    grid, tasks = _scenario("open01_swap_timed")
    result = DbCbs(_params(tmp_path, 1.5, 0.5, max_ct=1)).plan(grid, tasks)
    assert not result.success
    assert result.stats.expanded_nodes == 498


def test_trace_shows_the_constraint_tree_lifecycle() -> None:
    # A swapping pair forces branching on the exact rung: the trace must show
    # per-agent space-time expansions (state = cell pair, cost = t), conflict_found
    # with kind VERTEX only (this branch has no edge kind at any delta), a
    # constraint_added for BOTH agents of the conflicting pair, path_found only for
    # the final solution paths, and planning_finished carrying the answered delta.
    buf = io.StringIO()
    grid, tasks = _scenario("pocket01_swap_timed")
    DbCbs(config("db_cbs")).plan(grid, tasks, TraceRecorder(buf))

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = [e["event"] for e in events]
    assert "node_expanded" in kinds and "path_found" in kinds
    expansions = [e for e in events if e["event"] == "node_expanded"]
    assert all("agent" in e and "t" in e and len(e["state"]) == 2 for e in expansions)
    conflicts = [e for e in events if e["event"] == "conflict_found"]
    assert conflicts, "a swapping pair must conflict on the exact rung"
    assert all(e["kind"] == "vertex" and "to" not in e for e in conflicts)
    constraints = [e for e in events if e["event"] == "constraint_added"]
    assert {e["agent"] for e in constraints} == set(conflicts[0]["agents"])
    paths = [e for e in events if e["event"] == "path_found"]
    assert [e["agent"] for e in paths] == [0, 1]
    last_tree_event = max(
        i for i, k in enumerate(kinds) if k in ("conflict_found", "constraint_added")
    )
    assert all(i > last_tree_event for i, e in enumerate(events) if e["event"] == "path_found")
    finished = events[-1]
    assert finished["event"] == "planning_finished" and finished["success"] is True
    assert set(finished["metrics"]) == {"delta", "expanded_nodes", "makespan", "sum_of_costs"}
    # The default ladder answers on its exact rung.
    assert finished["metrics"]["delta"] == 0.5
