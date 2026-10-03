"""RHCR: the hybrid folded into time — a Windowed MAPF solver (plain CBS inside)
re-run from the ACTUAL positions every h steps, resolving only the collisions whose
arrival step falls inside (t_now, t_now + w]. The pinned numbers ARE the algorithm:
when the window covers an instance's whole horizon a single episode IS plain CBS —
field for field, expanded_nodes included — and rolling at h = 1 on top of that window
replays the very same trajectory. Narrow the window and the myopia is the point: at
w = 1 every conflict is resolved by a wait that merely defers itself into the next
window, so head-on swaps pin both agents forever (honest budget stop, never a verdict)
and even open01_swap goes suboptimal. The failure mode has two faces and the tests
keep them apart: an ill-formed instance fails at once with zero expansions (a t=0
collision is history, not a conflict any window could resolve), while a solvable
instance starved of budget fails honestly — the same cross that succeeds at 323
expansions stops after its root alone."""

from __future__ import annotations

import io
import json
from pathlib import Path

import pytest
from conftest import REPO_ROOT, config, grid_from, write_config

from mrmp.core.capabilities import Capability
from mrmp.core.params import ParamSet, ParamError
from mrmp.core.trace import TraceRecorder
from mrmp.core.types import AgentTask, Cell
from mrmp.maps.loader import load_map, load_scenario
from mrmp.maps.occupancy_grid import OccupancyGrid2D
from mrmp.search import Cbs, Rhcr


def assert_simultaneous(paths: list[list[Cell]], starts: list[Cell], goals: list[Cell]) -> None:
    """Full-horizon contract of an EXECUTED trajectory (the decentralized branch's
    shape): every path spans the makespan, every step stays or steps one cell, and
    across every pair no vertex conflict and no swap at any step — a windowed episode
    resolves every collision inside its window before any of it is ever executed."""
    horizon = max(len(p) for p in paths) - 1
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


def _params(tmp_path: Path, window: int, replan_period: int, max_steps: int) -> ParamSet:
    """A three-param config decl so tests can pin a window/period/budget without
    editing the shipped configs/search/rhcr.yaml."""
    path = write_config(
        tmp_path / "rhcr.yaml",
        "rhcr",
        [
            {"name": "window", "type": "int", "default": window, "min": 1,
             "description": "test window"},
            {"name": "replan_period", "type": "int", "default": replan_period, "min": 1,
             "description": "test period"},
            {"name": "max_steps", "type": "int", "default": max_steps, "min": 1,
             "description": "test budget"},
        ],
        section="search",
    )
    return ParamSet.from_yaml(path)


def test_contract_matches_config() -> None:
    planner = Rhcr(config("rhcr"))
    assert planner.name == config("rhcr").algorithm == "rhcr"
    assert planner.required_capabilities() == {Capability.DISCRETE_SPACE}
    # The window and its replanning period are the axis; the budget is the honest stop.
    assert config("rhcr").get_int("window") == 2
    assert config("rhcr").get_int("replan_period") == 1
    assert config("rhcr").get_int("max_steps") == 64


def test_replan_period_above_window_raises(tmp_path: Path) -> None:
    # w >= h is the safety condition itself: executing a step no resolved window ever
    # covered is not a harder problem, it is an unsafe one — raise, never clamp. Each
    # parameter alone validates fine (both are ints >= 1); only plan() can see the pair.
    planner = Rhcr(_params(tmp_path, window=2, replan_period=3, max_steps=64))
    grid = grid_from(["..", ".."])
    with pytest.raises(ParamError):
        planner.plan(grid, [AgentTask((0, 0), (1, 1)), AgentTask((1, 0), (0, 1))])


def test_single_agent_is_plain_a_star_rolled() -> None:
    # One agent, nobody to conflict with: every episode's windowed search is the same
    # tie-break-pinned A* wave and h = 1 re-solves from each executed step. The
    # executed trajectory is exactly the straight-up-then-right line the CBS page pins
    # — rolling changes the route's SHAPE not at all, only how often it is recomputed.
    planner = Rhcr(config("rhcr"))
    result = planner.plan(grid_from(["....."] * 5), [AgentTask((4, 0), (0, 4))])
    assert result.success
    assert result.paths == [
        [(4, 0), (3, 0), (2, 0), (1, 0), (0, 0), (0, 1), (0, 2), (0, 3), (0, 4)]
    ]
    assert result.cost == pytest.approx(8.0)
    assert result.stats.expanded_nodes == 84


def test_already_at_goal_is_vacuously_done() -> None:
    # start == goal on a one-agent instance: the loop's first check is already true,
    # no episode ever runs — zero expansions, one-step path, zero cost.
    planner = Rhcr(config("rhcr"))
    result = planner.plan(grid_from(["..", ".."]), [AgentTask((0, 0), (0, 0))])
    assert result.success
    assert result.paths == [[(0, 0)]]
    assert result.cost == pytest.approx(0.0)
    assert result.stats.expanded_nodes == 0


def test_duplicate_assignment_is_not_an_instance() -> None:
    # Same well-formedness rule as the decentralized branch: a t=0 collision is
    # history, not a conflict any window could resolve — fail before episode zero.
    planner = Rhcr(config("rhcr"))
    grid = grid_from(["..", ".."])
    result = planner.plan(grid, [AgentTask((0, 0), (1, 1)), AgentTask((0, 0), (0, 1))])
    assert not result.success
    assert result.paths == []
    assert result.stats.expanded_nodes == 0


def test_window_covering_the_horizon_is_exactly_cbs(tmp_path: Path) -> None:
    # The generalization pinned at its limit. With w = 99 the single episode's window
    # covers every horizon here, so RHCR IS plain CBS — same tie-breaks, so not merely
    # the same optimum but the SAME paths (cut each padded trajectory at its own first
    # arrival) and the SAME honest expansion count. The pinned numbers are CBS's own,
    # exactly as test_cbs pins them: 35 / 574 / 2753.
    one_shot = _params(tmp_path, window=99, replan_period=99, max_steps=64)
    cbs = Cbs(config("cbs"))
    for name, cost, expanded in [
        ("open01_cross", 33.0, 35),
        ("open01_swap", 30.0, 574),
        ("maze01_two", 66.0, 2753),
    ]:
        grid, tasks = _scenario(name)
        single = Rhcr(one_shot).plan(grid, tasks)
        joint = cbs.plan(grid, tasks)
        assert single.success and joint.success
        # Same paths: RHCR pads to the makespan; cut at each agent's own arrival.
        assert [p[: len(q)] for p, q in zip(single.paths, joint.paths, strict=True)] == joint.paths
        assert single.cost == pytest.approx(joint.cost) == pytest.approx(cost)
        assert single.stats.expanded_nodes == joint.stats.expanded_nodes == expanded


def test_rolling_replays_the_single_episode_trajectory(tmp_path: Path) -> None:
    # Re-solving from the actual positions every single step replays EXACTLY the
    # trajectory the one-shot episode produced (the sum-of-costs objective is what
    # makes rolling free — each re-solve lands on the same optimal continuation).
    # What rolling is never free of is the price: 3112 low-level pops instead of 574.
    grid, tasks = _scenario("open01_swap")
    one_shot = Rhcr(_params(tmp_path, window=99, replan_period=99, max_steps=64)).plan(grid, tasks)
    rolled = Rhcr(_params(tmp_path, window=99, replan_period=1, max_steps=64)).plan(grid, tasks)
    assert rolled.paths == one_shot.paths
    assert rolled.cost == pytest.approx(one_shot.cost) == pytest.approx(30.0)
    assert rolled.stats.expanded_nodes == 3112
    assert one_shot.stats.expanded_nodes == 574


def test_open01_cross_the_window_is_inert_here(tmp_path: Path) -> None:
    # The crossing the whole branch pins at [16, 17] = 33: timing already separates the
    # pair at (10, 9), so no window ever sees a conflict and every width from 1 up pins
    # the SAME run down to the expansion count — on this map the window is inert by
    # geometry, not by size.
    grid, tasks = _scenario("open01_cross")
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    for w in (1, 2):
        result = Rhcr(_params(tmp_path, window=w, replan_period=1, max_steps=64)).plan(grid, tasks)
        assert result.success
        assert_simultaneous(result.paths, starts, goals)
        assert _per_agent(result.paths) == [16, 17]
        assert max(len(p) for p in result.paths) - 1 == 17
        assert result.cost == pytest.approx(33.0)
        assert result.stats.expanded_nodes == 323


def test_open01_swap_the_window_is_the_axis(tmp_path: Path) -> None:
    # The head-on swap on row 10. At the default w = 2 the conflict enters a window
    # while both agents can still route around each other: agent 0's straight line is
    # never constrained and agent 1's detour costs exactly the joint optimum (30,
    # makespan 16 — CBS's number). At w = 1 every conflict arrives one step late: the
    # only resolution left is a wait that defers itself into the next window, and the
    # alternating waits cost a step nobody can recover — 31 with makespan 17.
    grid, tasks = _scenario("open01_swap")
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]

    wide = Rhcr(config("rhcr")).plan(grid, tasks)
    assert wide.success
    assert_simultaneous(wide.paths, starts, goals)
    assert _per_agent(wide.paths) == [16, 14]
    assert wide.cost == pytest.approx(30.0)
    assert max(len(p) for p in wide.paths) - 1 == 16
    assert wide.stats.expanded_nodes == 576

    narrow = Rhcr(_params(tmp_path, window=1, replan_period=1, max_steps=64)).plan(grid, tasks)
    assert narrow.success
    assert_simultaneous(narrow.paths, starts, goals)
    assert _per_agent(narrow.paths) == [16, 14]  # the detour still happens...
    assert narrow.cost == pytest.approx(31.0)  # ...but the deferral chain costs one more
    assert max(len(p) for p in narrow.paths) - 1 == 17
    assert narrow.stats.expanded_nodes == 376


def test_pocket_and_tee_the_window_is_the_edge_of_solvable(tmp_path: Path) -> None:
    # The head-on swaps with a pocket / junction: solvable only if someone sees the
    # conflict EARLY enough for the constrained-optimal re-plan to route around instead
    # of waiting. At w = 1 the wait is always locally cheaper and the alternating
    # constraints pin both agents until the budget expires — an honest stop, never a
    # verdict (the same instances solve at the default window). Both maps pin the exact
    # same stall cost: pinned adjacent, every episode spends the same handful of pops.
    for name, cost, expanded in [("pocket01_swap", 10.0, 165), ("tee01_head_on", 11.0, 220)]:
        grid, tasks = _scenario(name)
        starts = [t.start for t in tasks]
        goals = [t.goal for t in tasks]
        result = Rhcr(config("rhcr")).plan(grid, tasks)
        assert result.success
        assert_simultaneous(result.paths, starts, goals)
        assert _per_agent(result.paths) == [4, 6]
        assert result.cost == pytest.approx(cost)
        assert max(len(p) for p in result.paths) - 1 == 6
        assert result.stats.expanded_nodes == expanded

        stalled = Rhcr(_params(tmp_path, window=1, replan_period=1, max_steps=64)).plan(grid, tasks)
        assert not stalled.success
        assert stalled.paths == []
        assert stalled.stats.expanded_nodes == 1895


def test_maze01_two_the_corridor_meeting_needs_no_foresight(tmp_path: Path) -> None:
    # The two-room maze pins 66/33 at every window (the meeting point sits where a wait
    # alone resolves the head-on — foresight buys nothing here either), and the rolling
    # re-solve pays for its foresight in expansions: w = 1 spends 13860 pops because
    # every late conflict is still resolved by a one-step-early constraint.
    grid, tasks = _scenario("maze01_two")
    starts = [t.start for t in tasks]
    goals = [t.goal for t in tasks]
    result = Rhcr(config("rhcr")).plan(grid, tasks)
    assert result.success
    assert_simultaneous(result.paths, starts, goals)
    assert _per_agent(result.paths) == [33, 33]
    assert result.cost == pytest.approx(66.0)
    assert max(len(p) for p in result.paths) - 1 == 33
    assert result.stats.expanded_nodes == 13958


def test_corridor_head_on_stalls_at_every_window(tmp_path: Path) -> None:
    # Width-1 corridor, swapping ends: no joint plan exists at ANY window (the same
    # tree-graph argument the Push and Swap page pins). Every episode resolves its
    # window by pinning both agents in place; the loop stops honestly at the budget.
    # Pinned at a small budget because every step re-runs a whole windowed CBS — the
    # count is the honest price of 8 episodes, not a verdict (the same instance at the
    # shipped budget just keeps paying until step 64).
    grid, tasks = _scenario("corridor01_head_on")
    result = Rhcr(_params(tmp_path, window=2, replan_period=1, max_steps=8)).plan(grid, tasks)
    assert not result.success
    assert result.paths == []
    assert result.stats.expanded_nodes == 976


def test_budget_exhaustion_is_not_a_verdict(tmp_path: Path) -> None:
    # The SAME solvable cross that succeeds at the shipped config stops after episode
    # zero's root expansions when max_steps = 1 (t_now starts at 0, so the budget is
    # already spent before a single step commits). A budget stop says "no execution
    # within budget", never "unsolvable" — the windowed instance was always satisfiable.
    grid, tasks = _scenario("open01_cross")
    result = Rhcr(_params(tmp_path, window=2, replan_period=1, max_steps=1)).plan(grid, tasks)
    assert not result.success
    assert result.paths == []
    # Exactly the root's two unconstrained A* expansions — 35, the same number the
    # single-episode CBS-equivalent run spends on this scenario in total.
    assert result.stats.expanded_nodes == 35


def test_trace_shows_the_rolling_tree_lifecycle() -> None:
    # A crossing pair on a 3x3 grid. The window covers the whole horizon here, so the
    # FIRST episode already resolves the crossing — and because rolling replans from the
    # executed positions with NO constraints carried forward, the same collision is
    # rediscovered and re-resolved in the next episode at the SAME absolute step: two
    # conflict_found events for cell (2, 2) at t = 2, each branched on both agents.
    # Times are ABSOLUTE steps throughout; path_found appears only once execution ends.
    buf = io.StringIO()
    grid = grid_from(["..."] * 3)
    tasks = [AgentTask((2, 0), (2, 2)), AgentTask((0, 2), (2, 0))]
    planner = Rhcr(config("rhcr"))
    rec = TraceRecorder(buf)
    rec.planning_started("rhcr", "mem", {"window": 2, "replan_period": 1, "max_steps": 64})
    result = planner.plan(grid, tasks, rec)
    rec.close()

    assert result.success and result.cost == pytest.approx(6.0)
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = [e["event"] for e in events]
    expansions = [e for e in events if e["event"] == "node_expanded"]
    # Per-agent space-time expansion: every event names its agent and ABSOLUTE step.
    assert all("agent" in e and "t" in e and len(e["state"]) == 2 for e in expansions)
    conflicts = [e for e in events if e["event"] == "conflict_found"]
    # The same collision, rediscovered once per episode until its step is behind them.
    assert [(e["kind"], tuple(e["cell"]), e["t"], tuple(e["agents"])) for e in conflicts] == [
        ("vertex", (2, 2), 2, (0, 1)),
        ("vertex", (2, 2), 2, (0, 1)),
    ]
    constraints = [(e["agent"], e["kind"], tuple(e["cell"]), e["t"])
                   for e in events if e["event"] == "constraint_added"]
    assert constraints == [
        (0, "vertex", (2, 2), 2), (1, "vertex", (2, 2), 2),
        (0, "vertex", (2, 2), 2), (1, "vertex", (2, 2), 2),
    ]
    paths = [e for e in events if e["event"] == "path_found"]
    assert [e["agent"] for e in paths] == [0, 1]
    # path_found comes after every conflict/constraint event — the executed trajectory
    # is emitted once, when execution stops on the simultaneous-co-presence step.
    last_tree = max(i for i, k in enumerate(kinds) if k in ("conflict_found", "constraint_added"))
    assert all(i > last_tree for i, e in enumerate(events) if e["event"] == "path_found")
    finished = events[-1]
    assert finished["event"] == "planning_finished" and finished["success"] is True
    assert set(finished["metrics"]) == {"expanded_nodes", "makespan", "sum_of_costs"}
