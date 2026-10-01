"""MAPF-POST: what a plan is worth once it has a clock.

Nothing here re-tests CBS — the discrete plan underneath is the search branch's, already
pinned by test_cbs.py. What these tests pin is the CONVERSION: one event per retained
location (waits dropped), one Type-2 precedence per shared location running from the
earlier visitor's safety marker to the later one's, and a max-relaxation pass over the
resulting STN whose fixed point is the earliest schedule.

The pinned numbers ARE the algorithm: at unit velocity every scenario reproduces its
discrete plan's own costs exactly (this branch's baseline), a faster agent is held at a
crossing until the slower one's marker clears, a slow agent stretches everything behind
it, and one scenario shows the opposite — compression, where a whole step of discrete
waiting turned out to be unnecessary. The failure case is inherited: no plan to
post-process is reported as a failure with zeroed metrics, never as an empty schedule.

Safety is checked on the schedules themselves rather than trusted (the paper's Theorem 2
restated): for every cell two routes share, the visitors' protected clouds are disjoint
in time, and equivalently the grid-graph distance between the robots never falls under
2*delta*v_min/v_max — exactly tight where a Type-2 edge binds.
"""

from __future__ import annotations

import io
import json
from dataclasses import replace
from pathlib import Path

import pytest
from conftest import REPO_ROOT, config, grid_from, write_config

from mrmp.core.capabilities import Capability
from mrmp.core.params import ParamSet
from mrmp.core.trace import TraceRecorder
from mrmp.core.types import AgentTask, Cell, TimedPlanResult
from mrmp.kinodynamic import MapfPost
from mrmp.maps.loader import load_map, load_scenario
from mrmp.maps.occupancy_grid import OccupancyGrid2D


def _scenario(name: str) -> tuple[OccupancyGrid2D, list[AgentTask]]:
    """Scenario cells AND each agent's velocity limit — this branch is the only one that
    reads vmax, so its helper cannot reuse the discrete planners'."""
    scenario = load_scenario(REPO_ROOT / "maps" / "scenarios" / f"{name}.yaml")
    grid = load_map(scenario.map_path)
    assert isinstance(grid, OccupancyGrid2D)  # the only map type; narrow for the type checker
    tasks = [
        AgentTask(
            start=grid.world_to_cell(*spec.start),
            goal=grid.world_to_cell(*spec.goal),
            vmax=spec.vmax,
        )
        for spec in scenario.agents
    ]
    return grid, tasks


def _at(result: TimedPlanResult, agent: int, cell: Cell) -> float:
    """The scheduled arrival time at one retained location."""
    return result.times[agent][result.routes[agent].index(cell)]


def _departure(result: TimedPlanResult, agent: int, cell: Cell, v: float) -> float:
    """When that agent leaves the cell (the next arrival minus its own traversal l/v).
    The Type-2 precedence reads this instant plus delta/v, never the arrival itself."""
    route = result.routes[agent]
    return result.times[agent][route.index(cell) + 1] - 1.0 / v


def _position(route: list[Cell], times: list[float], v: float, tau: float) -> tuple[float, float]:
    """Where an agent is at time tau under the uniform velocity model: dwell on a cell
    until its departure (the NEXT arrival minus this agent's own traversal time l/v), then
    traverse at exactly v. Cell indices are used as coordinates — grid distance and
    position space are the same metric here."""
    if tau <= 0:
        return (float(route[0][0]), float(route[0][1]))
    if tau >= times[-1]:
        return (float(route[-1][0]), float(route[-1][1]))
    for i in range(len(route) - 1):
        arrive = times[i + 1]
        depart = arrive - 1.0 / v
        if tau < depart:
            return (float(route[i][0]), float(route[i][1]))
        if tau < arrive:
            frac = (tau - depart) * v
            a, b = route[i], route[i + 1]
            return (a[0] + (b[0] - a[0]) * frac, a[1] + (b[1] - a[1]) * frac)
    raise AssertionError("unreachable: tau lies inside the schedule")


def _cloud(times: list[float], v: float, delta: float, index: int) -> tuple[float, float]:
    """The open time interval during which that agent lies strictly INSIDE the radius-
    delta cloud of one retained location: from delta/v before its arrival until delta/v
    after it leaves (a final cell never leaves, so its interval is unbounded)."""
    enter = times[index] - delta / v
    if index + 1 >= len(times):
        return (enter, float("inf"))
    depart = times[index + 1] - 1.0 / v
    return (enter, depart + delta / v)


def cloud_gap(result: TimedPlanResult, vmax: list[float], delta: float) -> float:
    """Tightest gap between the two visitors' cloud intervals over every shared cell.
    Negative would mean overlap; exactly 0 means the precedence binds (open intervals may
    touch)."""
    worst = float("inf")
    for i in range(len(result.routes)):
        for j in range(i + 1, len(result.routes)):
            index_i = {cell: k for k, cell in enumerate(result.routes[i])}
            for k, cell in enumerate(result.routes[j]):
                if cell not in index_i:
                    continue
                left = _cloud(result.times[i], vmax[i], delta, index_i[cell])
                right = _cloud(result.times[j], vmax[j], delta, k)
                first, second = (left, right) if left[0] <= right[0] else (right, left)
                worst = min(worst, second[0] - first[1])
    return worst


def min_graph_distance(
    result: TimedPlanResult, vmax: list[float]
) -> tuple[float, float]:
    """Exact minimum grid-graph distance over the whole execution and when it happens.

    A position is piecewise-linear in time with breakpoints ONLY at arrivals (and at the
    departures they imply), so evaluating every breakpoint is exact rather than sampled:
    between consecutive breakpoints each coordinate is affine, so a sum of absolutes cannot
    dip below its endpoint values."""
    horizon = max(times[-1] for times in result.times)
    points: set[float] = {0.0}
    for k, times in enumerate(result.times):
        points.update(float(t) for t in times)
        points.update(times[i] - 1.0 / vmax[k] for i in range(1, len(times)))
    best, at = float("inf"), 0.0
    for tau in sorted(p for p in points if p <= horizon):
        pos = [_position(result.routes[k], result.times[k], vmax[k], tau) for k in range(len(vmax))]
        for i in range(len(pos)):
            for j in range(i + 1, len(pos)):
                d = abs(pos[i][0] - pos[j][0]) + abs(pos[i][1] - pos[j][1])
                if d < best:
                    best, at = d, tau
    return best, at


def _config(tmp_path: Path, delta: float) -> ParamSet:
    """The declared config with `delta` replaced (the inherited CBS budget stays)."""
    return ParamSet.from_yaml(
        write_config(
            tmp_path / "mapf_post.yaml",
            "mapf_post",
            [
                {"name": "delta", "type": "float", "default": delta, "min": 0.0, "max": 0.5,
                 "description": "test delta"},
                {"name": "max_ct_expansions", "type": "int", "default": 256, "min": 1,
                 "description": "test budget"},
            ],
            section="kinodynamic",
        )
    )


def test_contract_matches_config() -> None:
    planner = MapfPost(config("mapf_post"))
    assert planner.name == config("mapf_post").algorithm == "mapf_post"
    # The input stays discrete: cells plus a per-agent velocity limit.
    assert planner.required_capabilities() == {Capability.DISCRETE_SPACE}
    assert config("mapf_post").get_float("delta") == 0.25
    assert config("mapf_post").get_int("max_ct_expansions") == 256


def test_delta_zero_is_refused_rather_than_silently_unsafe(tmp_path: Path) -> None:
    # The declared range includes 0 (a config may declare it), but a collapsed marker voids
    # the safety bound, so the planner itself refuses it instead of scheduling unsafely.
    planner = MapfPost(_config(tmp_path, 0.0))
    grid, tasks = _scenario("pocket01_swap_timed")
    with pytest.raises(ValueError):
        planner.plan(grid, tasks)


def test_single_agent_scales_time_by_its_own_velocity() -> None:
    # One agent, no Type-2 edge possible: the STN is a single chain, so every arrival is
    # exactly the previous one plus l/v. Velocity alone sets the clock — 4 cells at v = 2
    # arrive 0.5 apart, and the cost is TIME (2.0), not steps (4).
    planner = MapfPost(config("mapf_post"))
    grid = grid_from(["....."] * 3)
    result = planner.plan(grid, [AgentTask((1, 0), (1, 4), vmax=2.0)])
    assert result.success
    assert result.routes == [[(1, 0), (1, 1), (1, 2), (1, 3), (1, 4)]]
    assert result.times == [[0.0, 0.5, 1.0, 1.5, 2.0]]
    assert result.cost == pytest.approx(2.0) and result.makespan == pytest.approx(2.0)
    # Four segments x three edges each, every one of them strictly improving exactly once:
    # the relaxation count is a property of the chain's shape, not of the map.
    assert result.stats.expanded_nodes == 12


def test_unit_velocity_reproduces_the_discrete_plan_exactly() -> None:
    # Both agents at one cell per time unit and every shared location visited a whole step
    # apart: each Type-2 constraint then asks for LESS than the chain already provides, so
    # the schedule IS the discrete plan's own steps — and the metrics equal CBS's cost and
    # makespan, this branch's baseline against which everything else is compared.
    planner = MapfPost(config("mapf_post"))
    grid, tasks = _scenario("open01_swap_timed")
    result = planner.plan(grid, tasks)
    assert result.success
    assert result.times[0] == [float(t) for t in range(17)]  # 16 moves at v = 1
    assert result.times[1] == [float(t) for t in range(15)]
    assert result.cost == pytest.approx(30.0)  # CBS's sum_of_costs on this scenario
    assert result.makespan == pytest.approx(16.0)


def test_waits_are_dropped_and_steps_only_order_the_edges() -> None:
    # The discrete plan behind tee01_head_on spends a whole step waiting at (1,2); the
    # route keeps one entry per LOCATION and its arrival times stop counting steps.
    planner = MapfPost(config("mapf_post"))
    grid, tasks = _scenario("tee01_head_on_timed")
    result = planner.plan(grid, tasks)
    assert result.routes[0] == [(1, 1), (1, 2), (1, 3), (1, 4), (1, 5)]
    # ...and dropping the wait is exactly what continuous execution buys back: agent 1
    # leaves (1,3) at t = 2 and its marker clears a delta later, so agent 0 arrives at 2.5
    # instead of step 3 — the pair sums to 10.5 where the discrete plan cost 11.
    assert result.times[0] == [0.0, 1.0, 2.5, 3.5, 4.5]
    assert result.cost == pytest.approx(10.5)
    assert result.makespan == pytest.approx(6.0)


def test_a_wider_marker_gives_the_discrete_timing_back(tmp_path: Path) -> None:
    # The same instance with delta = 0.5 (the largest legal value): the two markers around a
    # shared cell now need 2 * 0.5 = 1 time unit of separation at v = 1 — exactly the whole
    # step the discrete plan had already spent waiting. The schedule relaxes back to those
    # steps, which is what "this constraint no longer binds" means physically.
    grid, tasks = _scenario("tee01_head_on_timed")
    result = MapfPost(_config(tmp_path, 0.5)).plan(grid, tasks)
    assert result.times[0] == [0.0, 1.0, 3.0, 4.0, 5.0]
    assert result.cost == pytest.approx(11.0)


def test_the_fast_agent_is_held_until_the_slow_one_clears() -> None:
    # The crossing pair at unequal velocities: agent 1 (v = 2) would reach the shared cell
    # (10,9) at t = 4.5 on its own chain alone, but agent 0 (v = 1) only clears ITS marker
    # at 8 + 0.25/1 = 8.25, so agent 1 arrives at 8.25 + 0.25/2 = 8.375 and every later
    # arrival inherits that offset. This schedule is not a rescaled copy of the discrete
    # plan — which is the whole point of putting time on it.
    planner = MapfPost(config("mapf_post"))
    grid, tasks = _scenario("open01_cross_timed")
    assert [t.vmax for t in tasks] == [1.0, 2.0]
    result = planner.plan(grid, tasks)
    assert result.success
    # Agent 0 is never held: its own chain (one cell per unit time) stays tight.
    assert result.times[0] == [float(t) for t in range(17)]
    assert _at(result, 1, (9, 9)) == pytest.approx(4.0)
    # Agent 0 leaves the shared cell at T(next) - l/v = 9 - 1 = 8 and its marker clears at
    # 8 + delta/1; agent 1 needs another delta/v of its own on top of that.
    assert _departure(result, 0, (10, 9), 1.0) == pytest.approx(8.0)
    assert _at(result, 1, (10, 9)) == pytest.approx(8.375)
    # Sum 28.375 against the discrete plan's 33: the fast agent finishes in 12.375 of its
    # own time units while the slow one still needs its full 16 — makespan is the slower clock.
    assert result.times[1][-1] == pytest.approx(12.375)
    assert result.cost == pytest.approx(28.375)
    assert result.makespan == pytest.approx(16.0)


def test_a_slow_agent_stretches_everything_behind_it() -> None:
    # The same maze swap at v = 1 and v = 1/4. The slow agent needs four time units per
    # cell, so its marker leaves a shared cell a whole unit after it does — the fast one's
    # arrival there is pushed from step 23 to 89.25 (the slow agent departs at 88 and its
    # marker clears at 88 + 0.25/0.25 = 89). The fast agent still finishes in 99.25 of ITS
    # units; the makespan is the slow agent's 132 = 4 x 33.
    planner = MapfPost(config("mapf_post"))
    grid, tasks = _scenario("maze01_two_timed")
    assert [t.vmax for t in tasks] == [1.0, 0.25]
    result = planner.plan(grid, tasks)
    assert result.success
    assert result.times[1][:6] == [0.0, 4.0, 8.0, 12.0, 16.0, 20.0]
    assert _at(result, 1, (2, 9)) == pytest.approx(88.0)
    assert _at(result, 0, (2, 9)) == pytest.approx(89.25)
    assert result.times[0][-1] == pytest.approx(99.25)
    assert result.cost == pytest.approx(231.25)
    assert result.makespan == pytest.approx(132.0)


def test_scaling_both_velocities_scales_time_and_nothing_else() -> None:
    # Same instance with both velocity limits doubled: every chain bound halves and the
    # fixed point with it (Type-2 edges carry LB 0, so nothing absolute survives), which is
    # a check that no absolute clock leaked into the construction.
    planner = MapfPost(config("mapf_post"))
    grid, tasks = _scenario("pocket01_swap_timed")
    slow = planner.plan(grid, tasks)
    fast = planner.plan(grid, [replace(t, vmax=t.vmax * 2.0) for t in tasks])
    assert fast.routes == slow.routes
    assert [t / 2.0 for t in slow.times[0]] == pytest.approx(list(fast.times[0]))
    assert [t / 2.0 for t in slow.times[1]] == pytest.approx(list(fast.times[1]))
    assert fast.cost == pytest.approx(slow.cost / 2.0)


def test_every_shared_cell_stays_protected() -> None:
    # Theorem 2, checked on the schedules instead of trusted: for every cell two routes
    # share, the two visitors' protected clouds are disjoint in time (a negative gap would
    # mean overlap), and equivalently the grid-graph distance never falls under
    # 2*delta*v_min/v_max. Both are evaluated at every breakpoint of the piecewise-linear
    # motion, so they are exact minima rather than samples.
    planner = MapfPost(config("mapf_post"))
    delta = config("mapf_post").get_float("delta")
    for name in (
        "open01_cross_timed",
        "open01_swap_timed",
        "pocket01_swap_timed",
        "tee01_head_on_timed",
        "maze01_two_timed",
    ):
        grid, tasks = _scenario(name)
        result = planner.plan(grid, tasks)
        assert result.success
        vmax = [t.vmax for t in tasks]
        gap = cloud_gap(result, vmax, delta)
        assert gap >= 0.0, f"{name}: protected clouds overlap by {-gap}"
        bound = 2 * delta * min(vmax) / max(vmax)
        distance, at = min_graph_distance(result, vmax)
        assert distance >= bound - 1e-9, f"{name}: {distance} < {bound} at t={at}"


def test_the_bound_is_tight_exactly_where_a_type_2_edge_binds() -> None:
    # On the unit-velocity tee junction the minimum distance is EXACTLY 2*delta (0.5): the
    # later visitor reaches its marker at the instant the earlier one passes its own, which
    # is what "this precedence binds" means physically. The crossing pair's bound is looser
    # (v_min/v_max = 1/2) and not even reached — safety there comes with margin, and the
    # test says so instead of pretending every constraint binds.
    planner = MapfPost(config("mapf_post"))
    grid, tasks = _scenario("tee01_head_on_timed")
    result = planner.plan(grid, tasks)
    distance, at = min_graph_distance(result, [t.vmax for t in tasks])
    assert distance == pytest.approx(0.5) and at == pytest.approx(2.0)

    grid, tasks = _scenario("open01_cross_timed")
    crossed = planner.plan(grid, tasks)
    distance, _ = min_graph_distance(crossed, [t.vmax for t in tasks])
    assert distance > 2 * 0.25 * (1.0 / 2.0)


def test_no_plan_to_post_process_is_reported_as_a_failure() -> None:
    # The corridor swap has no discrete plan at all and this branch invents none: the
    # inherited budget stop comes through as a failure with every metric zeroed, so a
    # reader can never mistake "nothing to schedule" for an empty schedule.
    planner = MapfPost(config("mapf_post"))
    grid, tasks = _scenario("corridor01_head_on_timed")
    result = planner.plan(grid, tasks)
    assert not result.success
    assert result.routes == [] and result.times == []
    assert result.cost == 0.0 and result.makespan == 0.0
    assert result.stats.expanded_nodes == 0


def test_trace_carries_schedules_and_no_search_events() -> None:
    # The underlying CBS runs SILENTLY: a timed trace carries planning_started (with vmax),
    # one schedule_found per agent in index order, and planning_finished — no
    # node_expanded and no conflict_found, because this branch's own work metric counts STN
    # relaxations, not the base search's expansions.
    buf = io.StringIO()
    grid, tasks = _scenario("pocket01_swap_timed")
    with TraceRecorder(buf) as recorder:
        MapfPost(config("mapf_post")).plan(grid, tasks, recorder)
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    # planning_started is the driver's event (it carries vmax), never the planner's.
    assert {e["event"] for e in events} == {"schedule_found", "planning_finished"}
    founds = [e for e in events if e["event"] == "schedule_found"]
    assert [e["agent"] for e in founds] == [0, 1]
    assert founds[0]["cells"][0] == [1, 1] and founds[0]["times"][0] == 0
    finished = events[-1]
    assert finished["success"] is True
    assert set(finished["metrics"]) == {"expanded_nodes", "makespan", "sum_of_costs"}
    assert finished["metrics"]["sum_of_costs"] == pytest.approx(10.0)
