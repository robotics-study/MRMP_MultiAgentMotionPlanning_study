"""MAPF-POST — plan post-processing, the kinodynamic branch's (only) algorithm.

Hönig, Kumar, Cohen, Ma, Xu, Ayanian & Koenig (ICAPS 2016), "Multi-Agent Path
Finding with Kinematic Constraints": MAPF solvers plan on a graph in discrete
steps; real robots move continuously at finite velocity. MAPF-POST plans NOTHING
new — it takes the search branch's collision-free discrete plan (CBS runs here,
silently, as the underlying solver) and converts it into a plan-execution
schedule: per-agent routes with wait actions removed plus one earliest arrival
time per retained location, so each agent dwells on a cell until its departure
and traverses at exactly its own velocity limit.

The conversion (the paper's Algorithm 1, implemented literally):

* TPG. A vertex is an event — "agent j enters location s at discrete step t"
  (waits removed: the first occurrence of each retained location keeps its step).
  Type-1 edges chain each agent's own events in route order. Type-2 edges encode
  the plan's collision-freeness as precedence: for every location two agents both
  enter, an edge runs from the EARLIER visitor's event to the LATER one (the scan
  stops at the first later visit — later ones are implied by transitivity). Both
  directions of a same-edge swap can never appear in a collision-free plan (t > s
  and t <= s+1 force t = s+1, exactly an edge conflict), so the earlier visitor
  always has an outgoing move edge and the later one an incoming one: the
  degenerate case where an edge would have no chain to attach to cannot occur.
* Augmentation. Every Type-1 edge e (length l(e) = 1 here, always — retained
  cells are adjacent) splits into v -> m1 (length delta), m1 -> m2 (l - 2*delta),
  m2 -> v' (delta); each Type-2 edge is rewired from the earlier visitor's safety
  marker m1 (delta past its cell, on the outgoing edge) to the later visitor's
  marker m2 (delta before its cell, on the incoming edge). Delta must keep every
  edge longer than 2*delta: validated 0 < delta <= 0.5 = l/2; with OPEN protected
  clouds of radius delta around cells, 2*delta <= l keeps every pair disjoint.
* STN. Every augmented edge becomes a simple temporal constraint [LB, inf] with
  LB = its length / v_k (that agent's own velocity limit) and Type-2 edges get
  [0, inf]; each agent's first event is pinned to t = 0 ([0, 0] from X_S). The
  graph stays acyclic (Type-2 edges always point forward in discrete step), so a
  negative-cost cycle is impossible — the STN is always consistent (paper Theorem
  1) and the EARLIEST schedule exists. This repo computes it as max-relaxation
  Bellman-Ford on the constraint edges directly, which yields exactly the paper's
  t(v) = -dist(v, X_S) over the distance graph; expanded_nodes counts every
  relaxation that strictly improved a label (the algorithm's own work metric).

Execution semantics (the uniform velocity model): an agent dwells at cell c_i
until its departure D_i = T(next) - l(e)/v_k and then traverses at exactly v_k,
arriving exactly on the scheduled time. Safety (paper Theorem 2): point agents
executing a consistent schedule keep graph distance >= 2*delta*v_min/v_max > 0 —
the Type-2 edge makes the later visitor reach its delta-marker only after the
earlier one passed ITS delta-marker leaving that cell, and both markers sit at
distance delta from the shared cell on distinct edges (a same-edge swap is not a
collision-free plan, see above).

The paper's LP variants (minimize flow time / maximize v_min) are NOT implemented
— this is the earliest schedule of Algorithm 1; sum_of_costs here is the resulting
flow-time analogue (goal arrival times summed), not an optimized objective. The
non-holonomic extension (orientation vertices, rotate actions) is out of scope:
this repo's robots stay point agents on a grid.

Determinism across languages: vertex numbering and edge-list order are fixed by
construction order (agents in index order, segments ascending; Type-2 scans agent
index then step then other-agent index), and delta plus every demo vmax value are
dyadic rationals so all times are exactly representable doubles — the strict
improvement counts and arrival times are bit-identical across languages.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core.capabilities import Capability, DiscreteSpace
from ..core.planner import KinodynamicPlanner
from ..core.trace import TraceRecorder
from ..core.types import AgentTask, Cell, PlanStats, TimedPlanResult
from ..search.cbs import Cbs


@dataclass(frozen=True)
class _Edge:
    """One STN constraint: the tail's time plus LB never exceeds the head's."""

    src: int
    dst: int
    lb: float


class MapfPost(KinodynamicPlanner):
    @property
    def name(self) -> str:
        return "mapf_post"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE}

    def plan(
        self,
        space: DiscreteSpace,
        tasks: list[AgentTask],
        recorder: TraceRecorder | None = None,
    ) -> TimedPlanResult:
        delta = float(self.params.get_float("delta"))
        # The config range already clamps into [0.0, 0.5]; the paper's condition is
        # strict positivity — delta = 0 would collapse every safety marker onto its
        # location vertex and void Theorem 2's positive bound.
        if not delta > 0.0:
            raise ValueError(f"mapf_post: delta must be > 0 (got {delta})")

        # The underlying discrete plan: CBS runs SILENTLY (recorder=None) — this
        # branch's trace carries only the schedule it produces, and its own metric
        # counts STN relaxations, not the base search's expansions. A budget hit or
        # an exhausted tree is inherited honestly as "no plan to post-process".
        base = Cbs(self.params).plan(space, tasks, None)
        if not base.success:
            if recorder is not None:
                recorder.planning_finished(
                    False, {"expanded_nodes": 0.0, "makespan": 0.0, "sum_of_costs": 0.0}
                )
            return TimedPlanResult(success=False)

        # Route extraction: drop waits (keep the first of every run of identical
        # cells); retained events keep their discrete step indices — those steps are
        # only ever used to ORDER Type-2 edges, never as times.
        routes: list[list[Cell]] = []
        steps: list[list[int]] = []
        for path in base.paths:
            route: list[Cell] = []
            kept: list[int] = []
            for t, cell in enumerate(path):
                if not route or cell != route[-1]:
                    route.append(cell)
                    kept.append(t)
            routes.append(route)
            steps.append(kept)

        labels, edges, event_ids = self._build_stn(base.paths, routes, steps, tasks, delta)
        expanded = _relax(labels, edges)
        times_by_agent = [[labels[vid] for vid in ids] for ids in event_ids]
        makespan = max(times[-1] for times in times_by_agent)
        cost = sum(times[-1] for times in times_by_agent)

        if recorder is not None:
            for k in range(len(routes)):
                recorder.schedule_found(k, routes[k], times_by_agent[k])
            recorder.planning_finished(
                True,
                {
                    "expanded_nodes": float(expanded),
                    "makespan": makespan,
                    "sum_of_costs": cost,
                },
            )
        return TimedPlanResult(
            True, routes, times_by_agent, cost, makespan, PlanStats(expanded_nodes=expanded)
        )

    @staticmethod
    def _build_stn(
        paths: list[list[Cell]],
        routes: list[list[Cell]],
        steps: list[list[int]],
        tasks: list[AgentTask],
        delta: float,
    ) -> tuple[list[float], list[_Edge], list[list[int]]]:
        """The augmented TPG as a flat edge list plus the initial labels.

        Vertex ids ascend agent by agent (each agent's events in step order), then
        markers take the remaining ids in chain order — identical enumeration in
        Python, C++ and TS. A parked agent (single retained cell) contributes just
        its pinned source vertex: nobody can visit that cell later without a
        discrete conflict, so it has no outgoing edge to attach anything to."""
        n_agents = len(routes)
        # Event vertices first: ids assigned per agent in step order.
        event_ids: list[list[int]] = []
        next_id = 0
        for route in routes:
            ids = list(range(next_id, next_id + len(route)))
            event_ids.append(ids)
            next_id += len(route)
        # Chain sub-edges and their markers, agent by agent, segment by segment.
        edges: list[_Edge] = []
        m1_of: list[list[int]] = [[] for _ in range(n_agents)]
        m2_of: list[list[int]] = [[] for _ in range(n_agents)]
        for j in range(n_agents):
            v_j = float(tasks[j].vmax)
            ids = event_ids[j]
            for i in range(len(ids) - 1):
                m1, m2 = next_id, next_id + 1
                next_id += 2
                m1_of[j].append(m1)
                m2_of[j].append(m2)
                edges.append(_Edge(ids[i], m1, delta / v_j))
                edges.append(_Edge(m1, m2, (1.0 - 2.0 * delta) / v_j))
                edges.append(_Edge(m2, ids[i + 1], delta / v_j))
        # Type-2 precedence: for every retained event of j (except a final parked
        # one — see above), scan each other agent's RAW path beyond that step; the
        # first later visit wins (later ones are implied by transitivity through
        # it). The found occurrence is a retained arrival by construction: the step
        # before it held a different cell, or j would have collided with k there.
        for j in range(n_agents):
            for i in range(len(event_ids[j]) - 1):
                cell = routes[j][i]
                step = steps[j][i]
                for k in range(n_agents):
                    if k == j:
                        continue
                    found = -1
                    for t in range(step + 1, len(paths[k])):
                        if paths[k][t] == cell:
                            found = t
                            break
                    if found < 0:
                        continue
                    idx_k = steps[k].index(found)
                    # The later visit is at step >= 1 so it arrived from another
                    # cell — its incoming segment (and its m2 marker) exists.
                    edges.append(_Edge(m1_of[j][i], m2_of[k][idx_k - 1], 0.0))
        labels = [-math.inf] * next_id
        for j in range(n_agents):
            labels[event_ids[j][0]] = 0.0  # X_S -> every source event, bound [0, 0]
        return labels, edges, event_ids


def _relax(labels: list[float], edges: list[_Edge]) -> int:
    """Max-relaxation Bellman-Ford on the constraint edges — the DAG's unique fixed
    point. Every pass reads every edge in construction order; a pass that improves
    nothing stops the loop (the graph is acyclic, so labels are finite everywhere:
    every vertex chains from some pinned source). -inf + lb stays -inf, so an edge
    out of an unrelaxed vertex never fires. Returns the count of relaxations that
    strictly improved a label — the algorithm's expanded_nodes."""
    expanded = 0
    changed = True
    while changed:
        changed = False
        for e in edges:
            cand = labels[e.src] + e.lb
            if cand > labels[e.dst]:
                labels[e.dst] = cand
                expanded += 1
                changed = True
    return expanded
