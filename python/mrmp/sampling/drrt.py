"""dRRT — Solovey, Salzman & Halperin 2016 (IJRR 35(5):501–513; arXiv:1305.2889).

The sampling branch's implicit-roadmap pole ("MRdRRT" in the paper). The
composite roadmap is never built: each robot gets its OWN PRM roadmap G_i over
the continuous free space (start and goal are always vertices), and the composite
roadmap G is their TENSOR product — a joint vertex is a collision-free placement
tuple, and a joint edge moves EVERY robot simultaneously along its own roadmap
edge (a PRM roadmap has no self-loops: every tensor edge moves everyone; waiting
happens only in the sequential connector phase). dRRT explores this
exponentially-large implicit graph the way RRT explores a continuous space: grow
a tree T rooted at the start tuple toward uniform joint samples, and connect it
to the goal tuple with a local connector. Probabilistically complete (Theorem 1),
no optimality guarantee — the paper itself lists optimality as future work; dRRT*
is that later wave.

Semantics fixed here (identical in C++/TS; paper section per decision):

* Individual roadmap G_i (§2). Vertices: [start, goal] first (insertion order),
  then n rejection-sampled points — uniform over the map's world extent, kept
  only where free_point(q, r) holds. Edges: for each vertex v in insertion
  order, its k nearest vertices by Euclidean distance (ties: lower insertion
  index first); an edge exists iff segment_free(v→w, r_i) — evaluated exactly
  once per pair with the EARLIER-inserted endpoint as segment start, so both
  languages evaluate bit-identical expressions. Adjacency lists are sorted by
  ascending vertex index (the oracle's tie-break reads them in this order).
* Direction oracle O_D (§3.1, §4.1). For joint sample q and tree node C, the
  candidate neighbor is (c'_1,...,c'_m) where c'_i is the adjacency-list neighbor
  of c_i whose ray from c_i makes the SMALLEST angle with the ray to q_i —
  implemented as argmax cosine similarity over the ascending-ordered adjacency
  list, strict > so ties keep the lower index. If any robot's vertex has no
  neighbors at all, O_D = ∅ and the sample is ignored (§4.1: "the new sample is
  ignored and another sample is drawn").
* Tensor edge validity. The candidate joint vertex joins the tree only if no
  robot-robot collision occurs while everyone moves simultaneously: for every
  pair i<j, moving_pair_distance(c_i→c'_i, c_j→c'_j) >= r_i + r_j (collision is
  strict overlap; the relative-motion segment covers both endpoints too, so this
  one check also keeps accepted vertices pairwise collision-free — the root is
  checked explicitly instead).
* Tree growth (§3.2, Algorithm 1–2). Round i = 1..max_rounds: EXPAND runs
  N = 2^i iterations (the paper's parameter-free schedule): sample q_rand uniform
  over the joint embedding box (per-agent x-then-y draws from ONE shared PRNG
  stream — roadmap construction consumed this same stream first), NEAREST tree
  node by Euclidean distance over concatenated coordinates (ties: earliest
  insertion), O_D, accept iff valid and not already a tree vertex. Then
  CONNECT_TO_TARGET with K = i: for the K nearest tree nodes to the goal tuple
  (ties: earliest insertion) try the local connector; first success wins — when
  the goal tuple itself is a tree node it ranks first (distance 0) and its
  connector moves nobody.
* Local connector (§3.2, §4.2 — van den Berg et al. 2009 prioritization). For a
  tree node q, each robot i gets π_i = the hop-shortest BFS path on G_i from q's
  vertex to the goal vertex (fixed ascending neighbor order; first discovery is
  the parent). Priorities: if moving along π_i would collide with robot j parked
  at its connector-start v_j, then i moves AFTER j; if it would collide with j
  parked at its GOAL t_j, then i moves BEFORE j. The induced directed graph must
  be acyclic — a cycle fails this candidate (the next of the K is tried).
  Execution follows Kahn topological order, lowest index first among ready robots:
  one robot walks its π_i one roadmap edge per tick while every other robot waits
  in place. Acyclicity is exactly what makes execution collision-free: when i
  moves, every robot whose parked disc could touch π_i has already left it behind.
* Retrieval (§3.2 Algorithm 1 line 6): the tree chain root → q concatenated per
  agent (every tree tick moves ALL robots simultaneously — tensor edges), then the
  connector's sequential ticks. Each edge traversal is one time step, exactly like
  the discrete branch's unit-cost actions; each agent's path is trimmed after its
  LAST move (trailing waits at one's own goal are not motion — same metric as
  every planner here: a step costs 1 unless it is a wait at one's own goal).
* Termination. Success = the connector succeeded from some tree node. Budget
  exhausted → honest "no solution found within budget", never "unsolvable" —
  dRRT is probabilistically complete, not complete. Instance verdicts that ARE
  final: a start or goal disc overlapping an obstacle cell; two START discs
  overlapping (no valid initial configuration exists at all); two GOAL discs
  overlapping (every plan must hold both discs simultaneously at its end — no
  valid final configuration ever will). A start overlapping ANOTHER robot's goal
  is deliberately NOT a verdict: that robot can vacate before the other arrives.

Determinism: the PRNG is the same MINSTD Lehmer generator as MA-RRT*/sRRT (s ←
16807·s mod 2³¹−1, u = s/(2³¹−1) ∈ (0,1)); roadmap construction draws first
(agent 0's n samples, then agent 1's, …), the tree phase after it. Every float
expression (cosine oracle, Euclidean distances, moving-pair distance, free/segment
predicates) has a fixed operation order mirrored bit-for-bit in C++ and TS.
"""

from __future__ import annotations

import math

from ..core.capabilities import Capability, ContinuousSpace
from ..core.geometry import moving_pair_distance, point_segment_distance
from ..core.planner import ContinuousMultiAgentPlanner
from ..core.trace import TraceRecorder
from ..core.types import ContinuousPlanResult, ContinuousTask, PlanStats, Point


class _Lehmer:
    """MINSTD Lehmer PRNG: s ← 16807·s mod (2³¹−1), u = s/(2³¹−1) ∈ (0,1).

    Integer-exact in Python and C++ int64 and exact in IEEE doubles everywhere,
    so the three engines draw byte-identical streams from the same seed."""

    def __init__(self, seed: int) -> None:
        self._s = seed

    def uniform(self) -> float:
        self._s = (16807 * self._s) % 2147483647
        return self._s / 2147483647.0


def _dist(a: Point, b: Point) -> float:
    """Euclidean distance — fixed expression order (sqrt of the sum of squared
    differences; hypot would round differently)."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    return math.sqrt(dx * dx + dy * dy)


def _cosine(q: Point, c: Point, v: Point) -> float:
    """The oracle's angle ranking: the cosine of the angle between rays ρ(c,q)
    and ρ(c,v). Fixed expression order; argmax over it == argmin over the angle.
    A degenerate ray (either endpoint pair at identical points — only possible
    when a robot's start EQUALS its goal) has no angle: cosine is fixed to 0.0,
    which under strict > leaves the ascending-list order as the tie-break."""
    dx1 = q[0] - c[0]
    dy1 = q[1] - c[1]
    dx2 = v[0] - c[0]
    dy2 = v[1] - c[1]
    denom = math.sqrt(dx1 * dx1 + dy1 * dy1) * math.sqrt(dx2 * dx2 + dy2 * dy2)
    if denom == 0.0:
        return 0.0
    return (dx1 * dx2 + dy1 * dy2) / denom


class Drrt(ContinuousMultiAgentPlanner):
    @property
    def name(self) -> str:
        return "drrt"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.CONTINUOUS_SPACE}

    def plan(
        self,
        space: ContinuousSpace,
        tasks: list[ContinuousTask],
        recorder: TraceRecorder | None = None,
    ) -> ContinuousPlanResult:
        seed = self.params.get_int("seed")
        samples_per_robot = self.params.get_int("samples_per_robot")
        k_fanout = self.params.get_int("roadmap_k")
        max_rounds = self.params.get_int("max_rounds")

        m = len(tasks)
        radii = [task.radius for task in tasks]
        starts = [task.start for task in tasks]
        goals = [task.goal for task in tasks]

        def fail(expanded: int) -> ContinuousPlanResult:
            if recorder is not None:
                recorder.planning_finished(
                    False, {"expanded_nodes": float(expanded), "makespan": 0.0, "sum_of_costs": 0.0}
                )
            return ContinuousPlanResult(
                False, [], 0.0, makespan=0.0, stats=PlanStats(expanded_nodes=expanded)
            )

        # Instance verdicts (not budget): a disc overlapping an obstacle cell at
        # its start or goal makes the instance unsolvable outright. Pairwise overlap
        # is final at BOTH ends: two starts overlapping means no valid initial
        # configuration exists; two goals overlapping means no valid FINAL one ever
        # will (every plan holds both discs at their goals at once). A start that
        # overlaps another robot's goal is NOT a verdict — i can vacate before j
        # arrives, so such instances stay plannable.
        for i in range(m):
            start_free = space.free_point(starts[i], radii[i])
            goal_free = space.free_point(goals[i], radii[i])
            if not (start_free and goal_free):
                return fail(0)
        for i in range(m):
            for j in range(i + 1, m):
                if _dist(starts[i], starts[j]) < radii[i] + radii[j]:
                    return fail(0)
                if _dist(goals[i], goals[j]) < radii[i] + radii[j]:
                    return fail(0)

        rng = _Lehmer(seed)
        x_min, y_min, x_max, y_max = space.extent()

        # --- individual roadmaps G_i (§2) --------------------------------------
        # Vertices: [start, goal] then n rejection samples in draw order (one
        # shared PRNG stream: agent 0's samples first). Edges: each vertex links
        # to its k nearest by Euclidean distance (ties: lower insertion index);
        # an edge exists iff the swept disc of the straight segment stays clear.
        vertices: list[list[Point]] = []
        adjacency: list[list[list[int]]] = []
        for i in range(m):
            vi: list[Point] = [starts[i], goals[i]]
            while len(vi) < samples_per_robot + 2:
                sx = x_min + rng.uniform() * (x_max - x_min)
                sy = y_min + rng.uniform() * (y_max - y_min)
                if space.free_point((sx, sy), radii[i]):
                    vi.append((sx, sy))
            edge_set: set[tuple[int, int]] = set()
            for v in range(len(vi)):
                # Candidates sorted by (distance ascending, index ascending); the
                # first k are candidates. A pair already accepted is skipped; a
                # pair rejected earlier re-checks to the same verdict because the
                # segment is always evaluated from the EARLIER-inserted endpoint.
                cands = sorted(
                    ((_dist(vi[v], vi[w]), w) for w in range(len(vi)) if w != v),
                    key=lambda item: (item[0], item[1]),
                )
                for _, w in cands[:k_fanout]:
                    pair = (v, w) if v < w else (w, v)
                    if pair in edge_set:
                        continue
                    if space.segment_free(vi[pair[0]], vi[pair[1]], radii[i]):
                        edge_set.add(pair)
            adj_i: list[list[int]] = [[] for _ in vi]
            for a, b in sorted(edge_set):  # (min,max) order — the trace's edge order too
                adj_i[a].append(b)
                adj_i[b].append(a)
            vertices.append(vi)
            adjacency.append(adj_i)
            if recorder is not None:
                recorder.roadmap_built(i, vi, sorted(edge_set))

        # --- the implicit composite roadmap's tree T (§3.2) ---------------------
        # Joint vertices are tuples of per-agent vertex indices; every agent's
        # start is index 0 and goal index 1 of its own roadmap. The "goal tuple"
        # is not a fixed index tuple here — it is the joint POINTS below: a tree
        # node whose coordinates equal them sits at distance 0 and ranks first.
        root: tuple[int, ...] = tuple(0 for _ in range(m))
        goal_points: list[Point] = [vertices[i][1] for i in range(m)]

        def joint_distance(a: tuple[int, ...], q: list[Point]) -> float:
            # Euclidean distance over concatenated coordinates — squared diffs
            # summed in fixed order (agent 0 x,y then agent 1 x,y, …), one sqrt.
            total = 0.0
            for i in range(m):
                dx = vertices[i][a[i]][0] - q[i][0]
                dy = vertices[i][a[i]][1] - q[i][1]
                total += dx * dx + dy * dy
            return math.sqrt(total)

        def flatten(state: tuple[int, ...]) -> list[float]:
            flat: list[float] = []
            for i in range(m):
                p = vertices[i][state[i]]
                flat.extend([p[0], p[1]])
            return flat

        states: list[tuple[int, ...]] = [root]
        parents_joint: list[int] = [-1]
        index_of: dict[tuple[int, ...], int] = {root: 0}
        if recorder is not None:
            recorder.node_expanded(flatten(root))

        def oracle(state: tuple[int, ...], q: list[Point]) -> tuple[int, ...] | None:
            """O_D (§3.1/§4.1): per robot, the adjacency neighbor whose ray from
            the current vertex makes the smallest angle to the sample's point —
            argmax cosine over the ascending-ordered list (strict > keeps the
            lower index on ties). Any robot without neighbors → ∅ → ignored."""
            candidate: list[int] = []
            for i in range(m):
                c_idx = state[i]
                nbrs = adjacency[i][c_idx]
                if not nbrs:
                    return None
                c_point = vertices[i][c_idx]
                q_point = q[i]
                best_idx = -1
                best_cos = 0.0
                first = True
                for w in nbrs:
                    cos = _cosine(q_point, c_point, vertices[i][w])
                    if first or cos > best_cos:
                        best_cos = cos
                        best_idx = w
                        first = False
                candidate.append(best_idx)
            return tuple(candidate)

        def edge_valid(prev: tuple[int, ...], nxt: tuple[int, ...]) -> bool:
            """Tensor edge: everyone moves at once; valid iff no pair of discs
            strictly overlaps anywhere along the simultaneous motion (the
            relative-motion segment covers both endpoints too)."""
            for i in range(m):
                for j in range(i + 1, m):
                    d = moving_pair_distance(
                        vertices[i][prev[i]], vertices[i][nxt[i]],
                        vertices[j][prev[j]], vertices[j][nxt[j]],
                    )
                    if d < radii[i] + radii[j]:
                        return False
            return True

        def bfs_local(start_idx: int, agent: int) -> list[Point] | None:
            """Hop-shortest path on G_agent from the node's vertex to its goal
            (vertex 1); fixed ascending neighbor order, first discovery wins. The
            parent chain is walked BACKWARD from the goal and reversed — the
            returned point path STARTS at the node's vertex and ENDS at the goal."""
            tree: dict[int, int] = {start_idx: start_idx}
            frontier = [start_idx]
            while 1 not in tree:
                nxt_frontier: list[int] = []
                for c in frontier:
                    for w in adjacency[agent][c]:
                        if w not in tree:
                            tree[w] = c
                            nxt_frontier.append(w)
                if not nxt_frontier:
                    return None  # goal vertex unreachable on this roadmap
                frontier = nxt_frontier
            chain: list[int] = []
            node = 1
            while True:
                chain.append(node)
                if tree[node] == node:
                    break
                node = tree[node]
            chain.reverse()  # goal → start is the parent-walk order; motion is not
            return [vertices[agent][idx] for idx in chain]

        def path_hits_point(path: list[Point], p: Point, radius_sum: float) -> bool:
            """A robot moving along `path` overlaps a disc of radius_sum parked at
            p iff some motion segment comes strictly closer than radius_sum."""
            for t in range(len(path) - 1):
                if point_segment_distance(p, path[t], path[t + 1]) < radius_sum:
                    return True
            return False

        def connect(q_state: tuple[int, ...]) -> list[list[Point]] | None:
            """§4.2 prioritized decoupling: per-robot BFS paths + priority DAG;
            acyclic → execute in Kahn order (lowest index first among ready). The
            returned paths START at q_state's points (the caller splices them onto
            the tree chain); cyclic or unreachable → None, next candidate."""
            paths_local: list[list[Point]] = []
            for i in range(m):
                pi = bfs_local(q_state[i], i)
                if pi is None:
                    return None
                paths_local.append(pi)

            # Priority DAG. π_i hitting j's parked disc at v_j → i after j; π_i
            # hitting j's goal disc → i before j. Both directions for one pair is
            # a cycle — the candidate honestly fails.
            deps: list[set[int]] = [set() for _ in range(m)]  # deps[i]: must move before i
            for i in range(m):
                for j in range(i + 1, m):
                    r_sum = radii[i] + radii[j]
                    if path_hits_point(paths_local[i], vertices[j][q_state[j]], r_sum):
                        deps[i].add(j)  # π_i hits v_j → i moves after j
                    if path_hits_point(paths_local[j], vertices[i][q_state[i]], r_sum):
                        deps[j].add(i)  # π_j hits v_i → j moves after i
                    if path_hits_point(paths_local[i], goals[j], r_sum):
                        deps[j].add(i)  # π_i hits t_j → i moves before j
                    if path_hits_point(paths_local[j], goals[i], r_sum):
                        deps[i].add(j)  # π_j hits t_i → j moves before i

            # Kahn topological order, lowest index first among ready robots.
            order: list[int] = []
            done: set[int] = set()
            while len(order) < m:
                ready = next((r for r in range(m) if r not in done and deps[r] <= done), None)
                if ready is None:
                    return None  # cyclic priorities — this candidate fails
                order.append(ready)
                done.add(ready)

            # Execute sequentially: each robot walks its π_i one edge per tick;
            # everyone else waits in place (their path repeats its last point).
            paths: list[list[Point]] = [[vertices[i][q_state[i]]] for i in range(m)]
            for mover in order:
                pi = paths_local[mover]
                for t in range(1, len(pi)):
                    for a in range(m):
                        paths[a].append(pi[t] if a == mover else paths[a][-1])
            return paths

        # --- main loop (Algorithm 1) ---------------------------------------------
        connector_paths: list[list[Point]] | None = None
        chain_nodes: list[int] = []
        for round_i in range(1, max_rounds + 1):
            # EXPAND: N = 2^i sample-and-extend iterations (the paper's
            # parameter-free schedule — the round index IS the budget).
            for _ in range(2**round_i):
                q_rand = [
                    (
                        x_min + rng.uniform() * (x_max - x_min),
                        y_min + rng.uniform() * (y_max - y_min),
                    )
                    for _ in range(m)
                ]
                near_idx = 0
                near_d = joint_distance(states[0], q_rand)
                for idx in range(1, len(states)):
                    d = joint_distance(states[idx], q_rand)
                    if d < near_d:  # strict < — earliest insertion wins ties
                        near_d = d
                        near_idx = idx
                candidate = oracle(states[near_idx], q_rand)
                if candidate is None or candidate in index_of:
                    continue  # O_D = ∅, or already a tree vertex (no-op sample)
                if not edge_valid(states[near_idx], candidate):
                    continue  # simultaneous motion collides — sample ignored
                states.append(candidate)
                parents_joint.append(near_idx)
                index_of[candidate] = len(states) - 1
                if recorder is not None:
                    recorder.node_expanded(flatten(candidate))

            # CONNECT TO TARGET: K = round_i nearest tree nodes to the goal tuple.
            ranked = sorted(
                ((joint_distance(s, goal_points), idx) for idx, s in enumerate(states)),
                key=lambda item: (item[0], item[1]),
            )[:round_i]
            for _, node_idx in ranked:
                q_state = states[node_idx]
                connected = connect(q_state)
                if connected is None:
                    continue
                # RETRIEVE PATH: the tree chain root → q (all robots move on every
                # tree tick), then the connector's sequential ticks spliced on.
                chain: list[int] = []
                node = node_idx
                while node != -1:
                    chain.append(node)
                    node = parents_joint[node]
                chain.reverse()
                chain_nodes = chain
                connector_paths = connected
                break
            if connector_paths is not None:
                break

        if connector_paths is None:
            return fail(len(states))

        # Tree phase: every tree edge moves ALL robots simultaneously (tensor
        # product — no self-loops on the individual roadmaps).
        paths: list[list[Point]] = [[vertices[a][root[a]]] for a in range(m)]
        for node_idx in chain_nodes[1:]:
            state = states[node_idx]
            for a in range(m):
                paths[a].append(vertices[a][state[a]])
        # Connector phase: splice the sequential ticks on (skip the duplicated head).
        for a in range(m):
            paths[a].extend(connector_paths[a][1:])

        # Trim after each agent's LAST move, then the unit-cost metric (a step
        # costs 1 unless it is a wait at one's own goal — sRRT wave convention).
        trimmed: list[list[Point]] = []
        cost = 0.0
        for i, path in enumerate(paths):
            last_move = 0
            for t in range(1, len(path)):
                if path[t] != path[t - 1]:
                    last_move = t
            trimmed.append(path[: last_move + 1])
            for t in range(1, len(trimmed[-1])):
                if not (trimmed[-1][t] == trimmed[-1][t - 1] and trimmed[-1][t] == goals[i]):
                    cost += 1.0

        makespan = float(max(len(p) - 1 for p in trimmed))
        if recorder is not None:
            for i, path in enumerate(trimmed):
                recorder.path_found(path, i)
            recorder.planning_finished(
                True,
                {
                    "expanded_nodes": float(len(states)),
                    "makespan": makespan,
                    "sum_of_costs": cost,
                },
            )
        return ContinuousPlanResult(
            True, trimmed, cost,
            makespan=makespan, stats=PlanStats(expanded_nodes=len(states)),
        )
