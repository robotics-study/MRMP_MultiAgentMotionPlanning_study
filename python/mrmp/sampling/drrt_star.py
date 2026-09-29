"""dRRT* — Shome, Solovey, Dobson, Halperin & Bekris (Autonomous Robots 44(3-4):
443–467, 2020; conference version Dobson, Shome, Halperin & Bekris, IROS 2017).

The asymptotically-optimal chapter of the implicit-roadmap genealogy. The same
tensor-product roadmap as dRRT — each robot keeps its OWN PRM over the continuous
free space — but both halves change:

* The individual roadmaps G_i switch from k-nearest to the PRM* CONNECTION RADIUS
  r(n) = (1+eta)^2 * sqrt(mu(C_f) * log n / (d * zeta_d * n)) at d = 2, zeta_2 = pi
  (Theorem 1's bound; mu(C_f) is the raster's own free-cell measure). And Adj(v_i,
  G_i) now INCLUDES v_i itself: a tensor edge may move a subset of the robots —
  waiting is native to the graph, so dRRT's decoupled local connector is gone. The
  goal tuple is reached by tree search alone.
* The tree search switches from direction-oracle growth to RRT*-style rewiring
  (Algorithms 6/7/8 of the paper). Every joint vertex carries a cost-to-come —
  the sum of per-robot arc lengths along the tree path (the paper's primary cost
  function; max_i ‖σ_i‖ is its second one) — candidates are re-parented when
  cheaper, and branch-and-bound refuses any expansion whose candidate cost exceeds
  the incumbent solution. The oracle becomes I_d: a goal-biased sample switches
  every robot to argmin-H (H = shortest-path length on G_i to that robot's goal
  vertex, precomputed by Dijkstra); an unbiased sample leaves each robot on a
  uniformly random neighbor of its current tree vertex. Child promotion: when a
  GENERATED node improves H over its chosen parent, IT becomes the next V_last and
  the next expansion is greedy (q_rand = T) — that is how the tree dives at the
  goal; re-expanding an EXISTING state never promotes, so once the goal tuple is
  in the tree greedy mode ends and plain exploration resumes.

Anytime by construction: the loop never stops on first success; it spends every
iteration growing and rewiring while branch-and-bound prunes what cannot beat the
incumbent, so the returned chain is the best found within the budget (the paper's
outer/inner loop split degenerates at n_it = 1 — the solution check runs every
iteration). Budget exhaustion without ever reaching the goal tuple is honest "no
solution found", never "unsolvable". The instance verdicts are final and
identical to dRRT's: a start or goal disc overlapping an obstacle cell, two START
discs overlapping (no valid initial configuration), or two GOAL discs overlapping
(no valid FINAL one).

Semantics fixed here (identical in C++/TS; paper section per decision):

* Roadmap (§4 + Theorem 1's radius). Vertices: [start, goal] first (insertion
  order), then n rejection samples — uniform over the map's world extent, kept
  only where free_point(q, r) holds. An edge exists iff dist(v, w) < r(n) AND the
  swept-disc check from the EARLIER-inserted endpoint passes (both languages
  evaluate bit-identical expressions). Adjacency lists are ascending vertex-index
  lists INCLUDING the self-loop at own index — waiting is a graph edge here.
* Expansion (Alg. 7). V_last starts as the root, so the first expansion is greedy.
  Greedy mode: q_rand IS the goal tuple (no draws at all). Exploration mode: one
  bias draw decides; an unbiased sample draws per-agent coordinates (x-then-y,
  agent order), then each robot still on the random branch draws its uniform
  neighbor pick (agent order). V_near = nearest tree node by concatenated
  Euclidean distance (strict <, earliest insertion wins ties) in exploration mode,
  V_last itself in greedy mode. I_d per robot: exact goal equality switches to
  argmin H over Adj (strict < keeps the lower index on ties; an all-inf argmin
  resolves to the lowest index too).
* Candidate handling (Alg. 7 lines 8–19). N = Adj(V_new, G_hat) ∩ T scanned in
  ascending insertion order; a candidate edge must also be collision-free for the
  OTHER robots (moving_pair_distance >= r_i + r_j — the same tensor-edge validity
  as dRRT; a motionless robot contributes its point against the mover's segment).
  V_best = argmin over valid candidates of c(U) + w(U, V_new), strict < so the
  earliest-inserted candidate wins exact ties; no valid candidate refuses the
  expansion. Branch-and-bound: candidate cost > incumbent (strict) refuses it too.
  Adding appends to the children lists (ascending index); re-parenting requires a
  STRICT improvement and recomputes the moved subtree's costs top-down. Triangle
  inequality makes cycles structurally impossible: an ancestor's cost already
  dominates the direct edge, so the strict-improvement test can never fire upward.
  Then every U in N (ascending) is re-parented under V_new when that strictly
  improves c(U). Child promotion: a GENERATED node with H(V_new) < H(V_best)
  becomes the next V_last; anything else — refused expansion, or an expansion
  whose state was already in T (no node generated) — resets V_last to ∅.
* Cost and retrieval. Edge weight w(u, v) = sum_i ‖v_i - u_i‖ (fixed ascending
  order); c(root) = 0; c(child) = c(parent) + w — so c(V) is exactly the sum of
  per-robot arc lengths along the tree chain. The solution is the tree chain root
  → goal tuple (each agent's waypoint list trimmed after its LAST move; trailing
  waits add zero length). Reported metrics: sum_of_costs = Σ_i ‖σ_i‖ and
  makespan = max_i ‖σ_i‖, recomputed from the traced paths in fixed order — the
  paper's own two cost functions, reparameterization-invariant, which is exactly
  why the execution replay's tick grid stays display-only.

Determinism: the PRNG is the same MINSTD Lehmer generator as MA-RRT*/sRRT/dRRT
(s ← 16807·s mod 2³¹−1, u = s/(2³¹−1) ∈ (0,1)); roadmap construction draws first
(agent 0's n samples x-then-y, then agent 1's, …), the tree phase after it: per
exploration iteration one bias draw, then coordinates, then neighbor picks — a
greedy (V_last ≠ ∅) iteration draws NOTHING. Every float expression (Euclidean
distance, edge weights, arc lengths, the radius bound itself) has a fixed
operation order mirrored bit-for-bit in C++ and TS.
"""

from __future__ import annotations

import heapq
import math

from ..core.capabilities import Capability, ContinuousSpace
from ..core.geometry import moving_pair_distance
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


class DrrtStar(ContinuousMultiAgentPlanner):
    @property
    def name(self) -> str:
        return "drrt_star"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.CONTINUOUS_SPACE}

    def plan(
        self,
        space: ContinuousSpace,
        tasks: list[ContinuousTask],
        recorder: TraceRecorder | None = None,
    ) -> ContinuousPlanResult:
        seed = self.params.get_int("seed")
        n_samples = self.params.get_int("samples_per_robot")
        eta = self.params.get_float("eta")
        goal_sample_rate = self.params.get_float("goal_sample_rate")
        max_iterations = self.params.get_int("max_iterations")

        m = len(tasks)
        radii = [task.radius for task in tasks]
        starts = [task.start for task in tasks]
        goals = [task.goal for task in tasks]

        def fail(expanded: int) -> ContinuousPlanResult:
            if recorder is not None:
                recorder.planning_finished(
                    False, {"expanded_nodes": float(expanded), "makespan": 0.0, "sum_of_costs": 0.0}
                )
            return ContinuousPlanResult(False, [], 0.0, makespan=0.0,
                                       stats=PlanStats(expanded_nodes=expanded))

        # Instance verdicts (not budget) — identical to dRRT: a disc overlapping an
        # obstacle cell at its start or goal; two starts overlapping (no valid
        # initial configuration); two goals overlapping (no valid FINAL one). A
        # start overlapping ANOTHER robot's goal is NOT a verdict — i can vacate.
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

        # Theorem 1's radius at d = 2 (zeta_2 = pi), verbatim: r(n) = gamma *
        # sqrt(log n / n) with gamma = (1+eta)^2 * sqrt(mu(C_f) / (2*pi)) folded
        # into one factor. Fixed expression order — identical bits in every engine.
        nf = float(n_samples)
        radius_r = ((1.0 + eta) * (1.0 + eta)) * math.sqrt(
            (space.area() * math.log(nf)) / (2.0 * math.pi * nf)
        )

        # --- individual roadmaps G_i (§4): [start, goal] first, n rejection
        # samples after; an edge exists iff dist < r(n) AND the swept-disc check
        # from the EARLIER-inserted endpoint passes. Adjacency keeps ascending
        # index order and INCLUDES the self-loop at own index — waiting is a
        # graph edge here (the paper's footnote to §4.1). ---
        vertices: list[list[Point]] = []
        adjacency: list[list[list[int]]] = []
        for i in range(m):
            vi: list[Point] = [starts[i], goals[i]]
            while len(vi) < n_samples + 2:
                sx = x_min + rng.uniform() * (x_max - x_min)
                sy = y_min + rng.uniform() * (y_max - y_min)
                if space.free_point((sx, sy), radii[i]):
                    vi.append((sx, sy))
            edge_set: set[tuple[int, int]] = set()
            for v in range(len(vi)):
                for w in range(v + 1, len(vi)):
                    if _dist(vi[v], vi[w]) < radius_r and space.segment_free(
                        vi[v], vi[w], radii[i]
                    ):
                        edge_set.add((v, w))
            adj_i: list[list[int]] = []
            for v in range(len(vi)):
                adj_i.append([w for w in range(len(vi))
                              if w == v or (min(v, w), max(v, w)) in edge_set])
            vertices.append(vi)
            adjacency.append(adj_i)
            if recorder is not None:
                recorder.roadmap_built(i, vi, sorted(edge_set))

        # H_i(v): shortest-path LENGTH on G_i from v to that robot's goal vertex
        # (index 1), Dijkstra over the roadmap edges; +inf when unreachable (an
        # all-inf argmin resolves to the lowest index like every other tie here).
        def dijkstra(i: int) -> list[float]:
            n_i = len(vertices[i])
            dist_h = [math.inf] * n_i
            dist_h[1] = 0.0
            queue: list[tuple[float, int]] = [(0.0, 1)]
            while queue:
                d_u, u = heapq.heappop(queue)
                if d_u > dist_h[u]:
                    continue
                for w in adjacency[i][u]:
                    w_len = _dist(vertices[i][u], vertices[i][w])
                    if d_u + w_len < dist_h[w]:
                        dist_h[w] = d_u + w_len
                        heapq.heappush(queue, (dist_h[w], w))
            return dist_h

        h_table = [dijkstra(i) for i in range(m)]

        root: tuple[int, ...] = tuple(0 for _ in range(m))
        goal_tuple: tuple[int, ...] = tuple(1 for _ in range(m))

        def flatten(state: tuple[int, ...]) -> list[float]:
            flat: list[float] = []
            for i in range(m):
                p = vertices[i][state[i]]
                flat.extend([p[0], p[1]])
            return flat

        def joint_distance(a: tuple[int, ...], q: list[Point]) -> float:
            # Euclidean over concatenated coordinates — squared diffs summed in
            # ascending agent order (one sqrt), fixed like everything else.
            total = 0.0
            for i in range(m):
                dx = vertices[i][a[i]][0] - q[i][0]
                dy = vertices[i][a[i]][1] - q[i][1]
                total += dx * dx + dy * dy
            return math.sqrt(total)

        def edge_weight(a: tuple[int, ...], b: tuple[int, ...]) -> float:
            # w(u, v) = sum_i ‖v_i - u_i‖ — ascending agent order, fixed.
            total = 0.0
            for i in range(m):
                total += _dist(vertices[i][a[i]], vertices[i][b[i]])
            return total

        def edge_valid(a: tuple[int, ...], b: tuple[int, ...]) -> bool:
            # A tensor edge is valid iff no pair of discs strictly overlaps while
            # everyone moves (a motionless robot contributes its point against the
            # mover's segment — moving_pair_distance covers both endpoints).
            for i in range(m):
                for j in range(i + 1, m):
                    d = moving_pair_distance(
                        vertices[i][a[i]], vertices[i][b[i]],
                        vertices[j][a[j]], vertices[j][b[j]],
                    )
                    if d < radii[i] + radii[j]:
                        return False
            return True

        def h_of(state: tuple[int, ...]) -> float:
            # Composite heuristic: the SUM of per-robot shortest-path lengths (the
            # cost function is a sum too). inf propagates; inf < inf is false.
            total = 0.0
            for i in range(m):
                total += h_table[i][state[i]]
            return total

        states: list[tuple[int, ...]] = [root]
        parents: list[int] = [-1]
        costs: list[float] = [0.0]
        children: list[list[int]] = [[]]
        index_of: dict[tuple[int, ...], int] = {root: 0}
        if recorder is not None:
            recorder.node_expanded(flatten(root))

        def rewire(node: int, new_parent: int) -> None:
            """Re-parent node under new_parent (strictly cheaper by the caller's
            check) and recompute the moved subtree top-down. Children lists stay
            ascending, so this BFS order is deterministic; every descendant's cost
            is recomputed from its (already-updated) parent."""
            old = parents[node]
            if old >= 0:
                children[old].remove(node)
            parents[node] = new_parent
            siblings = children[new_parent]
            pos = 0
            while pos < len(siblings) and siblings[pos] < node:
                pos += 1
            siblings.insert(pos, node)
            costs[node] = costs[new_parent] + edge_weight(states[new_parent], states[node])
            queue = [node]
            while queue:
                x = queue.pop(0)
                for ch in children[x]:
                    costs[ch] = costs[x] + edge_weight(states[x], states[ch])
                    queue.append(ch)

        incumbent_cost = math.inf
        best_paths: list[list[Point]] | None = None
        v_last: int | None = 0  # Algorithm 6 line 1: V_last <- S (greedy first)

        for _ in range(max_iterations):
            if v_last is None:
                # Exploration: one bias draw decides; an unbiased sample draws the
                # joint sample per agent x-then-y, and each robot then draws its
                # uniform neighbor pick (agent order). A biased sample IS the goal
                # tuple — every robot takes the guided branch, no further draws.
                u_bias = rng.uniform()
                if u_bias < goal_sample_rate:
                    q_rand: list[Point] = list(goals)
                else:
                    q_rand = []
                    for _ in range(m):
                        sx = x_min + rng.uniform() * (x_max - x_min)
                        sy = y_min + rng.uniform() * (y_max - y_min)
                        q_rand.append((sx, sy))
                near_idx = 0
                near_d = joint_distance(states[0], q_rand)
                for idx in range(1, len(states)):
                    d = joint_distance(states[idx], q_rand)
                    if d < near_d:  # strict < — earliest insertion wins ties
                        near_d = d
                        near_idx = idx
            else:
                # Greedy child propagation (Alg. 7 lines 4-6): the sample IS the
                # goal tuple and V_near is V_last itself; every robot's q_i equals
                # its goal exactly, so I_d takes the guided branch for all of them.
                q_rand = list(goals)
                near_idx = v_last

            # I_d (Alg. 8), robots in ascending order: exact goal equality switches
            # to argmin H over Adj; otherwise a uniform random neighbor pick. The
            # picks are VERTEX INDICES — the joint state is an index tuple.
            v_new_list: list[int] = []
            for i in range(m):
                adj = adjacency[i][states[near_idx][i]]
                if q_rand[i] == goals[i]:
                    # argmin H over Adj; strict < keeps the LOWEST index on exact
                    # ties — and an all-inf neighbourhood (this vertex's component
                    # never reaches the goal) is one big tie, so it resolves to
                    # adj[0], the lowest-index entry too.
                    best_idx = adj[0]
                    best_h = h_table[i][adj[0]]
                    for w in adj[1:]:
                        if h_table[i][w] < best_h:
                            best_h = h_table[i][w]
                            best_idx = w
                    v_new_list.append(best_idx)
                else:
                    pick = int(rng.uniform() * len(adj))  # u in (0,1) -> valid index
                    v_new_list.append(adj[pick])
            v_state = tuple(v_new_list)

            # N = Adj(V_new, G_hat) ∩ T — scan tree nodes in ascending insertion
            # order; adjacency is per-robot and reflexive (self-loop at own index).
            cand: list[int] = [
                u_idx for u_idx in range(len(states))
                if all(v_state[i] in adjacency[i][states[u_idx][i]] for i in range(m))
            ]

            # V_best = argmin over VALID candidates of c(U) + w(U, V_new); strict <
            # so the earliest-inserted candidate wins exact ties. No valid
            # candidate -> the expansion is refused (V_last <- ∅).
            best_parent = -1
            best_cost = math.inf
            for u_idx in cand:
                if not edge_valid(states[u_idx], v_state):
                    continue
                c_cand = costs[u_idx] + edge_weight(states[u_idx], v_state)
                if c_cand < best_cost:  # strict < — earliest insertion wins ties
                    best_cost = c_cand
                    best_parent = u_idx
            if best_parent == -1:
                v_last = None
                continue
            if best_cost > incumbent_cost:
                # Branch-and-bound (strict): nothing this expansion could become
                # beats the incumbent — refuse it outright.
                v_last = None
                continue

            inserted = v_state not in index_of
            if inserted:
                idx_new = len(states)
                states.append(v_state)
                parents.append(best_parent)
                costs.append(best_cost)
                children.append([])
                children[best_parent].append(idx_new)  # ascending: idx_new is largest
                index_of[v_state] = idx_new
                if recorder is not None:
                    recorder.node_expanded(flatten(v_state))
            else:
                idx_new = index_of[v_state]
                if best_cost < costs[idx_new]:
                    rewire(idx_new, best_parent)

            # Rewire pass (Alg. 7 lines 16-18): every tree neighbor V_new now owns
            # gets re-parented under V_new when that strictly improves its cost —
            # and it can never fire UPWARD: for an ancestor u, costs[idx_new] >=
            # costs[u] + w by the triangle inequality over per-agent lengths, so
            # costs[idx_new] + w < costs[u] is impossible. No cycles, ever.
            for u_idx in cand:
                if u_idx == idx_new:
                    continue
                if edge_valid(v_state, states[u_idx]):
                    w2 = edge_weight(v_state, states[u_idx])
                    if costs[idx_new] + w2 < costs[u_idx]:
                        rewire(u_idx, idx_new)

            # Child promotion (Alg. 7 line 19): a node GENERATED this iteration
            # becomes the next V_last iff it made heuristic progress toward the
            # goals over its chosen parent. Re-expanding an EXISTING state never
            # generates anything — even when the rewire pass above improved its
            # cost-to-come, nothing was produced, so V_last resets to empty and
            # exploration resumes next iteration. That is what keeps the loop
            # anytime after the goal tuple enters T: greedy mode would otherwise
            # keep re-expanding the goal state (its own argmin-H neighbor) forever.
            if inserted and h_of(v_state) < h_of(states[best_parent]):
                v_last = idx_new
            else:
                v_last = None

            # Connect to Target: the goal tuple is a tree node or the solution
            # does not exist yet; cost(π) is exactly c(goal node).
            g_idx = index_of.get(goal_tuple)
            if g_idx is not None and costs[g_idx] < incumbent_cost:
                chain: list[int] = []
                node = g_idx
                while node != -1:
                    chain.append(node)
                    node = parents[node]
                chain.reverse()
                # chain holds NODE indices root → goal node; agent i's waypoint at
                # step t is the vertex its index tuple selects at that node.
                raw_paths: list[list[Point]] = [[] for _ in range(m)]
                for node_idx in chain:
                    state = states[node_idx]
                    for i in range(m):
                        raw_paths[i].append(vertices[i][state[i]])
                # Trim each agent's waypoint list after its LAST move (trailing
                # waits add zero arc length; replay clamps past the end anyway).
                trimmed: list[list[Point]] = []
                for points in raw_paths:
                    last_move = 0
                    for t in range(1, len(points)):
                        if points[t] != points[t - 1]:
                            last_move = t
                    trimmed.append(points[: last_move + 1])
                incumbent_cost = costs[g_idx]
                best_paths = trimmed

        if best_paths is None:
            return fail(len(states))

        # Geometric metrics (the paper's own cost functions): sum and max over the
        # per-agent arc lengths, recomputed from the traced paths in fixed order.
        # (c(goal node) drove the search with the same quantities grouped edge-
        # first; float associativity makes the two groupings differ in low bits —
        # both are pinned, neither is "more true" than the other.)
        lengths: list[float] = []
        for path in best_paths:
            length = 0.0
            for t in range(1, len(path)):
                length += _dist(path[t - 1], path[t])
            lengths.append(length)
        sum_of_costs = 0.0
        makespan = 0.0
        for i in range(m):
            sum_of_costs += lengths[i]
            if lengths[i] > makespan:
                makespan = lengths[i]

        if recorder is not None:
            for i, path in enumerate(best_paths):
                recorder.path_found(path, i)
            recorder.planning_finished(
                True,
                {
                    "expanded_nodes": float(len(states)),
                    "makespan": makespan,
                    "sum_of_costs": sum_of_costs,
                },
            )
        return ContinuousPlanResult(True, best_paths, sum_of_costs,
                                    makespan=makespan,
                                    stats=PlanStats(expanded_nodes=len(states)))
