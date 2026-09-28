"""MA-RRT* — Čáp, Novák, Vokřínek & Pěchouček 2013 (AAMAS).

The sampling branch's coupled pole: RRT* (Karaman & Frazzoli 2011) run on the
JOINT state space of per-agent motion graphs. The paper's own discretization —
waypoints on a grid, unit-duration primitives over the 4-neighborhood plus wait —
is exactly this repo's `DiscreteSpace`, so MA-RRT* is RRT* in graph form (the
paper's G-RRT*, §4.1) lifted to n-tuples of waypoints (§4.2): a tree whose
vertices are joint states, grown by STEERING (the paper's GREEDY: every agent
greedily steps toward the sampled tuple's own coordinate, one joint step at a
time, aborting on any conflict), rewired by NEAR balls, and steered
asymptotically to optimality by RRT*'s parent-selection/rewiring.

Semantics fixed here (identical in C++/TS; paper section per decision):

* Motion graph (§3): vertices are free cells; primitives are the unit moves in
  the map's fixed order plus the wait self-loop, all of duration 1 at speed 1.
  A primitive costs 0 iff start = end = that agent's goal (waiting at one's own
  goal is free), else 1 — the paper's solution metric: summed time outside the
  goal (§3). Joint distance between two states is Σ_i Euclidean(x_i, y_i) /
  speed (= 1): a lower bound on joint-transition cost (§4.2), and the metric
  NEAREST/NEAR run on. The joint space has dimensionality d = 2k (each agent
  contributes one planar waypoint); that is what r_n's exponent 1/d uses.
* Tree growth (Algorithms 1–4): each iteration SAMPLEs a joint state — the goal
  tuple with probability p_goal (§4.3/§4.4 goal biasing), else every agent's
  waypoint drawn uniformly from the vertex set in agent order — then EXTEND:
  NEAREST finds the tree vertex closest to the sample (ties: lowest insertion
  index); GREEDY walks every agent simultaneously toward the sample's own
  coordinates, one joint step per tick, and returns its partial segment if a
  conflict or the c_max budget cuts it short. A non-empty segment adds a new
  tree vertex; NEAR(T, x_new, |V|) with r_n = max{γ(log n/n)^(1/d), m} (m = 1,
  the longest primitive — §4.1's requirement that the ball never shrink below
  an edge length) picks rewiring candidates: a candidate becomes the new parent
  only on STRICT cost improvement, and existing vertices rewire only on strict
  improvement through the new vertex.
* The paper leaves three things ambiguous; each gets an explicit choice here,
  identical in every language: (1) Algorithm 2 line 19 reads GREEDY(G_M, x,
  x_near) with x still naming the sample — read as a typo for x_new, since
  reaching x_near THROUGH the new vertex is the whole point of rewiring; (2)
  sampling a state already in the tree is a no-op (the paper's set-union line
  never says what a duplicate means for the parent edge); (3) GREEDY's argmin
  over children breaks ties by the map's fixed neighbor order.
* §4.4 optimizations, both on: goal biasing (above) and informed pruning — once
  a solution exists, a candidate vertex whose chain cost plus the metric lower
  bound to the goal tuple exceeds the incumbent solution cost is not added.
* The tree IS the answer: when the goal tuple becomes a vertex, the chain from
  root to it, concatenated per agent and trimmed after each agent's last move
  (trailing waits sit at the goal and cost 0), is the returned plan. The loop
  runs its whole budget — RRT* is anytime; rewiring keeps improving the chain
  long after the first solution.

Determinism: the PRNG is a MINSTD Lehmer generator (s ← 16807·s mod 2³¹−1,
u = s/(2³¹−1) ∈ (0,1)) — integer-exact in Python and C++ int64 and exact in
IEEE doubles everywhere, so all three engines draw identical streams. r_n is
quantized to a 1/64 lattice because log/pow are libm-dependent at the ULP
level; distances are sums of correctly-rounded sqrt over integer deltas, which
are bit-identical across languages already.
"""

from __future__ import annotations

import math

from ..core.capabilities import Capability, DiscreteSpace
from ..core.planner import MultiAgentPlanner
from ..core.trace import TraceRecorder
from ..core.types import AgentTask, Cell, MultiPlanResult, PlanStats


class _Lehmer:
    """MINSTD Lehmer PRNG: s ← 16807·s mod (2³¹−1), u = s/(2³¹−1) ∈ (0,1).

    Integer-exact in Python and C++ int64 and exact in IEEE doubles everywhere,
    so the three engines draw byte-identical streams from the same seed."""

    def __init__(self, seed: int) -> None:
        self._s = seed

    def uniform(self) -> float:
        self._s = (16807 * self._s) % 2147483647
        return self._s / 2147483647.0


def _euclid(a: Cell, b: Cell) -> float:
    """Euclidean distance over integer cells — sqrt of an exact integer square,
    so the correctly-rounded IEEE sqrt returns identical bits in all languages."""
    return math.sqrt(float((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2))


class MaRrtStar(MultiAgentPlanner):
    @property
    def name(self) -> str:
        return "ma_rrt_star"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE}

    def plan(
        self,
        space: DiscreteSpace,
        tasks: list[AgentTask],
        recorder: TraceRecorder | None = None,
    ) -> MultiPlanResult:
        seed = self.params.get_int("seed")
        gamma = self.params.get_float("gamma")
        p_goal_prob = self.params.get_float("goal_sampling_probability")
        c_max = float(self.params.get_int("greedy_cost_budget"))
        max_iterations = self.params.get_int("max_iterations")

        goals = tuple(task.goal for task in tasks)
        start = tuple(task.start for task in tasks)
        k = len(tasks)

        # Two agents sharing a cell at t=0 is an unsolvable instance — no
        # conflict-free joint trajectory can ever separate them (the same honest
        # verdict the coupled searchers give).
        if any(start[i] == start[j] for i in range(k) for j in range(i + 1, k)):
            if recorder is not None:
                recorder.planning_finished(
                    False, {"expanded_nodes": 0.0, "makespan": 0.0, "sum_of_costs": 0.0}
                )
            return MultiPlanResult(False, [], 0.0, PlanStats(expanded_nodes=0))

        rng = _Lehmer(seed)
        cells = space.cells()  # the motion graph's vertex set W (row-major order)
        dim = 2 * k  # dimensionality d of the joint state space (one waypoint per agent)
        m_min = 1.0  # longest primitive: one cell at speed 1 in unit duration

        def dist_joint(a: tuple[Cell, ...], b: tuple[Cell, ...]) -> float:
            """Σ_i Euclidean(x_i, y_i) — the paper's joint distance (speed 1),
            summed in agent order so every language rounds identically."""
            total = 0.0
            for i in range(k):
                total += _euclid(a[i], b[i])
            return total

        def greedy(
            x: tuple[Cell, ...], target: tuple[Cell, ...]
        ) -> tuple[list[list[Cell]], float, bool]:
            """GREEDY(G_M, s, d) (paper Alg 4): all agents step simultaneously;
            each picks the FIRST-min child by Euclidean distance to its own
            coordinate of `target`, ties broken by the map's fixed neighbor order.
            A joint step that would repeat a cell (vertex conflict) or swap a pair
            (edge conflict) aborts: the segment ends at the pre-step snapshot. The
            loop runs while the joint state differs from `target` and the
            accumulated primitive cost still satisfies c ≤ c_max. Returns
            (per-agent segments incl. both endpoints, accumulated cost, reached).
            Termination: any step that changes some agent's cell costs ≥ 1 (a move
            is never free), so budget exhaustion bounds every call — EXCEPT a step
            where every agent's argmin picks wait, which changes no state at all.
            Such a step can never reach the target either, so the walk returns
            immediately there instead of burning budget on zero-progress waits; on
            any instance where progress is still possible the behavior is identical
            to the paper's loop."""
            seg: list[list[Cell]] = [[cell] for cell in x]
            cur = list(x)
            cost = 0.0
            while cur != list(target) and cost <= c_max:
                prev = list(cur)
                step_cost = 0.0
                nxt: list[Cell] = []
                for i in range(k):
                    best_child: Cell | None = None
                    best_d = math.inf
                    for child, _primitive in space.neighbors(cur[i]):
                        d = _euclid(child, target[i])
                        if d < best_d:  # first strict min = fixed-order tie-break
                            best_d = d
                            best_child = child
                    assert best_child is not None  # the wait self-loop always qualifies
                    nxt.append(best_child)
                # CollisionFree over the new step (earlier steps were checked when
                # they were added; separation is per-step, so this suffices).
                if any(
                    nxt[i] == nxt[j] or (nxt[i] == prev[j] and prev[i] == nxt[j])
                    for i in range(k)
                    for j in range(i + 1, k)
                ):
                    return seg, cost, False
                # A step where every agent's argmin picked wait changes no state;
                # it can never reach the target, so end the walk here (see above).
                if all(nxt[i] == prev[i] for i in range(k)):
                    return seg, cost, False
                for i in range(k):
                    # Primitive cost: 0 iff start = end = that agent's goal
                    # (wait-at-goal is free), else the unit duration 1.
                    if not (nxt[i] == prev[i] == goals[i]):
                        step_cost += 1.0
                    seg[i].append(nxt[i])
                cur = nxt
                cost += step_cost
            return seg, cost, tuple(cur) == tuple(target)

        def segment_cost(segment: list[list[Cell]]) -> float:
            """Σ over steps of Σ_i [0 if both endpoints are agent i's goal else 1]
            — the paper's cost(p): summed time outside the goal."""
            total = 0.0
            for t in range(1, len(segment[0])):
                for i, seg in enumerate(segment):
                    if not (seg[t] == seg[t - 1] == goals[i]):
                        total += 1.0
            return total

        def chain_cost(idx: int) -> float:
            """Summed primitive cost from the root to vertex `idx`. Segment costs
            are immutable once created, so walking the (shallow) parent chain and
            summing stored values is exact — rewiring changes which segments are
            on the chain, never what any segment costs."""
            total = 0.0
            node = idx
            while parents[node] != -1:
                total += seg_costs[node]
                node = parents[node]
            return total

        # The tree: parallel lists indexed by insertion order (insertion order IS
        # the NEAREST tie-break and the NEAR scan order). Root = start tuple; its
        # segment has no steps, so it contributes 0 to every chain cost.
        states: list[tuple[Cell, ...]] = [start]
        parents: list[int] = [-1]
        segments: list[list[list[Cell]]] = [[list([c]) for c in start]]
        seg_costs: list[float] = [0.0]
        index_of: dict[tuple[Cell, ...], int] = {start: 0}

        goal_index: int | None = 0 if start == goals else None
        best_cost = 0.0 if goal_index is not None else math.inf

        if recorder is not None:
            flat_start: list[int] = []
            for row, col in start:
                flat_start.extend([row, col])
            recorder.node_expanded(flat_start, 0.0)

        for _iteration in range(max_iterations):
            # --- SAMPLE (Alg 1 line 4 + §4.3/§4.4 goal biasing): one draw decides
            # the bias; unbiased samples draw one waypoint per agent, in order.
            if rng.uniform() < p_goal_prob:
                sample = goals
            else:
                sample = tuple(cells[int(rng.uniform() * len(cells))] for _ in range(k))

            # --- EXTEND (Alg 2). NEAREST over the whole tree; ties break to the
            # lowest insertion index.
            nearest_idx = 0
            nearest_d = dist_joint(states[0], sample)
            for idx in range(1, len(states)):
                d = dist_joint(states[idx], sample)
                if d < nearest_d:
                    nearest_d = d
                    nearest_idx = idx

            seg, seg_cost, _reached = greedy(states[nearest_idx], sample)
            # p_new = ∅ means GREEDY took no joint step at all — the sample IS the
            # nearest vertex, so the segment is just its single cell and new_state
            # below is already in the tree. A partial segment landing on an
            # existing vertex is the same case by outcome: the paper's set-union
            # line never says what a duplicate means for the parent edge; not
            # adding one is the only reading that keeps T a tree.
            new_state = tuple(seg[i][-1] for i in range(k))
            if len(seg[0]) == 1 or new_state in index_of:
                continue

            n = len(states)  # |V| of the OLD tree — what NEAR's radius formula reads
            r_n = max(math.floor(gamma * (math.log(n) / n) ** (1.0 / dim) * 64.0) / 64.0, m_min)

            # Parent selection: default is x_nearest (the vertex GREEDY extended
            # from); scan X_near in insertion order and take a STRICT improvement.
            best_parent = nearest_idx
            cost_new = chain_cost(nearest_idx) + seg_cost
            best_seg = seg
            best_seg_cost = seg_cost
            near_indices = [
                idx for idx in range(len(states)) if dist_joint(states[idx], new_state) <= r_n
            ]
            for idx in near_indices:
                cand_seg, cand_cost, cand_reached = greedy(states[idx], new_state)
                if not cand_reached:
                    continue  # x' ≠ x_new (Alg 2 line 10): not a candidate parent
                c_prime = chain_cost(idx) + cand_cost
                if c_prime < cost_new:
                    best_parent = idx
                    cost_new = c_prime
                    best_seg = cand_seg
                    best_seg_cost = cand_cost

            # §4.4 informed pruning: a vertex whose chain cost plus the metric
            # lower bound to the goal tuple exceeds the incumbent solution is dead
            # weight — do not add it, and do not rewire through it either.
            if cost_new + dist_joint(new_state, goals) > best_cost:
                continue

            new_idx = len(states)
            states.append(new_state)
            parents.append(best_parent)
            segments.append(best_seg)
            seg_costs.append(best_seg_cost)
            index_of[new_state] = new_idx
            if recorder is not None:
                flat_new: list[int] = []
                for row, col in new_state:
                    flat_new.extend([row, col])
                recorder.node_expanded(flat_new, cost_new)

            # Rewiring (Alg 2 lines 18–25): can any near vertex reach x_near better
            # THROUGH the new vertex? Strict improvement only; insertion-order scan.
            # Line 19's `GREEDY(G_M, x, x_near)` is read as GREEDY(x_new, x_near) —
            # reaching x_near through the new vertex is the whole point of rewiring.
            # No cycle can close: chain cost never decreases along a tree edge
            # (segment costs are ≥ 0), so if best_parent sat in idx's subtree we'd
            # have chain_cost(idx) ≤ chain_cost(best_parent) ≤ cost_new, and the
            # strict improvement test below could never fire for idx.
            for idx in near_indices:
                if idx == best_parent:
                    continue
                rew_seg, rew_cost, rew_reached = greedy(new_state, states[idx])
                if not rew_reached:
                    continue
                old_cost = chain_cost(idx)
                if old_cost > cost_new + rew_cost:
                    parents[idx] = new_idx
                    segments[idx] = rew_seg
                    seg_costs[idx] = rew_cost

            # The incumbent solution cost is whatever the (possibly rewired) goal
            # chain now costs — recomputed exactly, since rewiring changed chains.
            if goal_index is None and new_state == goals:
                goal_index = new_idx
            if goal_index is not None:
                best_cost = chain_cost(goal_index)

        # --- result -----------------------------------------------------------
        if goal_index is None:
            # Budget exhausted before the goal tuple ever became a vertex. An
            # honest "no solution found within budget" — MA-RRT* is only
            # probabilistically complete; this is not a verdict on the instance.
            if recorder is not None:
                recorder.planning_finished(
                    False,
                    {"expanded_nodes": float(len(states)), "makespan": 0.0, "sum_of_costs": 0.0},
                )
            return MultiPlanResult(False, [], 0.0, PlanStats(expanded_nodes=len(states)))

        # Chain root → goal node, concatenated per agent (junction cells appear in
        # both neighboring segments; skip each segment's duplicated head). Then
        # trim each agent's path after its LAST move: trailing steps are waits at
        # the agent's own goal costing 0, so trimming cannot change sum-of-costs.
        chain_indices: list[int] = []
        node = goal_index
        while node != -1:
            chain_indices.append(node)
            node = parents[node]
        chain_indices.reverse()
        paths: list[list[Cell]] = [[] for _ in range(k)]
        for node in chain_indices:
            seg = segments[node]
            for i in range(k):
                paths[i].extend(seg[i][1:] if paths[i] else seg[i])
        trimmed: list[list[Cell]] = []
        for path in paths:
            last_move = 0
            for t in range(1, len(path)):
                if path[t] != path[t - 1]:
                    last_move = t
            trimmed.append(path[: last_move + 1])

        cost = chain_cost(goal_index)
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
        return MultiPlanResult(True, trimmed, cost, PlanStats(expanded_nodes=len(states)))
