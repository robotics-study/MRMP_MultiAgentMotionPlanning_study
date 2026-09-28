"""sRRT — Wagner, Kang & Choset 2012 (ICRA); framework: Wagner & Choset (AIJ 2015).

The sampling branch's subdimensional pole. MA-RRT* fights dimension by growing
one tree over the full joint state; sRRT refuses the premise: plan for each robot
separately first (an INDIVIDUAL POLICY per robot), combine those paths into a
one-dimensional search space, and raise the local dimensionality only where robots
actually collide. The joint tree T_f still grows forward from q_I like an RRT, but
every node carries a collision set C_k: robots outside C_k obey their individual
policy (their coordinate is pinned to the policy's next step), robots inside C_k
are steered by samples toward sampled cells. A robot-robot collision on a local
path adds every involved robot to the expanding node's collision set and
back-propagates that addition up the predecessor chain until an ancestor already
lists the robot — so future expansions from any affected node move those robots
freely instead of obeying policies (the framework paper's words: "robots outside
the collision set will collide while obeying their individual policies… the search
space must include any possible joint action for the robots in the collision set").

Semantics fixed here (identical in C++/TS; paper section per decision):

* Individual policy (§II). The framework paper defines φᵢ as "the individually
  optimal path from each point in the free configuration space of a robot to its
  goal configuration, neglecting the presence of other robots"; the sRRT paper
  builds policies with RRTs only because "constructing such policies is infeasible
  when the dimensionality of the individual robot configuration space is large" —
  on this repo's DiscreteSpace it is not infeasible, so φᵢ IS the BFS tree grown
  backward from q_F^i (fixed neighbor order as parent tie-break; the goal's own
  policy step is a wait, the framework paper's "loop at the goal state"). The
  paper's on-demand extension ("if q ∉ Tᵢ extend until covered") is vacuous here:
  coverage is complete by construction — and if some start cell is NOT in agent i's
  policy tree, its goal is unreachable from its start at all, an instance verdict
  (no collision-free joint trajectory can exist), so planning fails immediately.
* Joint expansion (§III). NEAREST node q_r by Σ_i Manhattan (ties: lowest
  insertion index — insertion order IS the tie-break contract, as everywhere in
  this branch). The sample projects onto the local search space by formula (2):
  robots outside C_r have their coordinate replaced by φᵢ(q^i_r) — one policy
  step; robots inside keep the sampled cell. The local planner walks from q_r to
  the projection: a pinned robot's local path is its single policy edge then it
  waits (its local space is one-dimensional), a free robot greedy-steps toward
  its sampled cell (first strict minimum of Manhattan distance over the map's
  fixed neighbor order; at its target the wait self-loop is the unique minimum).
  All robots move in lockstep until every free robot has arrived. The walk needs
  no cost budget: a free robot's strict-min step strictly decreases its Manhattan
  distance to its target (or the walk gets stuck and honestly fails), so it ends
  in at most that distance steps; a pinned robot arrives after one step. Robot-
  obstacle collisions cannot occur by construction — every local path step moves
  over passable cells only, which is why no collision check here tests obstacles.
* Collision check on the local path: a vertex conflict (same cell, same step) or
  an edge swap (pair crosses one edge in opposite directions across a step). A
  conflict involving any robot NOT in C_r is informative — every involved robot
  joins C_r and back-propagates up the parent chain until an ancestor already
  lists it; the expansion aborts and the next sample re-expands with enlarged
  sets. A conflict between robots both already free carries no information: the
  expansion simply aborts, nothing updates. An accepted node inherits its
  parent's (post-update) collision set unchanged — sRRT never decouples a robot
  again on the branch where it was coupled; dimensionality drops only by growing
  a fresh branch off an ancestor whose set is smaller.
* A projected sample whose walk lands on an existing vertex is a no-op — the
  only reading that keeps T_f a tree, and why re-sampling the same region stops
  growing anything.
* Goal biasing (the repo's sampling convention, inherited from MA-RRT*'s §4.3/§4.4
  informed sampling): with probability p_goal the sample IS the goal tuple. The
  paper draws uniform samples only; on a grid, an un-biased joint goal tuple has
  probability ~|W|^-k per draw, so biasing is what makes the coupled regime reach
  the goal at finite budget (documented honestly on the page).
* Termination: the loop stops the moment the goal tuple becomes a vertex — sRRT
  has no optimality to refine (the paper explicitly leaves even probabilistic
  completeness unproven; this repo's honest verdict for an unsolvable instance
  within budget is "no solution found", never "unsolvable").

Determinism: the PRNG is the same MINSTD Lehmer generator as MA-RRT* (s ←
16807·s mod 2³¹−1, u = s/(2³¹−1) ∈ (0,1)) — integer-exact in Python and C++
int64 and exact in IEEE doubles everywhere, so all three engines draw identical
streams. Everything else is integer/grid arithmetic: no float appears anywhere in
this planner's decisions, which makes cross-language parity exact by construction.
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


def _manhattan(a: Cell, b: Cell) -> int:
    """Manhattan distance over integer cells — the same metric DiscreteSpace
    advertises; integer arithmetic, so identical in every language."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class Srrt(MultiAgentPlanner):
    @property
    def name(self) -> str:
        return "srrt"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE}

    def plan(
        self,
        space: DiscreteSpace,
        tasks: list[AgentTask],
        recorder: TraceRecorder | None = None,
    ) -> MultiPlanResult:
        seed = self.params.get_int("seed")
        p_goal_prob = self.params.get_float("goal_sampling_probability")
        max_iterations = self.params.get_int("max_iterations")

        goals = tuple(task.goal for task in tasks)
        start = tuple(task.start for task in tasks)
        k = len(tasks)

        # Two agents sharing a cell at t=0 is an unsolvable instance — no
        # conflict-free joint trajectory can ever separate them (the same honest
        # verdict every other planner here gives).
        if any(start[i] == start[j] for i in range(k) for j in range(i + 1, k)):
            if recorder is not None:
                recorder.planning_finished(
                    False, {"expanded_nodes": 0.0, "makespan": 0.0, "sum_of_costs": 0.0}
                )
            return MultiPlanResult(False, [], 0.0, PlanStats(expanded_nodes=0))

        rng = _Lehmer(seed)
        cells = space.cells()  # the motion graph's vertex set (row-major order)

        def neighbors(c: Cell) -> list[Cell]:
            """Passable 4-connected moves in fixed order (up/down/left/right),
            then the wait self-loop — identical contract to DiscreteSpace."""
            return [n for n, _cost in space.neighbors(c)]

        # --- individual policies (§II) ---------------------------------------
        # φᵢ = BFS tree grown backward from q_F^i over free cells. parent_i[c] is
        # the next step of the individually-optimal path from c to the goal; the
        # goal's own parent is itself (wait at one's own goal — the framework
        # paper's "loop at the goal state"). Fixed neighbor order = deterministic
        # BFS tie-break, identical in every language. A start cell missing from
        # its agent's policy tree means the goal is unreachable from that start
        # (BFS covered the whole component), so no joint solution exists — an
        # instance verdict, not a budget failure.
        policies: list[dict[Cell, Cell]] = []
        for i in range(k):
            tree: dict[Cell, Cell] = {goals[i]: goals[i]}
            frontier = [goals[i]]
            while frontier:
                nxt_frontier: list[Cell] = []
                for c in frontier:
                    for n in neighbors(c):
                        if n not in tree:  # fixed order ⇒ first parent wins (BFS tie-break)
                            tree[n] = c
                            nxt_frontier.append(n)
                frontier = nxt_frontier
            if start[i] not in tree:
                if recorder is not None:
                    recorder.planning_finished(
                        False, {"expanded_nodes": 0.0, "makespan": 0.0, "sum_of_costs": 0.0}
                    )
                return MultiPlanResult(False, [], 0.0, PlanStats(expanded_nodes=0))
            policies.append(tree)

        def dist_joint(a: tuple[Cell, ...], b: tuple[Cell, ...]) -> int:
            """Σ_i Manhattan — the joint distance NEAREST runs on, summed in
            agent order so every language sums identical integers."""
            total = 0
            for i in range(k):
                total += _manhattan(a[i], b[i])
            return total

        # --- joint tree T_f ---------------------------------------------------
        # Parallel lists indexed by insertion order (insertion order IS the
        # NEAREST tie-break). Root = start tuple, collision set ∅. Each node also
        # stores its local-path segment (per-agent cells, both endpoints included)
        # so the result chain can be concatenated without re-walking anything.
        states: list[tuple[Cell, ...]] = [start]
        parents_joint: list[int] = [-1]
        colsets: list[frozenset[int]] = [frozenset()]
        segments: list[list[list[Cell]]] = [[list([c]) for c in start]]
        index_of: dict[tuple[Cell, ...], int] = {start: 0}

        goal_index: int | None = 0 if start == goals else None

        if recorder is not None:
            flat_start: list[int] = []
            for row, col in start:
                flat_start.extend([row, col])
            recorder.node_expanded(flat_start)

        for _iteration in range(max_iterations):
            if goal_index is not None:
                # The tree already contains the goal tuple — stop growing (sRRT
                # has no anytime refinement to keep running for).
                break
            # --- SAMPLE: with probability p_goal the goal tuple itself (the repo's
            # informed-sampling convention); otherwise one waypoint per agent, in
            # agent order, uniform over the vertex set.
            if rng.uniform() < p_goal_prob:
                sample = goals
            else:
                sample = tuple(cells[int(rng.uniform() * len(cells))] for _ in range(k))

            # --- NEAREST over the whole tree; ties break to the lowest index.
            nearest_idx = 0
            nearest_d = dist_joint(states[0], sample)
            for idx in range(1, len(states)):
                d = dist_joint(states[idx], sample)
                if d < nearest_d:
                    nearest_d = d
                    nearest_idx = idx

            # --- LOCAL PLANNER (formula (2) + the lockstep walk). Pinned robots
            # take exactly their policy step; free robots greedy-step toward their
            # sampled cell and wait on arrival. A robot stuck at a Manhattan local
            # minimum of its target can never arrive — that expansion honestly
            # fails instead of looping forever.
            colset = colsets[nearest_idx]
            targets = tuple(
                policies[i][states[nearest_idx][i]] if i not in colset else sample[i]
                for i in range(k)
            )
            cur = states[nearest_idx]
            seg: list[list[Cell]] = [[c] for c in cur]  # per-agent cells, both endpoints
            aborted = False
            while any(cur[i] != targets[i] for i in range(k)):
                prev = cur
                nxt_list: list[Cell] = []
                stuck = False
                for i in range(k):
                    if cur[i] == targets[i]:
                        nxt_list.append(cur[i])  # arrived — the lockstep step is a wait
                        continue
                    best_child: Cell | None = None
                    best_d = math.inf
                    for child in neighbors(prev[i]):
                        d = _manhattan(child, targets[i])
                        if d < best_d:  # first strict min = fixed-order tie-break
                            best_d = d
                            best_child = child
                    assert best_child is not None  # the wait self-loop always qualifies
                    if best_child == prev[i]:
                        stuck = True  # local minimum: this target is unreachable here
                        break
                    nxt_list.append(best_child)
                if stuck:
                    aborted = True
                    break
                nxt = tuple(nxt_list)
                # Collision check on this step (vertex + swap), per agent pair.
                involved: set[int] = set()
                for i in range(k):
                    for j in range(i + 1, k):
                        if nxt[i] == nxt[j]:
                            involved.update((i, j))
                        elif nxt[i] == prev[j] and prev[i] == nxt[j]:
                            involved.update((i, j))
                if involved:
                    # ANY collision on the local path kills this expansion (the
                    # paper: a node is added only "if no collisions are found").
                    # An INFORMATIVE one (some involved robot not yet in C_r)
                    # additionally enlarges the set: every involved robot joins
                    # C_r and back-propagates up the chain until an ancestor
                    # already lists it (that ancestor's ancestors list it too by
                    # induction); the next sample re-expands with enlarged sets.
                    # A conflict between robots both already free carries no
                    # information: nothing updates, the expansion just fails.
                    if involved - colset:
                        new_set = colset | frozenset(involved)
                        colsets[nearest_idx] = new_set
                        colset = new_set
                        node = parents_joint[nearest_idx]
                        while node != -1 and not involved <= colsets[node]:
                            colsets[node] = colsets[node] | frozenset(involved)
                            node = parents_joint[node]
                    aborted = True
                    break
                for i in range(k):
                    seg[i].append(nxt[i])
                cur = nxt
            if aborted:
                continue

            new_state = cur
            if new_state in index_of:  # duplicate vertex — no-op (keeps T_f a tree)
                continue
            new_idx = len(states)
            states.append(new_state)
            parents_joint.append(nearest_idx)
            colsets.append(colset)  # the nearest node's post-update set, unchanged
            segments.append(seg)
            index_of[new_state] = new_idx
            if recorder is not None:
                flat_new: list[int] = []
                for row, col in new_state:
                    flat_new.extend([row, col])
                recorder.node_expanded(flat_new)

            # Goal check on the new vertex (per-coordinate equality — the joint
            # goal tuple is reached exactly).
            if new_state == goals:
                goal_index = new_idx

        # --- result -----------------------------------------------------------
        if goal_index is None:
            # Budget exhausted before the goal tuple ever became a vertex. An
            # honest "no solution found within budget" — sRRT's completeness is
            # unproven (the paper says so); this is not an instance verdict.
            if recorder is not None:
                recorder.planning_finished(
                    False,
                    {"expanded_nodes": float(len(states)), "makespan": 0.0, "sum_of_costs": 0.0},
                )
            return MultiPlanResult(False, [], 0.0, PlanStats(expanded_nodes=len(states)))

        # Chain root → goal node concatenated per agent (each segment's head is the
        # previous segment's tail — skip duplicated heads), then trim each agent's
        # path after its LAST move: trailing steps are waits at the agent's own
        # goal, so trimming cannot change any cost or any other agent's motion.
        chain_indices: list[int] = []
        node = goal_index
        while node != -1:
            chain_indices.append(node)
            node = parents_joint[node]
        chain_indices.reverse()
        paths: list[list[Cell]] = [[] for _ in range(k)]
        for node in chain_indices:
            seg = segments[node]
            for i in range(k):
                paths[i].extend(seg[i][1:] if paths[i] else seg[i])
        trimmed: list[list[Cell]] = []
        cost = 0.0
        for i, path in enumerate(paths):
            last_move = 0
            for t in range(1, len(path)):
                if path[t] != path[t - 1]:
                    last_move = t
            trimmed.append(path[: last_move + 1])
            # Per-agent cost: every action costs one time step except waiting at
            # one's own goal (the same metric every planner here reports).
            for t in range(1, len(trimmed[-1])):
                if not (trimmed[-1][t] == trimmed[-1][t - 1] == goals[i]):
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
        return MultiPlanResult(True, trimmed, cost, PlanStats(expanded_nodes=len(states)))
