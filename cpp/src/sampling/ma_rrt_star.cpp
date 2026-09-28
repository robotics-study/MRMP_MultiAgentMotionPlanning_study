// MA-RRT* — the C++ mirror of python/mrmp/sampling/ma_rrt_star.py, line for
// line. The Python docstrings carry the full paper mapping (Čáp et al. 2013,
// Algorithms 1–4 + §4.4 optimizations); this file repeats only what a C++
// reader needs. Cross-language bit-identity rests on: int64-exact MINSTD Lehmer
// PRNG, correctly-rounded sqrt over integer deltas, IEEE addition in agent
// order, and the 1/64-lattice quantization of r_n (log/pow are libm-dependent
// at the ULP level — quantizing makes membership flips astronomically unlikely).
#include "mrmp/sampling/ma_rrt_star.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>

namespace mrmp::sampling {

using core::Cell;
using core::DiscreteSpace;

namespace {

// The joint state: one Cell per agent, ordered by agent index (std::vector's
// lexicographic operator< via Cell::operator< keys the ordered index map).
using State = std::vector<Cell>;

// MINSTD Lehmer PRNG: s ← 16807·s mod (2³¹−1), u = s/(2³¹−1) ∈ (0,1). The
// multiply fits int64 exactly; the division is the same IEEE double in every
// language, so all three engines draw byte-identical streams from one seed.
struct Lehmer {
  std::int64_t s;
  double uniform() {
    s = (16807 * s) % 2147483647;
    return static_cast<double>(s) / 2147483647.0;
  }
};

// Euclidean distance over integer cells — sqrt of an exact integer square, so
// the correctly-rounded IEEE sqrt returns identical bits in all languages.
double euclid(const Cell& a, const Cell& b) {
  const long long dr = static_cast<long long>(a.row) - b.row;
  const long long dc = static_cast<long long>(a.col) - b.col;
  return std::sqrt(static_cast<double>(dr * dr + dc * dc));
}

}  // namespace

core::MultiPlanResult MaRrtStar::plan(const DiscreteSpace& space,
                                      const std::vector<core::AgentTask>& tasks,
                                      core::TraceRecorder* recorder) {
  const int seed = params_.get_int("seed");
  const double gamma = params_.get_float("gamma");
  const double p_goal_prob = params_.get_float("goal_sampling_probability");
  const double c_max = static_cast<double>(params_.get_int("greedy_cost_budget"));
  const int max_iterations = params_.get_int("max_iterations");

  State goals, start;
  for (const auto& task : tasks) {
    start.push_back(task.start);
    goals.push_back(task.goal);
  }
  const size_t k = tasks.size();

  // Two agents sharing a cell at t=0 is an unsolvable instance — no
  // conflict-free joint trajectory can ever separate them (the same honest
  // verdict the coupled searchers give).
  for (size_t i = 0; i < k; ++i) {
    for (size_t j = i + 1; j < k; ++j) {
      if (start[i] == start[j]) {
        if (recorder != nullptr) {
          recorder->planning_finished(false, {{"expanded_nodes", 0.0}, {"makespan", 0.0},
                                              {"sum_of_costs", 0.0}});
        }
        core::MultiPlanResult result;
        result.stats.expanded_nodes = 0;
        return result;
      }
    }
  }

  Lehmer rng{static_cast<std::int64_t>(seed)};
  const std::vector<Cell> cells = space.cells();  // the motion graph's vertex set W
  const double dim = static_cast<double>(2 * k);  // joint-space dimensionality
  const double m_min = 1.0;  // longest primitive: one cell at speed 1 in unit duration

  // Σ_i Euclidean(x_i, y_i) — the paper's joint distance (speed 1), summed in
  // agent order so every language rounds identically.
  auto dist_joint = [&](const State& a, const State& b) {
    double total = 0.0;
    for (size_t i = 0; i < k; ++i) total += euclid(a[i], b[i]);
    return total;
  };

  struct GreedyResult {
    std::vector<std::vector<Cell>> seg;  // per-agent cell sequences incl. both endpoints
    double cost;
    bool reached;
  };

  // GREEDY(G_M, s, d) (paper Alg 4): all agents step simultaneously; each picks
  // the FIRST-min child by Euclidean distance to its own coordinate of `target`,
  // ties broken by the map's fixed neighbor order. A joint step that would
  // repeat a cell (vertex conflict) or swap a pair (edge conflict) aborts: the
  // segment ends at the pre-step snapshot. The loop runs while the joint state
  // differs from `target` and the accumulated primitive cost satisfies c ≤ c_max.
  // Termination: any step that moves some agent costs ≥ 1, so budget exhaustion
  // bounds every call — EXCEPT a step where every agent's argmin picks wait,
  // which changes no state; it can never reach the target either, so the walk
  // returns immediately there instead of burning budget on zero-progress waits.
  auto greedy = [&](const State& x, const State& target) {
    GreedyResult r{std::vector<std::vector<Cell>>(k), 0.0, false};
    for (size_t i = 0; i < k; ++i) r.seg[i].push_back(x[i]);
    State cur = x;
    while (!(cur == target) && r.cost <= c_max) {
      const State prev = cur;
      double step_cost = 0.0;
      std::vector<Cell> nxt;
      for (size_t i = 0; i < k; ++i) {
        Cell best_child{0, 0};
        double best_d = std::numeric_limits<double>::infinity();
        for (const auto& [child, primitive_cost] : space.neighbors(cur[i])) {
          (void)primitive_cost;
          const double d = euclid(child, target[i]);
          if (d < best_d) {  // first strict min = fixed-order tie-break
            best_d = d;
            best_child = child;
          }
        }
        // The wait self-loop is always a candidate, so best_child is always set.
        nxt.push_back(best_child);
      }
      // CollisionFree over the new step (earlier steps were checked when they
      // were added; separation is per-step, so this suffices).
      bool conflict = false;
      for (size_t i = 0; i < k && !conflict; ++i) {
        for (size_t j = i + 1; j < k; ++j) {
          if (nxt[i] == nxt[j] || (nxt[i] == prev[j] && prev[i] == nxt[j])) {
            conflict = true;
            break;
          }
        }
      }
      if (conflict) return r;
      // A step where every agent's argmin picked wait changes no state.
      bool all_wait = true;
      for (size_t i = 0; i < k && all_wait; ++i) all_wait = nxt[i] == prev[i];
      if (all_wait) return r;
      for (size_t i = 0; i < k; ++i) {
        // Primitive cost: 0 iff start = end = that agent's goal (wait-at-goal
        // is free), else the unit duration 1.
        if (!(nxt[i] == prev[i] && nxt[i] == goals[i])) step_cost += 1.0;
        r.seg[i].push_back(nxt[i]);
      }
      cur = nxt;
      r.cost += step_cost;
    }
    r.reached = (cur == target);
    return r;
  };

  // Σ over steps of Σ_i [0 if both endpoints are agent i's goal else 1] — the
  // paper's cost(p): summed time outside the goal.
  auto segment_cost = [&](const std::vector<std::vector<Cell>>& seg) {
    double total = 0.0;
    for (size_t t = 1; t < seg[0].size(); ++t) {
      for (size_t i = 0; i < k; ++i) {
        if (!(seg[i][t] == seg[i][t - 1] && seg[i][t] == goals[i])) total += 1.0;
      }
    }
    return total;
  };

  // The tree: parallel vectors indexed by insertion order (insertion order IS
  // the NEAREST tie-break and the NEAR scan order). Root = start tuple; its
  // segment has no steps, so it contributes 0 to every chain cost. Segment costs
  // are immutable once created, so walking the (shallow) parent chain and summing
  // stored values is exact — rewiring changes which segments are on the chain,
  // never what any segment costs.
  std::vector<State> states{start};
  std::vector<int> parents{-1};
  std::vector<std::vector<Cell>> root_seg;
  for (const Cell& c : start) root_seg.push_back(std::vector<Cell>{c});
  std::vector<std::vector<std::vector<Cell>>> segments{root_seg};
  std::vector<double> seg_costs{0.0};
  std::map<State, int> index_of{{start, 0}};

  auto chain_cost = [&](int idx) {
    double total = 0.0;
    int node = idx;
    while (parents[static_cast<size_t>(node)] != -1) {
      total += seg_costs[static_cast<size_t>(node)];
      node = parents[static_cast<size_t>(node)];
    }
    return total;
  };

  bool goal_found = start == goals;
  int goal_index = goal_found ? 0 : -1;
  double best_cost = goal_found ? 0.0 : std::numeric_limits<double>::infinity();

  if (recorder != nullptr) {
    recorder->node_expanded(core::flatten(start), 0.0);
  }

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    // --- SAMPLE (Alg 1 line 4 + §4.3/§4.4 goal biasing): one draw decides the
    // bias; unbiased samples draw one waypoint per agent, in order.
    State sample;
    if (rng.uniform() < p_goal_prob) {
      sample = goals;
    } else {
      for (size_t i = 0; i < k; ++i) {
        sample.push_back(cells[static_cast<size_t>(rng.uniform() * static_cast<double>(
            cells.size()))]);
      }
    }

    // --- EXTEND (Alg 2). NEAREST over the whole tree; ties break to the lowest
    // insertion index.
    size_t nearest_idx = 0;
    double nearest_d = dist_joint(states[0], sample);
    for (size_t idx = 1; idx < states.size(); ++idx) {
      const double d = dist_joint(states[idx], sample);
      if (d < nearest_d) {
        nearest_d = d;
        nearest_idx = idx;
      }
    }

    // p_new = ∅ means GREEDY took no joint step at all — the sample IS the
    // nearest vertex, so the segment is just its single cell and new_state below
    // is already in the tree. A partial segment landing on an existing vertex is
    // the same case by outcome: the paper's set-union line never says what a
    // duplicate means for the parent edge; not adding one is the only reading
    // that keeps T a tree.
    GreedyResult ext = greedy(states[nearest_idx], sample);
    State new_state;
    for (size_t i = 0; i < k; ++i) new_state.push_back(ext.seg[i].back());
    if (ext.seg[0].size() == 1 || index_of.count(new_state)) continue;

    const double n = static_cast<double>(states.size());  // |V| of the OLD tree
    const double r_n = std::max(
        std::floor(((gamma * std::pow(std::log(n) / n, 1.0 / dim)) * 64.0)) / 64.0, m_min);

    // Parent selection: default is x_nearest (the vertex GREEDY extended from);
    // scan X_near in insertion order and take a STRICT improvement.
    size_t best_parent = nearest_idx;
    double cost_new = chain_cost(static_cast<int>(nearest_idx)) + ext.cost;
    std::vector<std::vector<Cell>> best_seg = std::move(ext.seg);
    double best_seg_cost = ext.cost;
    std::vector<size_t> near_indices;
    for (size_t idx = 0; idx < states.size(); ++idx) {
      if (dist_joint(states[idx], new_state) <= r_n) near_indices.push_back(idx);
    }
    for (size_t idx : near_indices) {
      GreedyResult cand = greedy(states[idx], new_state);
      if (!cand.reached) continue;  // x' ≠ x_new (Alg 2 line 10): not a candidate
      const double c_prime = chain_cost(static_cast<int>(idx)) + cand.cost;
      if (c_prime < cost_new) {
        best_parent = idx;
        cost_new = c_prime;
        best_seg = std::move(cand.seg);
        best_seg_cost = cand.cost;
      }
    }

    // §4.4 informed pruning: a vertex whose chain cost plus the metric lower
    // bound to the goal tuple exceeds the incumbent solution is dead weight — do
    // not add it, and do not rewire through it either.
    if (cost_new + dist_joint(new_state, goals) > best_cost) continue;

    const int new_idx = static_cast<int>(states.size());
    states.push_back(std::move(new_state));  // NOLINT — new_state read below via states[new_idx]
    parents.push_back(static_cast<int>(best_parent));
    segments.push_back(std::move(best_seg));
    seg_costs.push_back(best_seg_cost);
    index_of[states[static_cast<size_t>(new_idx)]] = new_idx;
    if (recorder != nullptr) recorder->node_expanded(core::flatten(states[new_idx]), cost_new);

    // Rewiring (Alg 2 lines 18–25): can any near vertex reach x_near better
    // THROUGH the new vertex? Strict improvement only; insertion-order scan. Line
    // 19's `GREEDY(G_M, x, x_near)` is read as GREEDY(x_new, x_near) — reaching
    // x_near through the new vertex is the whole point of rewiring. No cycle can
    // close: chain cost never decreases along a tree edge (segment costs are ≥ 0),
    // so if best_parent sat in idx's subtree we'd have chain_cost(idx) ≤
    // chain_cost(best_parent) ≤ cost_new, and the strict test below could never
    // fire for idx.
    for (size_t idx : near_indices) {
      if (idx == best_parent) continue;
      GreedyResult rew = greedy(states[static_cast<size_t>(new_idx)], states[idx]);
      if (!rew.reached) continue;
      const double old_cost = chain_cost(static_cast<int>(idx));
      if (old_cost > cost_new + rew.cost) {
        parents[idx] = new_idx;
        segments[idx] = std::move(rew.seg);
        seg_costs[idx] = rew.cost;
      }
    }

    // The incumbent solution cost is whatever the (possibly rewired) goal chain
    // now costs — recomputed exactly, since rewiring changed chains.
    if (!goal_found && states[static_cast<size_t>(new_idx)] == goals) {
      goal_found = true;
      goal_index = new_idx;
    }
    if (goal_found) best_cost = chain_cost(goal_index);
  }

  // --- result -----------------------------------------------------------
  if (!goal_found) {
    // Budget exhausted before the goal tuple ever became a vertex. An honest
    // "no solution found within budget" — MA-RRT* is only probabilistically
    // complete; this is not a verdict on the instance.
    if (recorder != nullptr) {
      recorder->planning_finished(false, {{"expanded_nodes", static_cast<double>(states.size())},
                                          {"makespan", 0.0},
                                          {"sum_of_costs", 0.0}});
    }
    core::MultiPlanResult result;
    result.stats.expanded_nodes = static_cast<int>(states.size());
    return result;
  }

  // Chain root → goal node, concatenated per agent (junction cells appear in
  // both neighboring segments; skip each segment's duplicated head). Then trim
  // each agent's path after its LAST move: trailing steps are waits at the
  // agent's own goal costing 0, so trimming cannot change sum-of-costs.
  std::vector<int> chain_indices;
  for (int node = goal_index; node != -1; node = parents[static_cast<size_t>(node)]) {
    chain_indices.push_back(node);
  }
  std::reverse(chain_indices.begin(), chain_indices.end());
  std::vector<std::vector<Cell>> paths(k);
  for (int node : chain_indices) {
    const auto& seg = segments[static_cast<size_t>(node)];
    for (size_t i = 0; i < k; ++i) {
      const auto& cells_of_agent = seg[i];
      // The segment includes both endpoints; the head duplicates what the
      // previous segment already ended on.
      size_t from = paths[i].empty() ? 0 : 1;
      for (size_t t = from; t < cells_of_agent.size(); ++t) paths[i].push_back(cells_of_agent[t]);
    }
  }
  std::vector<std::vector<Cell>> trimmed(k);
  for (size_t i = 0; i < k; ++i) {
    size_t last_move = 0;
    for (size_t t = 1; t < paths[i].size(); ++t) {
      if (!(paths[i][t] == paths[i][t - 1])) last_move = t;
    }
    trimmed[i].assign(paths[i].begin(), paths[i].begin() + static_cast<long>(last_move) + 1);
  }

  const double cost = chain_cost(goal_index);
  int makespan = 0;
  for (const auto& p : trimmed) {
    makespan = std::max(makespan, static_cast<int>(p.size()) - 1);
  }
  if (recorder != nullptr) {
    for (size_t i = 0; i < trimmed.size(); ++i) {
      recorder->path_found(trimmed[i], static_cast<int>(i));
    }
    recorder->planning_finished(true, {{"expanded_nodes", static_cast<double>(states.size())},
                                       {"makespan", static_cast<double>(makespan)},
                                       {"sum_of_costs", cost}});
  }
  core::MultiPlanResult result;
  result.success = true;
  result.paths = std::move(trimmed);
  result.cost = cost;
  result.stats.expanded_nodes = static_cast<int>(states.size());
  return result;
}

}  // namespace mrmp::sampling
