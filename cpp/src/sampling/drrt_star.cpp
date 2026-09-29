// dRRT* — the C++ mirror of python/mrmp/sampling/drrt_star.py, line for line.
// The Python docstring carries the full paper mapping (Shome, Solovey, Dobson,
// Halperin & Bekris 2020); this file repeats only what a C++ reader needs.
// Cross-language bit-identity rests on: the int64-exact MINSTD Lehmer PRNG and
// one fixed operation order for every float expression (Euclidean distance with
// a fixed sum order, moving-pair distance, free/segment predicates, the radius
// bound) — -ffp-contract=off keeps FMA from fusing multiplies into one rounding.
#include "mrmp/sampling/drrt_star.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include "mrmp/core/geometry.hpp"

namespace mrmp::sampling {

using core::Point;
using State = std::vector<int>;

namespace {

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

// π as the exact double Python's math.pi carries — the nearest double to π is
// the same constant on both sides, so the radius bound lands on identical bits.
constexpr double kPi = 3.141592653589793238462643383279502884;

// Euclidean distance — fixed expression order (sqrt of the sum of squared
// differences; hypot would round differently).
double dist_pt(const Point& a, const Point& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

// H_i(v): shortest-path LENGTH on G_i from v to that robot's goal vertex (index
// 1), Dijkstra over the roadmap edges. The heap pops (distance, index)
// lexicographically — exactly Python's heapq tuple order; +inf when unreachable
// (an all-inf argmin resolves to the lowest index like every other tie here).
std::vector<double> dijkstra_h(const std::vector<Point>& vi, const std::vector<std::vector<int>>& adj_i) {
  const int n_i = static_cast<int>(vi.size());
  std::vector<double> dist_h(static_cast<size_t>(n_i), INFINITY);
  dist_h[1] = 0.0;
  std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>,
                      std::greater<std::pair<double, int>>>
      queue;
  queue.push({0.0, 1});
  while (!queue.empty()) {
    const auto [d_u, u] = queue.top();
    queue.pop();
    if (d_u > dist_h[static_cast<size_t>(u)]) continue;  // stale heap entry
    for (int w : adj_i[static_cast<size_t>(u)]) {
      const double w_len = dist_pt(vi[static_cast<size_t>(u)], vi[static_cast<size_t>(w)]);
      if (d_u + w_len < dist_h[static_cast<size_t>(w)]) {
        dist_h[static_cast<size_t>(w)] = d_u + w_len;
        queue.push({dist_h[static_cast<size_t>(w)], w});
      }
    }
  }
  return dist_h;
}

}  // namespace

core::ContinuousPlanResult DrrtStar::plan(const core::ContinuousSpace& space,
                                          const std::vector<core::ContinuousAgentTask>& tasks,
                                          core::TraceRecorder* recorder) {
  const int seed = params_.get_int("seed");
  const int n_samples = params_.get_int("samples_per_robot");
  const double eta = params_.get_float("eta");
  const double goal_sample_rate = params_.get_float("goal_sample_rate");
  const int max_iterations = params_.get_int("max_iterations");

  const size_t m = tasks.size();
  std::vector<double> radii;
  std::vector<Point> starts, goals;
  for (const auto& task : tasks) {
    radii.push_back(task.radius);
    starts.push_back(task.start);
    goals.push_back(task.goal);
  }

  // Instance verdicts (not budget): a disc overlapping an obstacle cell at its
  // start or goal; two starts overlapping; two goals overlapping. A start that
  // overlaps ANOTHER robot's goal is NOT a verdict — i can vacate.
  auto fail = [&](int expanded, core::TraceRecorder* rec) {
    if (rec != nullptr) {
      rec->planning_finished(false, {{"expanded_nodes", static_cast<double>(expanded)},
                                     {"makespan", 0.0}, {"sum_of_costs", 0.0}});
    }
    core::ContinuousPlanResult result;
    result.stats.expanded_nodes = expanded;
    return result;
  };

  for (size_t i = 0; i < m; ++i) {
    const bool start_free = space.free_point(starts[i], radii[i]);
    const bool goal_free = space.free_point(goals[i], radii[i]);
    if (!(start_free && goal_free)) return fail(0, recorder);
  }
  for (size_t i = 0; i < m; ++i) {
    for (size_t j = i + 1; j < m; ++j) {
      if (dist_pt(starts[i], starts[j]) < radii[i] + radii[j]) return fail(0, recorder);
      if (dist_pt(goals[i], goals[j]) < radii[i] + radii[j]) return fail(0, recorder);
    }
  }

  Lehmer rng{static_cast<std::int64_t>(seed)};
  const std::array<double, 4> extent = space.extent();
  const double x_min = extent[0], y_min = extent[1], x_max = extent[2], y_max = extent[3];

  // Theorem 1's radius at d = 2 (zeta_2 = pi), verbatim — fixed expression order,
  // identical bits in every engine.
  const double nf = static_cast<double>(n_samples);
  const double radius_r = ((1.0 + eta) * (1.0 + eta)) *
                          std::sqrt((space.area() * std::log(nf)) / (2.0 * kPi * nf));

  // --- individual roadmaps G_i (§4): [start, goal] first, n rejection samples
  // after; an edge exists iff dist < r(n) AND the swept-disc check from the
  // EARLIER-inserted endpoint passes. Adjacency keeps ascending index order and
  // INCLUDES the self-loop at own index — waiting is a graph edge here. ---
  std::vector<std::vector<Point>> vertices;
  std::vector<std::vector<std::vector<int>>> adjacency;
  for (size_t i = 0; i < m; ++i) {
    std::vector<Point> vi{starts[i], goals[i]};
    while (vi.size() < static_cast<size_t>(n_samples) + 2) {
      const double sx = x_min + rng.uniform() * (x_max - x_min);
      const double sy = y_min + rng.uniform() * (y_max - y_min);
      if (space.free_point(Point{sx, sy}, radii[i])) vi.push_back(Point{sx, sy});
    }
    std::set<std::pair<int, int>> edge_set;
    for (size_t v = 0; v < vi.size(); ++v) {
      for (size_t w = v + 1; w < vi.size(); ++w) {
        if (dist_pt(vi[v], vi[w]) < radius_r && space.segment_free(vi[v], vi[w], radii[i])) {
          edge_set.insert({static_cast<int>(v), static_cast<int>(w)});
        }
      }
    }
    std::vector<std::vector<int>> adj_i;
    for (size_t v = 0; v < vi.size(); ++v) {
      std::vector<int> nbrs;
      for (size_t w = 0; w < vi.size(); ++w) {
        const int lo = static_cast<int>(std::min(v, w));
        const int hi = static_cast<int>(std::max(v, w));
        if (w == v || edge_set.count({lo, hi})) nbrs.push_back(static_cast<int>(w));
      }
      adj_i.push_back(nbrs);
    }
    vertices.push_back(vi);
    adjacency.push_back(adj_i);
    if (recorder != nullptr) {
      const std::vector<std::pair<int, int>> edges(edge_set.begin(), edge_set.end());
      recorder->roadmap_built(static_cast<int>(i), vi, edges);
    }
  }

  // H tables over each G_i (precomputed once — the paper's implementation note).
  std::vector<std::vector<double>> h_table;
  for (size_t i = 0; i < m; ++i) h_table.push_back(dijkstra_h(vertices[i], adjacency[i]));

  const State root(m, 0);
  const State goal_tuple(m, 1);

  auto flatten = [&](const State& state) {
    std::vector<Point> pts;
    for (size_t i = 0; i < m; ++i) pts.push_back(vertices[i][static_cast<size_t>(state[i])]);
    return core::flatten_points(pts);
  };

  // Euclidean over concatenated coordinates — squared diffs summed in ascending
  // agent order (one sqrt), fixed like everything else.
  auto joint_distance = [&](const State& a, const std::vector<Point>& q) {
    double total = 0.0;
    for (size_t i = 0; i < m; ++i) {
      const Point& p = vertices[i][static_cast<size_t>(a[i])];
      const double dx = p.x - q[i].x;
      const double dy = p.y - q[i].y;
      total += dx * dx + dy * dy;
    }
    return std::sqrt(total);
  };

  // w(u, v) = sum_i ‖v_i − u_i‖ — ascending agent order, fixed.
  auto edge_weight = [&](const State& a, const State& b) {
    double total = 0.0;
    for (size_t i = 0; i < m; ++i) {
      total += dist_pt(vertices[i][static_cast<size_t>(a[i])], vertices[i][static_cast<size_t>(b[i])]);
    }
    return total;
  };

  // A tensor edge is valid iff no pair of discs strictly overlaps while everyone
  // moves (a motionless robot contributes its point against the mover's segment).
  auto edge_valid = [&](const State& a, const State& b) {
    for (size_t i = 0; i < m; ++i) {
      for (size_t j = i + 1; j < m; ++j) {
        const double d = core::moving_pair_distance(
            vertices[i][static_cast<size_t>(a[i])], vertices[i][static_cast<size_t>(b[i])],
            vertices[j][static_cast<size_t>(a[j])], vertices[j][static_cast<size_t>(b[j])]);
        if (d < radii[i] + radii[j]) return false;
      }
    }
    return true;
  };

  // Composite heuristic: the SUM of per-robot shortest-path lengths (the cost
  // function is a sum too). inf propagates; inf < inf is false.
  auto h_of = [&](const State& state) {
    double total = 0.0;
    for (size_t i = 0; i < m; ++i) total += h_table[i][static_cast<size_t>(state[i])];
    return total;
  };

  std::vector<State> states{root};
  std::vector<int> parents{-1};
  std::vector<double> costs{0.0};
  std::vector<std::vector<int>> children(1);
  std::map<State, int> index_of{{root, 0}};
  if (recorder != nullptr) recorder->node_expanded(flatten(root));

  // Re-parent node under new_parent (strictly cheaper by the caller's check) and
  // recompute the moved subtree's costs top-down. Children lists stay ascending,
  // so this BFS order is deterministic; cycles are structurally impossible (the
  // triangle inequality over per-agent lengths keeps every ancestor's cost at or
  // below any direct edge into it).
  auto rewire = [&](int node, int new_parent) {
    const int old = parents[static_cast<size_t>(node)];
    if (old >= 0) {
      auto& siblings_old = children[static_cast<size_t>(old)];
      siblings_old.erase(std::find(siblings_old.begin(), siblings_old.end(), node),
                         siblings_old.end());
    }
    parents[static_cast<size_t>(node)] = new_parent;
    std::vector<int>& siblings = children[static_cast<size_t>(new_parent)];
    size_t pos = 0;
    while (pos < siblings.size() && siblings[pos] < node) ++pos;
    siblings.insert(siblings.begin() + static_cast<long>(pos), node);
    costs[static_cast<size_t>(node)] =
        costs[static_cast<size_t>(new_parent)] +
        edge_weight(states[static_cast<size_t>(new_parent)], states[static_cast<size_t>(node)]);
    std::deque<int> queue{node};
    while (!queue.empty()) {
      const int x = queue.front();
      queue.pop_front();
      for (int ch : children[static_cast<size_t>(x)]) {
        costs[static_cast<size_t>(ch)] =
            costs[static_cast<size_t>(x)] +
            edge_weight(states[static_cast<size_t>(x)], states[static_cast<size_t>(ch)]);
        queue.push_back(ch);
      }
    }
  };

  double incumbent_cost = INFINITY;
  std::vector<std::vector<Point>> best_paths;
  bool have_solution = false;
  int v_last = 0;  // Algorithm 6 line 1: V_last <- S (greedy first); -1 is ∅.

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    std::vector<Point> q_rand;
    size_t near_idx;
    if (v_last < 0) {
      // Exploration: one bias draw decides; an unbiased sample draws the joint
      // sample per agent x-then-y, and each robot then draws its uniform neighbor
      // pick (agent order). A biased sample IS the goal tuple — every robot takes
      // the guided branch, no further draws.
      const double u_bias = rng.uniform();
      if (u_bias < goal_sample_rate) {
        q_rand = goals;
      } else {
        for (size_t i = 0; i < m; ++i) {
          const double sx = x_min + rng.uniform() * (x_max - x_min);
          const double sy = y_min + rng.uniform() * (y_max - y_min);
          q_rand.push_back(Point{sx, sy});
        }
      }
      near_idx = 0;
      double near_d = joint_distance(states[0], q_rand);
      for (size_t idx = 1; idx < states.size(); ++idx) {
        const double d = joint_distance(states[idx], q_rand);
        if (d < near_d) {  // strict < — earliest insertion wins ties
          near_d = d;
          near_idx = idx;
        }
      }
    } else {
      // Greedy child propagation (Alg. 7 lines 4-6): the sample IS the goal tuple
      // and V_near is V_last itself; every robot's q_i equals its goal exactly,
      // so I_d takes the guided branch for all of them.
      q_rand = goals;
      near_idx = static_cast<size_t>(v_last);
    }

    // I_d (Alg. 8), robots in ascending order: exact goal equality switches to
    // argmin H over Adj (strict < keeps the lowest index on ties; an all-inf
    // neighbourhood resolves to adj[0], its lowest-index entry too); otherwise a
    // uniform random neighbor pick. The picks are VERTEX INDICES.
    State v_new;
    for (size_t i = 0; i < m; ++i) {
      const std::vector<int>& adj = adjacency[i][static_cast<size_t>(states[near_idx][i])];
      if (q_rand[i] == goals[i]) {
        int best_idx = adj[0];
        double best_h = h_table[i][static_cast<size_t>(adj[0])];
        for (size_t w = 1; w < adj.size(); ++w) {
          const double h = h_table[i][static_cast<size_t>(adj[w])];
          if (h < best_h) {
            best_h = h;
            best_idx = adj[w];
          }
        }
        v_new.push_back(best_idx);
      } else {
        const int pick = static_cast<int>(rng.uniform() * static_cast<double>(adj.size()));
        v_new.push_back(adj[static_cast<size_t>(pick)]);
      }
    }

    // N = Adj(V_new, G_hat) ∩ T — scan tree nodes in ascending insertion order;
    // adjacency is per-robot and reflexive (self-loop at own index).
    std::vector<int> cand;
    for (size_t u_idx = 0; u_idx < states.size(); ++u_idx) {
      bool adjacent = true;
      for (size_t i = 0; i < m && adjacent; ++i) {
        const std::vector<int>& adj_i = adjacency[i][static_cast<size_t>(states[u_idx][i])];
        if (std::find(adj_i.begin(), adj_i.end(), v_new[i]) == adj_i.end()) adjacent = false;
      }
      if (adjacent) cand.push_back(static_cast<int>(u_idx));
    }

    // V_best = argmin over VALID candidates of c(U) + w(U, V_new); strict < so
    // the earliest-inserted candidate wins exact ties. No valid candidate → ∅.
    int best_parent = -1;
    double best_cost = INFINITY;
    for (int u_idx : cand) {
      const State& u_state = states[static_cast<size_t>(u_idx)];
      if (!edge_valid(u_state, v_new)) continue;
      const double c_cand = costs[static_cast<size_t>(u_idx)] + edge_weight(u_state, v_new);
      if (c_cand < best_cost) {  // strict < — earliest insertion wins ties
        best_cost = c_cand;
        best_parent = u_idx;
      }
    }
    if (best_parent < 0) {
      v_last = -1;
      continue;
    }
    if (best_cost > incumbent_cost) {
      // Branch-and-bound (strict): nothing this expansion could become beats the
      // incumbent — refuse it outright.
      v_last = -1;
      continue;
    }

    const bool inserted = index_of.find(v_new) == index_of.end();
    int idx_new;
    if (inserted) {
      idx_new = static_cast<int>(states.size());
      states.push_back(v_new);
      parents.push_back(best_parent);
      costs.push_back(best_cost);
      children.emplace_back();
      children[static_cast<size_t>(best_parent)].push_back(idx_new);  // ascending: largest index
      index_of[v_new] = idx_new;
      if (recorder != nullptr) recorder->node_expanded(flatten(v_new));
    } else {
      idx_new = index_of.find(v_new)->second;
      if (best_cost < costs[static_cast<size_t>(idx_new)]) rewire(idx_new, best_parent);
    }

    // Rewire pass (Alg. 7 lines 16-18): every tree neighbor V_new now owns gets
    // re-parented under V_new when that strictly improves its cost — and it can
    // never fire UPWARD (triangle inequality over per-agent lengths).
    for (int u_idx : cand) {
      if (u_idx == idx_new) continue;
      const State& u_state = states[static_cast<size_t>(u_idx)];
      if (edge_valid(v_new, u_state)) {
        const double w2 = edge_weight(v_new, u_state);
        if (costs[static_cast<size_t>(idx_new)] + w2 < costs[static_cast<size_t>(u_idx)]) {
          rewire(u_idx, idx_new);
        }
      }
    }

    // Child promotion (Alg. 7 line 19): a GENERATED node becomes the next V_last
    // iff it improved H over its chosen parent; anything else resets to ∅ — that
    // is what keeps the loop anytime after the goal tuple enters T.
    if (inserted && h_of(v_new) < h_of(states[static_cast<size_t>(best_parent)])) {
      v_last = idx_new;
    } else {
      v_last = -1;
    }

    // Connect to Target: the goal tuple is a tree node or the solution does not
    // exist yet; cost(π) is exactly c(goal node).
    const auto g_it = index_of.find(goal_tuple);
    if (g_it != index_of.end() && costs[static_cast<size_t>(g_it->second)] < incumbent_cost) {
      std::vector<int> chain;
      int node = g_it->second;
      while (node != -1) {
        chain.push_back(node);
        node = parents[static_cast<size_t>(node)];
      }
      std::reverse(chain.begin(), chain.end());
      // chain holds NODE indices root → goal node; agent i's waypoint at step t
      // is the vertex its index tuple selects at that node.
      std::vector<std::vector<Point>> raw_paths(m);
      for (int node_idx : chain) {
        const State& state = states[static_cast<size_t>(node_idx)];
        for (size_t i = 0; i < m; ++i) raw_paths[i].push_back(vertices[i][static_cast<size_t>(state[i])]);
      }
      // Trim each agent's waypoint list after its LAST move (trailing waits add
      // zero arc length; replay clamps past the end anyway).
      std::vector<std::vector<Point>> trimmed;
      for (size_t i = 0; i < m; ++i) {
        const std::vector<Point>& points = raw_paths[i];
        size_t last_move = 0;
        for (size_t t = 1; t < points.size(); ++t) {
          if (!(points[t] == points[t - 1])) last_move = t;
        }
        trimmed.push_back(std::vector<Point>(points.begin(),
                                             points.begin() + static_cast<long>(last_move) + 1));
      }
      incumbent_cost = costs[static_cast<size_t>(g_it->second)];
      best_paths = trimmed;
      have_solution = true;
    }
  }

  if (!have_solution) return fail(static_cast<int>(states.size()), recorder);

  // Geometric metrics (the paper's own cost functions): sum and max over the
  // per-agent arc lengths, recomputed from the traced paths in fixed order. (c
  // (goal node) drove the search with the same quantities grouped edge-first;
  // float associativity makes the two groupings differ in low bits — both are
  // pinned, neither is "more true" than the other.)
  std::vector<double> lengths;
  for (const auto& path : best_paths) {
    double length = 0.0;
    for (size_t t = 1; t < path.size(); ++t) length += dist_pt(path[t - 1], path[t]);
    lengths.push_back(length);
  }
  double sum_of_costs = 0.0;
  double makespan = 0.0;
  for (size_t i = 0; i < m; ++i) {
    sum_of_costs += lengths[i];
    if (lengths[i] > makespan) makespan = lengths[i];
  }

  if (recorder != nullptr) {
    for (size_t i = 0; i < m; ++i) recorder->path_found(best_paths[i], static_cast<int>(i));
    recorder->planning_finished(true, {{"expanded_nodes", static_cast<double>(states.size())},
                                       {"makespan", makespan}, {"sum_of_costs", sum_of_costs}});
  }
  core::ContinuousPlanResult result;
  result.success = true;
  result.paths = best_paths;
  result.cost = sum_of_costs;
  result.makespan = makespan;
  result.stats.expanded_nodes = static_cast<int>(states.size());
  return result;
}

}  // namespace mrmp::sampling
