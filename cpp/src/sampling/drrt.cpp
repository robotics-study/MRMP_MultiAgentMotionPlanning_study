// dRRT — the C++ mirror of python/mrmp/sampling/drrt.py, line for line. The
// Python docstring carries the full paper mapping (Solovey, Salzman & Halperin
// 2016); this file repeats only what a C++ reader needs. Cross-language
// bit-identity rests on: the int64-exact MINSTD Lehmer PRNG and one fixed
// operation order for every float expression (cosine oracle, Euclidean distance
// with a fixed sum order, moving-pair distance, free/segment predicates) —
// -ffp-contract=off keeps FMA from fusing multiplies into one rounding.
#include "mrmp/sampling/drrt.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <utility>

#include "mrmp/core/geometry.hpp"

namespace mrmp::sampling {

using core::Point;

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

// Euclidean distance — fixed expression order (sqrt of the sum of squared
// differences; hypot would round differently).
double dist_pt(const Point& a, const Point& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

// The oracle's angle ranking: the cosine of the angle between rays ρ(c,q) and
// ρ(c,v). Fixed expression order; argmax over it == argmin over the angle. A
// degenerate ray (endpoints at identical points — only possible when a robot's
// start EQUALS its goal) has no angle: cosine is fixed to 0.0, which under
// strict > leaves the ascending-list order as the tie-break.
double cosine_ray(const Point& q, const Point& c, const Point& v) {
  const double dx1 = q.x - c.x;
  const double dy1 = q.y - c.y;
  const double dx2 = v.x - c.x;
  const double dy2 = v.y - c.y;
  const double denom = std::sqrt(dx1 * dx1 + dy1 * dy1) * std::sqrt(dx2 * dx2 + dy2 * dy2);
  if (denom == 0.0) return 0.0;
  return (dx1 * dx2 + dy1 * dy2) / denom;
}

}  // namespace

core::ContinuousPlanResult Drrt::plan(const core::ContinuousSpace& space,
                                      const std::vector<core::ContinuousAgentTask>& tasks,
                                      core::TraceRecorder* recorder) {
  const int seed = params_.get_int("seed");
  const int samples_per_robot = params_.get_int("samples_per_robot");
  const int k_fanout = params_.get_int("roadmap_k");
  const int max_rounds = params_.get_int("max_rounds");

  const size_t m = tasks.size();
  std::vector<double> radii;
  std::vector<Point> starts, goals;
  for (const auto& task : tasks) {
    radii.push_back(task.radius);
    starts.push_back(task.start);
    goals.push_back(task.goal);
  }

  // Instance verdicts (not budget): a disc overlapping an obstacle cell at its
  // start or goal makes the instance unsolvable outright — and so do two
  // overlapping start discs: no valid initial configuration exists at all.
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
    }
  }

  Lehmer rng{static_cast<std::int64_t>(seed)};
  const std::array<double, 4> extent = space.extent();
  const double x_min = extent[0], y_min = extent[1], x_max = extent[2], y_max = extent[3];

  // --- individual roadmaps G_i (§2) --------------------------------------------
  // Vertices: [start, goal] then n rejection samples in draw order (one shared
  // PRNG stream: agent 0's samples first). Edges: each vertex links to its k
  // nearest by Euclidean distance (ties: lower insertion index); an edge exists
  // iff the swept disc of the straight segment stays clear — evaluated exactly
  // once per pair, from the EARLIER-inserted endpoint.
  std::vector<std::vector<Point>> vertices;
  std::vector<std::vector<std::vector<int>>> adjacency;
  for (size_t i = 0; i < m; ++i) {
    std::vector<Point> vi{starts[i], goals[i]};
    while (vi.size() < static_cast<size_t>(samples_per_robot) + 2) {
      const double sx = x_min + rng.uniform() * (x_max - x_min);
      const double sy = y_min + rng.uniform() * (y_max - y_min);
      if (space.free_point(Point{sx, sy}, radii[i])) vi.push_back(Point{sx, sy});
    }
    std::set<std::pair<int, int>> edge_set;
    for (size_t v = 0; v < vi.size(); ++v) {
      // Candidates sorted by (distance ascending, index ascending); the first k
      // are candidates. A pair already accepted is skipped; a pair rejected
      // earlier re-checks to the same verdict because the segment is always
      // evaluated from the EARLIER-inserted endpoint.
      std::vector<std::pair<double, int>> cands;
      for (size_t w = 0; w < vi.size(); ++w) {
        if (w == v) continue;
        cands.emplace_back(dist_pt(vi[v], vi[w]), static_cast<int>(w));
      }
      std::sort(cands.begin(), cands.end(), [](const auto& a, const auto& b) {
        return a.first != b.first ? a.first < b.first : a.second < b.second;
      });
      for (size_t k = 0; k < static_cast<size_t>(k_fanout) && k < cands.size(); ++k) {
        const int w = cands[k].second;
        const std::pair<int, int> pair = (v < static_cast<size_t>(w))
                                             ? std::pair<int, int>{static_cast<int>(v), w}
                                             : std::pair<int, int>{w, static_cast<int>(v)};
        if (edge_set.count(pair)) continue;
        if (space.segment_free(vi[pair.first], vi[pair.second], radii[i])) {
          edge_set.insert(pair);
        }
      }
    }
    std::vector<std::vector<int>> adj_i(vi.size());
    for (const auto& [a, b] : edge_set) {  // lexicographic order — the trace's edge order too
      adj_i[a].push_back(b);
      adj_i[b].push_back(a);
    }
    vertices.push_back(vi);
    adjacency.push_back(adj_i);
    if (recorder != nullptr) {
      const std::vector<std::pair<int, int>> edges(edge_set.begin(), edge_set.end());
      recorder->roadmap_built(static_cast<int>(i), vi, edges);
    }
  }

  // --- the implicit composite roadmap's tree T (§3.2) ---------------------------
  // Joint vertices are tuples of per-agent vertex indices; every agent's start is
  // index 0 and goal index 1 of its own roadmap. The "goal tuple" is not a fixed
  // index tuple here — it is the joint POINTS below: a tree node whose
  // coordinates equal them sits at distance 0 and ranks first.
  using State = std::vector<int>;
  const State root(m, 0);
  std::vector<Point> goal_points;
  for (size_t i = 0; i < m; ++i) goal_points.push_back(vertices[i][1]);

  // Euclidean distance over concatenated coordinates — squared diffs summed in
  // fixed order (agent 0 x,y then agent 1 x,y, …), one sqrt.
  auto joint_distance = [&](const State& a, const std::vector<Point>& q) {
    double total = 0.0;
    for (size_t i = 0; i < m; ++i) {
      const Point& p = vertices[i][a[i]];
      const double dx = p.x - q[i].x;
      const double dy = p.y - q[i].y;
      total += dx * dx + dy * dy;
    }
    return std::sqrt(total);
  };

  auto flatten = [&](const State& state) {
    std::vector<Point> pts;
    for (size_t i = 0; i < m; ++i) pts.push_back(vertices[i][state[i]]);
    return core::flatten_points(pts);
  };

  std::vector<State> states{root};
  std::vector<int> parents_joint{-1};
  std::map<State, int> index_of{{root, 0}};
  if (recorder != nullptr) recorder->node_expanded(flatten(root));

  // O_D (§3.1/§4.1): per robot, the adjacency neighbor whose ray from the
  // current vertex makes the smallest angle to the sample's point — argmax
  // cosine over the ascending-ordered list (strict > keeps the lower index on
  // ties). Any robot without neighbors → ∅ → the sample is ignored.
  auto oracle = [&](const State& state, const std::vector<Point>& q, State* out) -> bool {
    State candidate;
    for (size_t i = 0; i < m; ++i) {
      const int c_idx = state[i];
      const std::vector<int>& nbrs = adjacency[i][c_idx];
      if (nbrs.empty()) return false;
      const Point c_point = vertices[i][c_idx];
      const Point q_point = q[i];
      int best_idx = -1;
      double best_cos = 0.0;
      bool first = true;
      for (int w : nbrs) {
        const double cos = cosine_ray(q_point, c_point, vertices[i][w]);
        if (first || cos > best_cos) {
          best_cos = cos;
          best_idx = w;
          first = false;
        }
      }
      candidate.push_back(best_idx);
    }
    *out = candidate;
    return true;
  };

  // Tensor edge: everyone moves at once; valid iff no pair of discs strictly
  // overlaps anywhere along the simultaneous motion (the relative-motion segment
  // covers both endpoints too).
  auto edge_valid = [&](const State& prev, const State& nxt) {
    for (size_t i = 0; i < m; ++i) {
      for (size_t j = i + 1; j < m; ++j) {
        const double d = core::moving_pair_distance(vertices[i][prev[i]], vertices[i][nxt[i]],
                                                    vertices[j][prev[j]], vertices[j][nxt[j]]);
        if (d < radii[i] + radii[j]) return false;
      }
    }
    return true;
  };

  // Hop-shortest path on G_agent from the node's vertex to its goal (vertex 1);
  // fixed ascending neighbor order, first discovery wins. The parent chain is
  // walked BACKWARD from the goal and reversed — the returned point path STARTS
  // at the node's vertex and ENDS at the goal.
  auto bfs_local = [&](int start_idx, size_t agent, std::vector<Point>* out) -> bool {
    std::map<int, int> tree{{start_idx, start_idx}};
    std::vector<int> frontier{start_idx};
    while (!tree.count(1)) {
      std::vector<int> nxt_frontier;
      for (int c : frontier) {
        for (int w : adjacency[agent][c]) {
          if (!tree.count(w)) {
            tree[w] = c;
            nxt_frontier.push_back(w);
          }
        }
      }
      if (nxt_frontier.empty()) return false;  // goal vertex unreachable on this roadmap
      frontier = nxt_frontier;
    }
    std::vector<int> chain;
    int node = 1;
    while (true) {
      chain.push_back(node);
      if (tree[node] == node) break;
      node = tree[node];
    }
    std::reverse(chain.begin(), chain.end());  // goal → start is the walk order; motion is not
    for (int idx : chain) out->push_back(vertices[agent][idx]);
    return true;
  };

  // A robot moving along `path` overlaps a disc of radius_sum parked at p iff
  // some motion segment comes strictly closer than radius_sum.
  auto path_hits_point = [&](const std::vector<Point>& path, const Point& p, double radius_sum) {
    for (size_t t = 0; t + 1 < path.size(); ++t) {
      if (core::point_segment_distance(p, path[t], path[t + 1]) < radius_sum) return true;
    }
    return false;
  };

  // §4.2 prioritized decoupling: per-robot BFS paths + priority DAG; acyclic →
  // execute in Kahn order (lowest index first among ready). The returned paths
  // START at q_state's points (the caller splices them onto the tree chain);
  // cyclic or unreachable → failure, next of the K candidates is tried.
  auto connect = [&](const State& q_state, std::vector<std::vector<Point>>* out) -> bool {
    std::vector<std::vector<Point>> paths_local;
    for (size_t i = 0; i < m; ++i) {
      std::vector<Point> pi;
      if (!bfs_local(q_state[i], i, &pi)) return false;
      paths_local.push_back(pi);
    }

    // Priority DAG. π_i hitting D(v_j) → i moves AFTER j; π_i hitting D(t_j) →
    // i moves BEFORE j. Both directions for one pair is a cycle — the candidate
    // honestly fails. deps[i]: robots that must move before i.
    std::vector<std::set<int>> deps(m);
    for (size_t i = 0; i < m; ++i) {
      for (size_t j = i + 1; j < m; ++j) {
        const double r_sum = radii[i] + radii[j];
        if (path_hits_point(paths_local[i], vertices[j][q_state[j]], r_sum)) deps[i].insert(static_cast<int>(j));
        if (path_hits_point(paths_local[j], vertices[i][q_state[i]], r_sum)) deps[j].insert(static_cast<int>(i));
        if (path_hits_point(paths_local[i], goals[j], r_sum)) deps[j].insert(static_cast<int>(i));
        if (path_hits_point(paths_local[j], goals[i], r_sum)) deps[i].insert(static_cast<int>(j));
      }
    }

    // Kahn topological order, lowest index first among ready robots.
    std::vector<int> order;
    std::set<int> done;
    while (order.size() < m) {
      int ready = -1;
      for (int r = 0; r < static_cast<int>(m); ++r) {
        if (done.count(r)) continue;
        bool all_done = true;
        for (int d : deps[r]) {
          if (!done.count(d)) {
            all_done = false;
            break;
          }
        }
        if (all_done) {
          ready = r;
          break;
        }
      }
      if (ready == -1) return false;  // cyclic priorities — this candidate fails
      order.push_back(ready);
      done.insert(ready);
    }

    // Execute sequentially: each robot walks its π_i one edge per tick; everyone
    // else waits in place (their path repeats its last point).
    std::vector<std::vector<Point>> paths;
    for (size_t i = 0; i < m; ++i) paths.push_back(std::vector<Point>{vertices[i][q_state[i]]});
    for (int mover : order) {
      const std::vector<Point>& pi = paths_local[mover];
      for (size_t t = 1; t < pi.size(); ++t) {
        for (size_t a = 0; a < m; ++a) {
          paths[a].push_back(static_cast<int>(a) == mover ? pi[t] : paths[a].back());
        }
      }
    }
    *out = paths;
    return true;
  };

  // --- main loop (Algorithm 1) ---------------------------------------------------
  // Round i: EXPAND runs N = 2^i sample-and-extend iterations (the paper's
  // parameter-free schedule — the round index IS the budget), then
  // CONNECT_TO_TARGET tries the K = i tree nodes nearest to the goal points.
  std::vector<std::vector<Point>> connector_paths;
  bool connected = false;
  std::vector<int> chain_nodes;
  for (int round_i = 1; round_i <= max_rounds && !connected; ++round_i) {
    const long long iterations = 1LL << round_i;
    for (long long it = 0; it < iterations; ++it) {
      std::vector<Point> q_rand;
      for (size_t i = 0; i < m; ++i) {
        const double ux = rng.uniform();
        const double uy = rng.uniform();
        q_rand.push_back(Point{x_min + ux * (x_max - x_min), y_min + uy * (y_max - y_min)});
      }
      size_t near_idx = 0;
      double near_d = joint_distance(states[0], q_rand);
      for (size_t idx = 1; idx < states.size(); ++idx) {
        const double d = joint_distance(states[idx], q_rand);
        if (d < near_d) {  // strict < — earliest insertion wins ties
          near_d = d;
          near_idx = idx;
        }
      }
      State candidate;
      if (!oracle(states[near_idx], q_rand, &candidate)) continue;  // O_D = ∅
      if (index_of.count(candidate)) continue;                       // already a tree vertex
      if (!edge_valid(states[near_idx], candidate)) {
        continue;  // simultaneous motion collides — sample ignored
      }
      states.push_back(candidate);
      parents_joint.push_back(static_cast<int>(near_idx));
      index_of[candidate] = static_cast<int>(states.size()) - 1;
      if (recorder != nullptr) recorder->node_expanded(flatten(candidate));
    }

    // CONNECT TO_TARGET: the K = round_i nearest tree nodes to the goal points,
    // ranked by (joint distance, insertion index).
    std::vector<std::pair<double, int>> ranked;
    for (size_t idx = 0; idx < states.size(); ++idx) {
      ranked.emplace_back(joint_distance(states[idx], goal_points), static_cast<int>(idx));
    }
    std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
      return a.first != b.first ? a.first < b.first : a.second < b.second;
    });
    for (int k = 0; k < round_i && k < static_cast<int>(ranked.size()); ++k) {
      const int node_idx = ranked[k].second;
      std::vector<std::vector<Point>> connected_paths;
      if (!connect(states[node_idx], &connected_paths)) continue;
      // RETRIEVE PATH: the tree chain root → q (all robots move on every tree
      // tick), then the connector's sequential ticks spliced on.
      std::vector<int> chain;
      int node = node_idx;
      while (node != -1) {
        chain.push_back(node);
        node = parents_joint[node];
      }
      std::reverse(chain.begin(), chain.end());
      chain_nodes = chain;
      connector_paths = connected_paths;
      connected = true;
      break;
    }
  }

  if (!connected) return fail(static_cast<int>(states.size()), recorder);

  // Tree phase: every tree edge moves ALL robots simultaneously (tensor product —
  // no self-loops on the individual roadmaps).
  std::vector<std::vector<Point>> paths;
  for (size_t a = 0; a < m; ++a) paths.push_back(std::vector<Point>{vertices[a][root[a]]});
  for (size_t ci = 1; ci < chain_nodes.size(); ++ci) {
    const State& state = states[chain_nodes[ci]];
    for (size_t a = 0; a < m; ++a) paths[a].push_back(vertices[a][state[a]]);
  }
  // Connector phase: splice the sequential ticks on (skip the duplicated head).
  for (size_t a = 0; a < m; ++a) {
    connector_paths[a].erase(connector_paths[a].begin());
    paths[a].insert(paths[a].end(), connector_paths[a].begin(), connector_paths[a].end());
  }

  // Trim after each agent's LAST move, then the unit-cost metric (a step costs 1
  // unless it is a wait at one's own goal — the same metric every planner here
  // reports).
  std::vector<std::vector<Point>> trimmed;
  double cost = 0.0;
  for (size_t i = 0; i < m; ++i) {
    size_t last_move = 0;
    for (size_t t = 1; t < paths[i].size(); ++t) {
      if (!(paths[i][t] == paths[i][t - 1])) last_move = t;
    }
    std::vector<Point> path_i(paths[i].begin(), paths[i].begin() + static_cast<long>(last_move) + 1);
    for (size_t t = 1; t < path_i.size(); ++t) {
      if (!(path_i[t] == path_i[t - 1] && path_i[t] == goals[i])) cost += 1.0;
    }
    trimmed.push_back(path_i);
  }

  double makespan = 0.0;
  for (const auto& p : trimmed) {
    makespan = std::max(makespan, static_cast<double>(static_cast<int>(p.size()) - 1));
  }
  if (recorder != nullptr) {
    for (size_t i = 0; i < m; ++i) recorder->path_found(trimmed[i], static_cast<int>(i));
    recorder->planning_finished(true, {{"expanded_nodes", static_cast<double>(states.size())},
                                       {"makespan", makespan}, {"sum_of_costs", cost}});
  }
  core::ContinuousPlanResult result;
  result.success = true;
  result.paths = trimmed;
  result.cost = cost;
  result.stats.expanded_nodes = static_cast<int>(states.size());
  return result;
}

}  // namespace mrmp::sampling
