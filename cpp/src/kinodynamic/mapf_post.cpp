#include "mrmp/kinodynamic/mapf_post.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "mrmp/search/cbs.hpp"

namespace mrmp::kinodynamic {

using core::Cell;
using core::DiscreteSpace;
using core::TimedPlanResult;

core::TimedPlanResult MapfPost::plan(const DiscreteSpace& space,
                                     const std::vector<core::AgentTask>& tasks,
                                     core::TraceRecorder* recorder) {
  const double delta = params_.get_float("delta");
  // The config range already clamps into [0.0, 0.5]; the paper's condition is strict
  // positivity — delta = 0 would collapse every safety marker onto its location vertex and
  // void Theorem 2's positive bound.
  if (!(delta > 0.0)) {
    throw std::runtime_error("mapf_post: delta must be > 0");
  }

  // The underlying discrete plan: CBS runs SILENTLY (recorder = nullptr) — this branch's
  // trace carries only the schedule it produces, and its own metric counts STN relaxations,
  // not the base search's expansions. A budget hit or an exhausted tree is inherited
  // honestly as "no plan to post-process".
  core::MultiPlanResult base = search::Cbs(params_).plan(space, tasks, nullptr);
  if (!base.success) {
    if (recorder != nullptr) {
      recorder->planning_finished(false, {{"expanded_nodes", 0.0},
                                          {"makespan", 0.0},
                                          {"sum_of_costs", 0.0}});
    }
    return TimedPlanResult{};
  }

  // Route extraction: drop waits (keep the first of every run of identical cells); retained
  // events keep their discrete step indices — those steps only ever ORDER Type-2 edges, they
  // are never times.
  std::vector<std::vector<Cell>> routes;
  std::vector<std::vector<int>> steps;
  for (const auto& path : base.paths) {
    std::vector<Cell> route;
    std::vector<int> kept;
    for (size_t t = 0; t < path.size(); ++t) {
      if (route.empty() || !(path[t] == route.back())) {
        route.push_back(path[t]);
        kept.push_back(static_cast<int>(t));
      }
    }
    routes.push_back(route);
    steps.push_back(kept);
  }

  std::vector<double> labels;
  std::vector<Edge> edges;
  std::vector<std::vector<int>> event_ids;
  build_stn(base.paths, routes, steps, tasks, delta, labels, edges, event_ids);
  const int expanded = relax(labels, edges);

  std::vector<std::vector<double>> times_by_agent;
  for (size_t j = 0; j < event_ids.size(); ++j) {
    std::vector<double> times;
    for (int id : event_ids[j]) times.push_back(labels[static_cast<size_t>(id)]);
    times_by_agent.push_back(times);
  }
  // Both metrics are the goal arrival times only (the flow-time analogue), summed and
  // maxed — never a step count.
  double cost = 0.0;
  double makespan = times_by_agent.front().back();
  for (const auto& times : times_by_agent) {
    makespan = std::max(makespan, times.back());
    cost += times.back();
  }

  if (recorder != nullptr) {
    for (size_t k = 0; k < routes.size(); ++k) {
      recorder->schedule_found(static_cast<int>(k), routes[k], times_by_agent[k]);
    }
    recorder->planning_finished(true, {{"expanded_nodes", static_cast<double>(expanded)},
                                       {"makespan", makespan},
                                       {"sum_of_costs", cost}});
  }

  TimedPlanResult out;
  out.success = true;
  out.routes = routes;
  out.times = times_by_agent;
  out.cost = cost;
  out.makespan = makespan;
  out.stats.expanded_nodes = expanded;
  return out;
}

void MapfPost::build_stn(const std::vector<std::vector<Cell>>& paths,
                         const std::vector<std::vector<Cell>>& routes,
                         const std::vector<std::vector<int>>& steps,
                         const std::vector<core::AgentTask>& tasks, double delta,
                         std::vector<double>& labels, std::vector<Edge>& edges,
                         std::vector<std::vector<int>>& event_ids) {
  const size_t n_agents = routes.size();
  // Event vertices first: ids assigned per agent in step order.
  int next_id = 0;
  for (const auto& route : routes) {
    std::vector<int> ids;
    for (size_t i = 0; i < route.size(); ++i) ids.push_back(next_id++);
    event_ids.push_back(ids);
  }
  // Chain sub-edges and their markers, agent by agent, segment by segment.
  std::vector<std::vector<int>> m1_of(n_agents), m2_of(n_agents);
  for (size_t j = 0; j < n_agents; ++j) {
    const double v_j = tasks[j].vmax;
    const std::vector<int>& ids = event_ids[j];
    for (size_t i = 0; i + 1 < ids.size(); ++i) {
      const int m1 = next_id++;
      const int m2 = next_id++;
      m1_of[j].push_back(m1);
      m2_of[j].push_back(m2);
      edges.push_back(Edge{ids[i], m1, delta / v_j});
      edges.push_back(Edge{m1, m2, (1.0 - 2.0 * delta) / v_j});
      edges.push_back(Edge{m2, ids[i + 1], delta / v_j});
    }
  }
  // Type-2 precedence: for every retained event of j (except a final parked one — it has
  // no outgoing edge to attach anything to), scan each other agent's RAW path beyond that
  // step; the first later visit wins (later ones are implied by transitivity through it).
  // The found occurrence is a retained arrival by construction: the step before it held a
  // different cell, or j would have collided with k there.
  for (size_t j = 0; j < n_agents; ++j) {
    const std::vector<int>& ids_j = event_ids[j];
    for (size_t i = 0; i + 1 < ids_j.size(); ++i) {
      const Cell cell = routes[j][i];
      const int step = steps[j][i];
      for (size_t k = 0; k < n_agents; ++k) {
        if (k == j) continue;
        int found = -1;
        for (int t = step + 1; t < static_cast<int>(paths[k].size()); ++t) {
          if (paths[k][static_cast<size_t>(t)] == cell) {
            found = t;
            break;
          }
        }
        if (found < 0) continue;
        size_t idx_k = 0;
        while (steps[k][idx_k] != found) ++idx_k;
        // The later visit is at step >= 1 so it arrived from another cell — its incoming
        // segment (and its m2 marker) exists.
        edges.push_back(Edge{m1_of[j][i], m2_of[k][idx_k - 1], 0.0});
      }
    }
  }
  labels.assign(static_cast<size_t>(next_id), -std::numeric_limits<double>::infinity());
  for (size_t j = 0; j < n_agents; ++j) {
    labels[static_cast<size_t>(event_ids[j][0])] = 0.0;  // X_S -> every source, bound [0, 0]
  }
}

int MapfPost::relax(std::vector<double>& labels, const std::vector<Edge>& edges) {
  // -inf + lb stays -inf, so an edge out of an unrelaxed vertex never fires; the graph is
  // acyclic, so every label ends finite (every vertex chains from some pinned source).
  int expanded = 0;
  bool changed = true;
  while (changed) {
    changed = false;
    for (const Edge& e : edges) {
      const double cand = labels[static_cast<size_t>(e.src)] + e.lb;
      if (cand > labels[static_cast<size_t>(e.dst)]) {
        labels[static_cast<size_t>(e.dst)] = cand;
        ++expanded;
        changed = true;
      }
    }
  }
  return expanded;
}

}  // namespace mrmp::kinodynamic
