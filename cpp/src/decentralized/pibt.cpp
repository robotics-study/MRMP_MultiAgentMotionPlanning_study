#include "mrmp/decentralized/pibt.hpp"

#include <algorithm>
#include <cassert>
#include <limits>
#include <tuple>
#include <vector>

namespace mrmp::decentralized {

using core::Cell;
using core::DiscreteSpace;

// 4-connected neighbor order — the SAME fixed convention as the map layer's move set.
// The wait action is not a neighbor: staying put is the current cell itself, which is
// always in the candidate list (that is what makes top-level decisions fail-proof).
namespace {
constexpr int kMoves4[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

// Distance from an unreachable goal: sorts after every reachable cell; among equals
// the fixed row-major order still decides. IEEE-754 infinity compares identically to
// Python's float('inf') by construction.
double inf_dist() { return std::numeric_limits<double>::infinity(); }
}  // namespace

std::vector<Cell> Pibt::nbrs(const std::set<Cell>& free_cells, const Cell& c) {
  std::vector<Cell> out;
  for (const auto& d : kMoves4) {
    Cell n{c.row + d[0], c.col + d[1]};
    if (free_cells.count(n)) out.push_back(n);
  }
  return out;
}

std::map<Cell, int> Pibt::bfs(const std::set<Cell>& free_cells, core::Cell goal) {
  std::map<Cell, int> dist{{goal, 0}};
  std::vector<Cell> queue{goal};
  size_t head = 0;
  while (head < queue.size()) {
    const Cell c = queue[head];
    ++head;
    for (const Cell& n : nbrs(free_cells, c)) {
      if (!dist.count(n)) {
        dist[n] = dist.at(c) + 1;
        queue.push_back(n);
      }
    }
  }
  return dist;
}

bool Pibt::decide(Sim& sim, int i, std::optional<int> from_agent) {
  ++sim.calls;
  const Cell cur = sim.pos[static_cast<size_t>(i)];
  const std::map<Cell, int>& dist = sim.dists[static_cast<size_t>(i)];

  // Candidates: current cell plus free neighbors (fixed order), sorted by the pinned
  // rank — distance-to-goal ascending (unreachable sorts last as +inf), then
  // unoccupied-before-occupied, then row-major cell order. The full key is unique per
  // candidate (cells are distinct), so this total order matches Python's stable sort.
  std::vector<Cell> candidates = nbrs(sim.free_cells, cur);
  candidates.push_back(cur);
  const auto rank = [&](const Cell& u) {
    auto it = dist.find(u);
    int occupied = 0;
    for (size_t j = 0; j < sim.pos.size(); ++j) {
      if (sim.pos[j] == u) occupied = 1;
    }
    return std::tuple<double, int, int, int>(it == dist.end() ? inf_dist() : (double)it->second,
                                             occupied, u.row, u.col);
  };
  std::stable_sort(candidates.begin(), candidates.end(),
                   [&](const Cell& a, const Cell& b) { return rank(a) < rank(b); });

  for (const Cell& v : candidates) {
    bool claimed = false;
    for (const auto& nxt : sim.nxt) {
      if (nxt.has_value() && *nxt == v) claimed = true;
    }
    if (claimed) continue;  // already claimed this step — vertex conflict by definition
    if (from_agent.has_value() && v == sim.pos[static_cast<size_t>(*from_agent)]) {
      continue;  // the claimant's own cell: taking it would be a swap
    }
    sim.nxt[static_cast<size_t>(i)] = v;  // tentative claim BEFORE recursing
    std::optional<int> occ;
    for (size_t j = 0; j < sim.pos.size(); ++j) {
      if (sim.pos[j] == v) {
        occ = static_cast<int>(j);
        break;
      }
    }
    if (occ.has_value() && *occ != i && !sim.nxt[static_cast<size_t>(*occ)].has_value()) {
      if (decide(sim, *occ, i)) return true;
      sim.nxt[static_cast<size_t>(i)] = std::nullopt;  // backtracked — never claimed
      continue;
    }
    return true;  // unoccupied, or its occupant already vacates: settled
  }
  sim.nxt[static_cast<size_t>(i)] = cur;  // stuck: stay put and report invalid upward
  return false;
}

core::MultiPlanResult Pibt::fail(core::TraceRecorder* recorder, int calls) const {
  // Honest failure (the branch's convention): no paths emitted, the decision count up
  // to the budget is still reported.
  if (recorder != nullptr) {
    recorder->planning_finished(false, {{"expanded_nodes", static_cast<double>(calls)},
                                        {"makespan", 0.0},
                                        {"sum_of_costs", 0.0}});
  }
  core::MultiPlanResult result;
  result.success = false;
  result.stats.expanded_nodes = calls;
  return result;
}

core::MultiPlanResult Pibt::plan(const DiscreteSpace& space,
                                 const std::vector<core::AgentTask>& tasks,
                                 core::TraceRecorder* recorder) {
  Sim sim;
  for (const Cell& c : space.cells()) sim.free_cells.insert(c);
  std::vector<Cell> starts, goals;
  for (const auto& t : tasks) {
    starts.push_back(t.start);
    goals.push_back(t.goal);
  }
  // The paper's instance is well-formed by definition (unique starts AND unique goals
  // on passable cells). A violating input is not an instance of this problem — report
  // it honestly, no plan.
  {
    std::set<Cell> s_starts(starts.begin(), starts.end());
    std::set<Cell> s_goals(goals.begin(), goals.end());
    bool invalid = s_starts.size() < starts.size() || s_goals.size() < goals.size();
    for (const Cell& c : starts) invalid = invalid || !sim.free_cells.count(c);
    for (const Cell& c : goals) invalid = invalid || !sim.free_cells.count(c);
    if (invalid) return fail(recorder, 0);
  }

  const int k = static_cast<int>(tasks.size());
  const int max_steps = params_.get_int("max_steps");
  // Static distance tables (paper: BFS from each goal up front).
  sim.goals = goals;
  for (const Cell& g : goals) sim.dists.push_back(bfs(sim.free_cells, g));
  sim.pos = starts;

  // Pinned priorities: agent 0 highest (ε closest to 1). Distinct by construction —
  // the paper requires distinct ε and only asks ε ∈ [0,1). Integer division would be a
  // bug; this is exactly Python's true division of the same integer operands.
  std::vector<double> eps(static_cast<size_t>(k)), p(static_cast<size_t>(k));
  for (int i = 0; i < k; ++i) {
    eps[static_cast<size_t>(i)] = static_cast<double>(k - 1 - i) / static_cast<double>(k);
    p[static_cast<size_t>(i)] = eps[static_cast<size_t>(i)];
  }

  std::vector<std::vector<Cell>> paths(starts.size());
  for (size_t i = 0; i < starts.size(); ++i) paths[i].push_back(starts[i]);

  auto all_at_goal = [&]() {
    for (int i = 0; i < k; ++i) {
      if (!(sim.pos[static_cast<size_t>(i)] == goals[static_cast<size_t>(i)])) return false;
    }
    return true;
  };

  int t = 0;
  while (!all_at_goal()) {
    if (t >= max_steps) {
      // Honest budget exhaustion — not a proof of unsolvability.
      return fail(recorder, sim.calls);
    }
    // Priority update (paper line 7): on goal → reset to ε; travelling → +1.
    for (int i = 0; i < k; ++i) {
      p[static_cast<size_t>(i)] =
          (sim.pos[static_cast<size_t>(i)] == goals[static_cast<size_t>(i)])
              ? eps[static_cast<size_t>(i)]
              : p[static_cast<size_t>(i)] + 1.0;
    }
    // Decreasing priority; values are distinct so the order is total.
    std::vector<int> order(static_cast<size_t>(k));
    for (int i = 0; i < k; ++i) order[static_cast<size_t>(i)] = i;
    std::stable_sort(order.begin(), order.end(),
                     [&](int a, int b) { return p[static_cast<size_t>(a)] > p[static_cast<size_t>(b)]; });
    sim.nxt.assign(static_cast<size_t>(k), std::nullopt);
    for (int i : order) {
      if (!sim.nxt[static_cast<size_t>(i)].has_value()) decide(sim, i, std::nullopt);
    }
    ++t;
    // Simultaneous commit: every decision is made against time-t positions, and only
    // after ALL decisions do the positions advance (top-level calls never fail —
    // Lemma 1 — so a nullopt here is an assertion failure, not a recoverable state).
    for (int i = 0; i < k; ++i) {
      assert(sim.nxt[static_cast<size_t>(i)].has_value());
      paths[static_cast<size_t>(i)].push_back(*sim.nxt[static_cast<size_t>(i)]);
      sim.pos[static_cast<size_t>(i)] = *sim.nxt[static_cast<size_t>(i)];
    }
  }

  double cost = 0.0;
  for (const auto& path : paths) {
    for (size_t s = 1; s < path.size(); ++s) {
      if (!(path[s] == path[s - 1])) cost += 1.0;
    }
  }
  if (recorder != nullptr) {
    for (size_t k2 = 0; k2 < paths.size(); ++k2) {
      recorder->path_found(paths[k2], static_cast<int>(k2));
    }
    recorder->planning_finished(
        true, {{"expanded_nodes", static_cast<double>(sim.calls)},
               {"makespan", static_cast<double>(t)},
               {"sum_of_costs", cost}});
  }
  core::MultiPlanResult result;
  result.success = true;
  result.paths = std::move(paths);
  result.cost = cost;
  result.stats.expanded_nodes = sim.calls;
  return result;
}

}  // namespace mrmp::decentralized
