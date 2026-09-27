#include "mrmp/mapf/prioritized_astar.hpp"

#include <algorithm>
#include <deque>
#include <queue>
#include <utility>

namespace mrmp::mapf {

using core::Cell;
using core::DiscreteSpace;

namespace {

// Cell occupied at step t by a finished path: after arrival the agent stays.
Cell occupied(const std::vector<Cell>& path, int t) {
  return static_cast<size_t>(t) < path.size() ? path[static_cast<size_t>(t)] : path.back();
}

// Static flood fill over passable cells — no reservations involved. FIFO order is
// irrelevant to the result (a set), but fixed for determinism across languages.
std::set<Cell> reachable(const DiscreteSpace& space, const Cell& start) {
  std::set<Cell> seen{start};
  std::deque<Cell> queue{start};
  while (!queue.empty()) {
    Cell cell = queue.front();
    queue.pop_front();
    for (const auto& [succ, cost] : space.neighbors(cell)) {
      (void)cost;
      if (seen.insert(succ).second) queue.push_back(succ);
    }
  }
  return seen;
}

// Heap node: f = g + h with g == t, plus the push counter for a stable FIFO
// tie-break among equal f — the exact counterpart of Python's (f, seq) tuples.
struct Node {
  double f;
  long long seq;
  Cell cell;
  int t;
};

// Min-heap on lexicographic (f, seq): std::priority_queue is a max-heap, so the
// comparator reports "a pops after b".
struct LowerPriority {
  bool operator()(const Node& a, const Node& b) const {
    if (a.f != b.f) return a.f > b.f;
    return a.seq > b.seq;
  }
};

}  // namespace

core::MultiPlanResult PrioritizedAStar::plan(const DiscreteSpace& space,
                                             const std::vector<core::AgentTask>& tasks,
                                             core::TraceRecorder* recorder) {
  std::vector<std::vector<Cell>> paths;
  int expanded = 0;
  for (size_t agent = 0; agent < tasks.size(); ++agent) {
    auto [path, n] = plan_one(space, tasks[agent], paths, static_cast<int>(agent), recorder);
    expanded += n;
    if (!path.has_value()) {
      // A later agent boxed in by earlier paths fails the whole plan. The trace
      // keeps every expansion and the earlier agents' path_found events; metrics
      // report the failure honestly (0 cost).
      if (recorder != nullptr) {
        recorder->planning_finished(false, {{"expanded_nodes", static_cast<double>(expanded)},
                                            {"makespan", 0.0},
                                            {"sum_of_costs", 0.0}});
      }
      core::MultiPlanResult result;
      result.success = false;
      result.stats.expanded_nodes = expanded;
      return result;
    }
    if (recorder != nullptr) recorder->path_found(*path, static_cast<int>(agent));
    paths.push_back(std::move(*path));
  }
  double cost = 0.0;
  int makespan = 0;
  for (const auto& p : paths) {
    cost += static_cast<double>(p.size() - 1);
    makespan = std::max(makespan, static_cast<int>(p.size()) - 1);
  }
  if (recorder != nullptr) {
    recorder->planning_finished(true, {{"expanded_nodes", static_cast<double>(expanded)},
                                       {"makespan", static_cast<double>(makespan)},
                                       {"sum_of_costs", cost}});
  }
  core::MultiPlanResult result;
  result.success = true;
  result.paths = std::move(paths);
  result.cost = cost;
  result.stats.expanded_nodes = expanded;
  return result;
}

std::pair<std::optional<std::vector<Cell>>, int> PrioritizedAStar::plan_one(
    const DiscreteSpace& space, const core::AgentTask& task,
    const std::vector<std::vector<Cell>>& planned, int agent, core::TraceRecorder* recorder) const {
  const Cell start = task.start;
  const Cell goal = task.goal;
  // An earlier path standing on this agent's start at t=0 is an unavoidable joint
  // conflict — no move can undo it.
  for (const auto& p : planned) {
    if (occupied(p, 0) == start) {
      return {std::nullopt, 0};
    }
  }

  std::set<Cell> reach = reachable(space, start);
  if (!reach.count(goal)) {
    return {std::nullopt, 0};
  }
  int frozen = 0;
  for (const auto& p : planned) frozen = std::max(frozen, static_cast<int>(p.size()) - 1);
  const int horizon = frozen + static_cast<int>(reach.size());

  // Every state is pushed exactly once (g == t never improves), so no relaxation
  // is needed. Finiteness: after time `frozen` all reservations are parked, so any
  // feasible suffix simplifies to a simple static path of length <= |reachable| —
  // states past the horizon cannot matter, and A* pops by f before ever reaching it
  // on the success path.
  std::priority_queue<Node, std::vector<Node>, LowerPriority> frontier;
  long long counter = 0;
  frontier.push({space.heuristic(start, goal), counter++, start, 0});
  std::set<std::pair<Cell, int>> seen{{std::make_pair(start, 0)}};
  std::map<std::pair<Cell, int>, Cell> parent;
  int n = 0;

  while (!frontier.empty()) {
    const Node cur = frontier.top();
    frontier.pop();
    // A pop IS the expansion — each state is pushed exactly once.
    ++n;
    if (recorder != nullptr)
      recorder->node_expanded(cur.cell, static_cast<double>(cur.t), agent, cur.t);
    // Popping the goal is not enough: under stay-at-goal semantics the finished
    // agent occupies its goal forever, so the pop is accepted only when no earlier
    // path visits the goal at any step >= t.
    bool guard = true;
    for (const auto& p : planned) {
      for (int tt = cur.t; static_cast<size_t>(tt) < p.size(); ++tt) {
        if (occupied(p, tt) == goal) {
          guard = false;
          break;
        }
      }
      if (!guard) break;
    }
    if (cur.cell == goal && guard) {
      std::vector<Cell> path{cur.cell};
      Cell cur_cell = cur.cell;
      int cur_t = cur.t;
      while (cur_t > 0) {
        Cell prev = parent[std::make_pair(cur_cell, cur_t)];
        path.push_back(prev);
        cur_cell = prev;
        --cur_t;
      }
      std::reverse(path.begin(), path.end());
      return {std::move(path), n};
    }
    if (cur.t == horizon) continue;
    for (const auto& [succ, cost] : space.neighbors(cur.cell)) {
      (void)cost;
      const int t2 = cur.t + 1;
      // A move into succ at step t+1 is legal only if no earlier path occupies
      // succ at t+1 (vertex conflict) nor swaps it with the current cell across
      // that step (edge/swap conflict).
      bool blocked = false;
      for (const auto& p : planned) {
        if (occupied(p, t2) == succ || (occupied(p, cur.t) == succ && occupied(p, t2) == cur.cell)) {
          blocked = true;
          break;
        }
      }
      if (blocked) continue;
      const auto key = std::make_pair(succ, t2);
      if (!seen.insert(key).second) continue;
      parent[key] = cur.cell;
      frontier.push({static_cast<double>(t2) + space.heuristic(succ, goal), counter++, succ, t2});
    }
  }
  return {std::nullopt, n};
}

}  // namespace mrmp::mapf
