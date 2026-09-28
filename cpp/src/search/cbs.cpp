#include "mrmp/search/cbs.hpp"

#include <algorithm>
#include <deque>
#include <map>
#include <queue>
#include <tuple>
#include <utility>

namespace mrmp::search {

using core::Cell;
using core::ConflictKind;
using core::DiscreteSpace;

namespace {

// Cell occupied at step t by a finished path: after arrival the agent stays.
Cell occupied(const std::vector<Cell>& path, int t) {
  return static_cast<size_t>(t) < path.size() ? path[static_cast<size_t>(t)] : path.back();
}

// Static flood fill over passable cells — no constraints involved. FIFO order is
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

// Low-level heap node: f = g + h with g == t, plus the push counter for a stable
// FIFO tie-break among equal f — the exact counterpart of Python's (f, seq).
struct Node {
  double f;
  long long seq;
  Cell cell;
  int t;
};

struct LowerPriority {
  bool operator()(const Node& a, const Node& b) const {
    if (a.f != b.f) return a.f > b.f;
    return a.seq > b.seq;
  }
};

}  // namespace

core::MultiPlanResult Cbs::plan(const DiscreteSpace& space,
                               const std::vector<core::AgentTask>& tasks,
                               core::TraceRecorder* recorder) {
  const int max_ct = params_.get_int("max_ct_expansions");

  // Root expansion (the first of the budget): every agent planned unconstrained,
  // in index order. If an unconstrained sub-search fails the goal is statically
  // unreachable — no constraint set can ever help, so the whole instance fails.
  int expanded = 0;
  std::vector<std::vector<Cell>> root_paths;
  for (size_t k = 0; k < tasks.size(); ++k) {
    auto [path, n] = plan_one(space, tasks[k], {}, static_cast<int>(k), recorder);
    expanded += n;
    if (!path.has_value()) {
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
    root_paths.push_back(std::move(*path));
  }

  // Best-first CT queue: cheapest sum-of-costs first, ties broken by creation
  // order (the same FIFO tie-break as every other page).
  struct QueueEntry {
    double cost;
    long long seq;
    CtNode node;
  };
  struct LowerQueuePriority {
    bool operator()(const QueueEntry& a, const QueueEntry& b) const {
      if (a.cost != b.cost) return a.cost > b.cost;
      return a.seq > b.seq;
    }
  };

  double root_cost = 0.0;
  for (const auto& p : root_paths) root_cost += static_cast<double>(p.size()) - 1.0;
  long long counter = 0;
  CtNode root_node;
  root_node.constraints.resize(tasks.size());
  root_node.paths = root_paths;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, LowerQueuePriority> queue;
  queue.push(QueueEntry{root_cost, counter++, std::move(root_node)});
  int ct_expansions = 1;

  while (!queue.empty() && ct_expansions < max_ct) {
    QueueEntry top = queue.top();
    queue.pop();
    ++ct_expansions;
    const double cost = top.cost;
    const CtNode& node = top.node;

    auto conflict = first_conflict(node.paths);
    if (!conflict.has_value()) {
      // Best-first pops solutions by increasing sum-of-costs and every node's
      // paths are individually optimal under their constraints — the first
      // conflict-free node IS jointly optimal. Done.
      int makespan = 0;
      for (const auto& p : node.paths) makespan = std::max(makespan, static_cast<int>(p.size()) - 1);
      if (recorder != nullptr) {
        for (size_t k = 0; k < node.paths.size(); ++k) {
          recorder->path_found(node.paths[k], static_cast<int>(k));
        }
        recorder->planning_finished(true, {{"expanded_nodes", static_cast<double>(expanded)},
                                           {"makespan", static_cast<double>(makespan)},
                                           {"sum_of_costs", cost}});
      }
      core::MultiPlanResult result;
      result.success = true;
      result.paths = node.paths;
      result.cost = cost;
      result.stats.expanded_nodes = expanded;
      return result;
    }

    const auto [kind, cell, t, pair_ij, to] = *conflict;
    if (recorder != nullptr) recorder->conflict_found(kind, cell, t, pair_ij, to);

    // Branch: one child per conflicting agent, in pair order. Each child forbids
    // its agent from THIS collision and re-plans only that agent; a child whose
    // low level fails dies and is never pushed.
    for (const int agent : {pair_ij.first, pair_ij.second}) {
      std::vector<std::vector<Constraint>> child_constraints = node.constraints;
      child_constraints[static_cast<size_t>(agent)].push_back(Constraint{kind, cell, t, to});
      if (recorder != nullptr) recorder->constraint_added(agent, kind, cell, t, to);
      auto [path, n] = plan_one(space, tasks[static_cast<size_t>(agent)],
                                child_constraints[static_cast<size_t>(agent)], agent, recorder);
      expanded += n;
      if (!path.has_value()) continue;
      std::vector<std::vector<Cell>> child_paths = node.paths;
      const double old_cost = static_cast<double>(child_paths[static_cast<size_t>(agent)].size()) - 1.0;
      child_paths[static_cast<size_t>(agent)] = std::move(*path);
      const double new_cost =
          static_cast<double>(child_paths[static_cast<size_t>(agent)].size()) - 1.0;
      queue.push(QueueEntry{cost - old_cost + new_cost, counter++,
                            CtNode{std::move(child_constraints), std::move(child_paths)}});
    }
  }

  // Queue exhausted before the budget: every branch died on its own constraints —
  // an honest unsolvability verdict. (Budget exhaustion, by contrast, is only "no
  // solution found within budget" — see the header doc.)
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

std::pair<std::optional<std::vector<Cell>>, int> Cbs::plan_one(
    const DiscreteSpace& space, const core::AgentTask& task,
    const std::vector<Constraint>& constraints, int agent, core::TraceRecorder* recorder) const {
  const Cell start = task.start;
  const Cell goal = task.goal;
  // A vertex constraint on the start cell at t=0 is unavoidable — no move can undo
  // being born inside a forbidden cell.
  for (const auto& c : constraints) {
    if (c.kind == ConflictKind::Vertex && c.cell == start && c.t == 0) return {std::nullopt, 0};
  }

  std::set<Cell> reach = reachable(space, start);
  if (!reach.count(goal)) return {std::nullopt, 0};
  int constrained_until = 0;
  for (const auto& c : constraints) constrained_until = std::max(constrained_until, c.t);
  const int horizon = constrained_until + static_cast<int>(reach.size());

  // Every state is pushed exactly once (g == t never improves), so a pop IS the
  // expansion. Finiteness: past the last constrained step nothing binds anymore, so
  // every feasible plan has an equivalent one of length <= max(constrained t) +
  // |reachable|; states past that horizon cannot matter.
  std::priority_queue<Node, std::vector<Node>, LowerPriority> frontier;
  long long counter = 0;
  frontier.push({space.heuristic(start, goal), counter++, start, 0});
  std::set<std::pair<Cell, int>> seen{{std::make_pair(start, 0)}};
  std::map<std::pair<Cell, int>, Cell> parent;
  int expanded = 0;

  while (!frontier.empty()) {
    const Node cur = frontier.top();
    frontier.pop();
    ++expanded;
    if (recorder != nullptr)
      recorder->node_expanded(cur.cell, static_cast<double>(cur.t), agent, cur.t);
    // Goal guard: after arrival the agent occupies `goal` forever, so the pop
    // counts only when NO vertex constraint on goal binds at any t' >= t.
    bool guard = true;
    for (const auto& c : constraints) {
      if (c.kind == ConflictKind::Vertex && c.cell == goal && c.t >= cur.t) {
        guard = false;
        break;
      }
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
      return {std::move(path), expanded};
    }
    if (cur.t == horizon) continue;
    const int t2 = cur.t + 1;
    for (const auto& [succ, cost] : space.neighbors(cur.cell)) {
      (void)cost;
      // A move into succ at step t+1 is legal only if no vertex constraint binds
      // (succ, t+1) and no edge constraint canonical-matches {cell, succ} at t+1.
      bool blocked = false;
      for (const auto& c : constraints) {
        if ((c.kind == ConflictKind::Vertex && c.cell == succ && c.t == t2) ||
            (c.kind == ConflictKind::Edge && c.t == t2 && matches_edge(c, cur.cell, succ))) {
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

  return {std::nullopt, expanded};
}

bool Cbs::matches_edge(const Constraint& c, const Cell& from_cell, const Cell& to_cell) {
  const bool forward = c.cell == from_cell && c.to.has_value() && *c.to == to_cell;
  const bool backward = c.cell == to_cell && c.to.has_value() && *c.to == from_cell;
  return forward || backward;
}

std::optional<std::tuple<ConflictKind, Cell, int, std::pair<int, int>, std::optional<Cell>>>
Cbs::first_conflict(const std::vector<std::vector<Cell>>& paths) {
  // Earliest conflict over all agent pairs; ties by (cell row, col), then pair
  // i < j. Past the last arrival every occupancy is constant, so a swap can no
  // longer happen and any shared cell was already caught — scanning to max(T_k)
  // covers everything.
  int horizon = 0;
  for (const auto& p : paths) horizon = std::max(horizon, static_cast<int>(p.size()) - 1);
  for (int t = 0; t <= horizon; ++t) {
    std::set<std::tuple<Cell, int, int>> candidates;
    std::map<std::tuple<Cell, int, int>, std::pair<ConflictKind, std::optional<Cell>>> kinds;
    for (size_t i = 0; i < paths.size(); ++i) {
      for (size_t j = i + 1; j < paths.size(); ++j) {
        const Cell now_i = occupied(paths[i], t), now_j = occupied(paths[j], t);
        if (now_i == now_j) {
          const auto key = std::make_tuple(now_i, static_cast<int>(i), static_cast<int>(j));
          candidates.insert(key);
          kinds[key] = {ConflictKind::Vertex, std::nullopt};
          continue;
        }
        if (t >= 1) {
          const Cell prev_i = occupied(paths[i], t - 1), prev_j = occupied(paths[j], t - 1);
          // Swap across the step (t-1 -> t): both moved and swapped cells.
          if (now_i == prev_j && now_j == prev_i) {
            const Cell lo = std::min(now_i, now_j), hi = std::max(now_i, now_j);
            const auto key = std::make_tuple(lo, static_cast<int>(i), static_cast<int>(j));
            candidates.insert(key);
            kinds[key] = {ConflictKind::Edge, hi};
          }
        }
      }
    }
    if (!candidates.empty()) {
      const auto [cell, i, j] = *candidates.begin();
      const auto& [kind, to] = kinds[std::make_tuple(cell, i, j)];
      return std::make_tuple(kind, cell, t, std::make_pair(i, j), to);
    }
  }
  return std::nullopt;
}

}  // namespace mrmp::search
