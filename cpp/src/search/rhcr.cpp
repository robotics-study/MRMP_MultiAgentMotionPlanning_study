#include "mrmp/search/rhcr.hpp"

#include <algorithm>
#include <deque>
#include <map>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace mrmp::search {

using core::Cell;
using core::ConflictKind;
using core::DiscreteSpace;

namespace {

// Cell occupied at absolute step t by a finished path: after arrival stays.
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

// Low-level heap node: f = absolute step + Manhattan heuristic, plus the push counter
// for a stable FIFO tie-break among equal f — Python's (f, seq) exactly.
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

core::MultiPlanResult Rhcr::plan(const DiscreteSpace& space,
                                 const std::vector<core::AgentTask>& tasks,
                                 core::TraceRecorder* recorder) {
  const int window = params_.get_int("window");
  const int period = params_.get_int("replan_period");
  const int max_steps = params_.get_int("max_steps");
  // Cross-parameter constraint the param schema cannot express: h > w would execute
  // steps that no resolved window ever covered — not a harder problem, an unsafe one.
  // The paper states it as a requirement (w >= h), so throw, never clamp.
  if (period > window) {
    throw std::runtime_error("param error: replan_period " + std::to_string(period) +
                             " exceeds window " + std::to_string(window) +
                             " (every executed step must fall inside some resolved window)");
  }

  const std::set<Cell> free_cells = [&space] {
    std::set<Cell> free_set;
    for (const Cell& c : space.cells()) free_set.insert(c);
    return free_set;
  }();
  std::vector<Cell> starts, goals;
  for (const auto& task : tasks) {
    starts.push_back(task.start);
    goals.push_back(task.goal);
  }
  // Well-formed instance: distinct starts, distinct goals, passable cells. A t=0
  // collision is history rather than a conflict the window could resolve — an
  // ill-formed input is not an instance of this problem at all.
  {
    std::set<Cell> start_set(starts.begin(), starts.end());
    std::set<Cell> goal_set(goals.begin(), goals.end());
    if (start_set.size() < starts.size() || goal_set.size() < goals.size()) {
      return fail(recorder, 0);
    }
    for (size_t i = 0; i < starts.size(); ++i) {
      if (!free_cells.count(starts[i]) || !free_cells.count(goals[i])) return fail(recorder, 0);
    }
  }

  const int k = static_cast<int>(tasks.size());
  std::vector<std::vector<Cell>> executed(k);  // executed[i][t] = cell at absolute step t
  for (int i = 0; i < k; ++i) executed[static_cast<size_t>(i)].push_back(starts[static_cast<size_t>(i)]);
  int t_now = 0;
  long long calls = 0;  // every low-level pop across ALL episodes — the honest price of rolling

  for (;;) {
    bool done_now = true;
    for (int i = 0; i < k; ++i) {
      if (!(executed[static_cast<size_t>(i)][static_cast<size_t>(t_now)] == goals[static_cast<size_t>(i)])) {
        done_now = false;
        break;
      }
    }
    if (done_now) return finish(executed, goals, t_now, static_cast<int>(calls), recorder);
    if (t_now >= max_steps) {
      // Honest budget exhaustion — and RHCR can never say more: a collision beyond w
      // is invisible forever, so no episode ever proves unsolvability.
      return fail(recorder, static_cast<int>(calls));
    }

    // One Windowed MAPF episode at absolute step t_now: fresh constraint-free root from
    // the ACTUAL positions (pliable — nothing old stays reserved), conflicts resolved
    // only for arrival steps in (t_now, t_now + window].
    int expanded = 0;
    std::optional<std::vector<std::vector<Cell>>> paths =
        episode(space, goals, executed, t_now, window, recorder, &expanded);
    calls += expanded;
    if (!paths.has_value()) return fail(recorder, static_cast<int>(calls));  // honest failure

    // Commit h steps; scan them for the first simultaneous co-presence.
    int done_at = -1;
    for (int s = t_now + 1; s <= t_now + period; ++s) {
      for (int i = 0; i < k; ++i) {
        executed[static_cast<size_t>(i)].push_back(
            occupied(paths->at(static_cast<size_t>(i)), static_cast<size_t>(s)));
      }
      if (done_at < 0) {
        bool all_on_goal = true;
        for (int i = 0; i < k; ++i) {
          if (!(executed[static_cast<size_t>(i)][static_cast<size_t>(s)] ==
                goals[static_cast<size_t>(i)])) {
            all_on_goal = false;
            break;
          }
        }
        if (all_on_goal) done_at = s;
      }
    }
    if (done_at >= 0) {
      // Everyone simultaneously on their final goal — from here everyone trivially
      // stays, so execution stops there. Costs are the FIRST arrivals (the paper's
      // flowtime); makespan is this step.
      return finish(executed, goals, done_at, static_cast<int>(calls), recorder);
    }
    t_now += period;
  }
}

std::optional<std::vector<std::vector<core::Cell>>> Rhcr::episode(
    const DiscreteSpace& space, const std::vector<core::Cell>&goals,
    const std::vector<std::vector<core::Cell>>& executed, int t_now, int window,
    core::TraceRecorder* recorder, int* expanded) const {
  // Root expansion: every agent planned unconstrained from its CURRENT position, in
  // index order; a sub-search failing fails the whole episode (and with it the run).
  std::vector<std::vector<core::Cell>> root_paths;
  for (size_t i = 0; i < goals.size(); ++i) {
    auto [rel, n] = plan_one(space, executed[i][static_cast<size_t>(t_now)], goals[i], {}, t_now,
                             recorder, static_cast<int>(i));
    *expanded += n;
    if (!rel.has_value()) return std::nullopt;
    // Re-absolutize: the sub-search indexes from 0 at t_now; history + rel is absolute.
    std::vector<core::Cell> abs_path(executed[i].begin(),
                                     executed[i].begin() + static_cast<std::ptrdiff_t>(t_now));
    abs_path.insert(abs_path.end(), rel->begin(), rel->end());
    root_paths.push_back(std::move(abs_path));
  }

  // Best-first CT queue: cheapest sum of ABSOLUTE arrivals first, ties by creation
  // order (the counter restarts every episode — tie-breaks only ever live inside one).
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
  root_node.constraints.resize(goals.size());
  root_node.paths = root_paths;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, LowerQueuePriority> queue;
  queue.push(QueueEntry{root_cost, counter++, std::move(root_node)});

  while (!queue.empty()) {
    QueueEntry top = queue.top();
    queue.pop();
    const double cost = top.cost;
    const CtNode& node = top.node;

    auto conflict = first_conflict(node.paths, t_now, window);
    if (!conflict.has_value()) return node.paths;  // window-clean — this episode's solution

    const auto [kind, cell, t, pair_ij, to] = *conflict;
    if (recorder != nullptr) recorder->conflict_found(kind, cell, t, pair_ij, to);

    // Branch: one child per conflicting agent, in pair order. Each child forbids its
    // agent from THIS collision and re-plans only that agent; a child whose low level
    // fails dies and is never pushed.
    for (const int agent : {pair_ij.first, pair_ij.second}) {
      std::vector<std::vector<Constraint>> child_constraints = node.constraints;
      child_constraints[static_cast<size_t>(agent)].push_back(Constraint{kind, cell, t, to});
      if (recorder != nullptr)
        recorder->constraint_added(agent, kind, cell, t, to);
      auto [rel, n] = plan_one(space, executed[static_cast<size_t>(agent)][static_cast<size_t>(t_now)],
                               goals[static_cast<size_t>(agent)],
                               child_constraints[static_cast<size_t>(agent)], t_now, recorder, agent);
      *expanded += n;
      if (!rel.has_value()) continue;
      std::vector<std::vector<core::Cell>> child_paths = node.paths;
      const double old_cost =
          static_cast<double>(child_paths[static_cast<size_t>(agent)].size()) - 1.0;
      std::vector<core::Cell> abs_path(executed[static_cast<size_t>(agent)].begin(),
                                       executed[static_cast<size_t>(agent)].begin() +
                                           static_cast<std::ptrdiff_t>(t_now));
      abs_path.insert(abs_path.end(), rel->begin(), rel->end());
      child_paths[static_cast<size_t>(agent)] = std::move(abs_path);
      const double new_cost =
          static_cast<double>(t_now) + static_cast<double>(rel->size()) - 1.0;
      queue.push(QueueEntry{cost - old_cost + new_cost, counter++,
                            CtNode{std::move(child_constraints), std::move(child_paths)}});
    }
  }

  // The tree emptied on its own: a sub-search died under every surviving branch. On
  // these maps this never happens (parking past the horizon always fits) — but where
  // it does, THIS is what an unsolvability verdict looks like.
  return std::nullopt;
}

std::pair<std::optional<std::vector<core::Cell>>, int> Rhcr::plan_one(
    const DiscreteSpace& space, const core::Cell& start, const core::Cell& goal,
    const std::vector<Constraint>& constraints, int t_now, core::TraceRecorder* recorder,
    int agent) const {
  // A vertex constraint on the start cell AT t_now is unavoidable — no move can undo
  // being born inside a forbidden cell. (By construction constraints here only ever
  // carry steps > t_now, so this guard never fires; mirrored for exactness.)
  for (const auto& c : constraints) {
    if (c.kind == ConflictKind::Vertex && c.cell == start && c.t <= t_now)
      return {std::nullopt, 0};
  }

  std::set<Cell> reach = reachable(space, start);
  if (!reach.count(goal)) return {std::nullopt, 0};  // statically unreachable — honest fail
  int constrained_until = t_now;
  for (const auto& c : constraints) constrained_until = std::max(constrained_until, c.t);
  const int horizon = constrained_until + static_cast<int>(reach.size());

  // Every state is pushed exactly once (g == t never improves), so a pop IS the
  // expansion. Finiteness: past the last constrained step nothing binds anymore, so
  // every feasible plan has an equivalent one of length <= max(constrained t) +
  // |reachable|; states past that horizon cannot matter.
  std::priority_queue<Node, std::vector<Node>, LowerPriority> frontier;
  long long counter = 0;
  frontier.push(
      {static_cast<double>(t_now) + space.heuristic(start, goal), counter++, start, t_now});
  std::set<std::pair<Cell, int>> seen{{std::make_pair(start, t_now)}};
  std::map<std::pair<Cell, int>, Cell> parent;
  int expanded = 0;

  while (!frontier.empty()) {
    const Node cur = frontier.top();
    frontier.pop();
    ++expanded;
    if (recorder != nullptr)
      recorder->node_expanded(cur.cell, static_cast<double>(cur.t), agent, cur.t);
    // Goal guard: after arrival the agent occupies `goal` forever, so the pop counts
    // only when NO vertex constraint on goal binds at any t' >= t.
    bool guard = true;
    for (const auto& c : constraints) {
      if (c.kind == ConflictKind::Vertex && c.cell == goal && c.t >= cur.t) {
        guard = false;
        break;
      }
    }
    if (cur.cell == goal && guard) {
      std::vector<core::Cell> path{cur.cell};
      core::Cell cur_cell = cur.cell;
      int cur_t = cur.t;
      while (cur_t > t_now) {
        core::Cell prev = parent[std::make_pair(cur_cell, cur_t)];
        path.push_back(prev);
        cur_cell = prev;
        --cur_t;
      }
      std::reverse(path.begin(), path.end());  // index 0 == step t_now
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

bool Rhcr::matches_edge(const Constraint& c, const core::Cell& from_cell,
                        const core::Cell& to_cell) {
  const bool forward = c.cell == from_cell && c.to.has_value() && *c.to == to_cell;
  const bool backward = c.cell == to_cell && c.to.has_value() && *c.to == from_cell;
  return forward || backward;
}

std::optional<std::tuple<ConflictKind, Cell, int, std::pair<int, int>, std::optional<Cell>>>
Rhcr::first_conflict(const std::vector<std::vector<Cell>>& paths, int t_now, int window) {
  // Earliest conflict whose ARRIVAL step falls inside the window, (t_now, t_now +
  // window]; ties by (cell row, col), then pair i < j. Beyond the window nothing is a
  // conflict: that is what the window IS. The scan never runs past the LAST arrival —
  // an early-parked agent still occupies its goal cell, and another agent passing
  // through that cell later is a real conflict (occupied() clamps to path.back()).
  int last_arrival = static_cast<int>(paths[0].size()) - 1;
  for (const auto& p : paths) {
    last_arrival = std::max(last_arrival, static_cast<int>(p.size()) - 1);
  }
  const int horizon = std::min(t_now + window, last_arrival);
  for (int t = t_now + 1; t <= horizon; ++t) {
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
        // Swap across the step (t-1 -> t): both moved and swapped cells.
        const Cell prev_i = occupied(paths[i], t - 1), prev_j = occupied(paths[j], t - 1);
        if (now_i == prev_j && now_j == prev_i) {
          const Cell lo = std::min(now_i, now_j), hi = std::max(now_i, now_j);
          const auto key = std::make_tuple(lo, static_cast<int>(i), static_cast<int>(j));
          candidates.insert(key);
          kinds[key] = {ConflictKind::Edge, hi};
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

core::MultiPlanResult Rhcr::finish(const std::vector<std::vector<core::Cell>>& executed,
                                   const std::vector<core::Cell>& goals, int done_at, int calls,
                                   core::TraceRecorder* recorder) const {
  // Success: truncate every trajectory at the co-presence step and report. Each
  // agent's cost is its FIRST arrival (the paper's flowtime); makespan is the
  // simultaneous co-presence step.
  std::vector<std::vector<core::Cell>> paths;
  for (size_t k = 0; k < executed.size(); ++k) {
    paths.push_back(std::vector<core::Cell>(executed[k].begin(),
                                            executed[k].begin() + static_cast<std::ptrdiff_t>(done_at + 1)));
  }
  double cost = 0.0;
  for (size_t k = 0; k < paths.size(); ++k) {
    for (int s = 0; s <= done_at; ++s) {
      if (paths[k][static_cast<size_t>(s)] == goals[k]) {
        cost += static_cast<double>(s);
        break;
      }
    }
  }
  if (recorder != nullptr) {
    for (size_t k = 0; k < paths.size(); ++k) {
      recorder->path_found(paths[k], static_cast<int>(k));
    }
    recorder->planning_finished(true, {{"expanded_nodes", static_cast<double>(calls)},
                                       {"makespan", static_cast<double>(done_at)},
                                       {"sum_of_costs", cost}});
  }
  core::MultiPlanResult result;
  result.success = true;
  result.paths = std::move(paths);
  result.cost = cost;
  result.stats.expanded_nodes = calls;
  return result;
}

core::MultiPlanResult Rhcr::fail(core::TraceRecorder* recorder, int calls) const {
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

}  // namespace mrmp::search
