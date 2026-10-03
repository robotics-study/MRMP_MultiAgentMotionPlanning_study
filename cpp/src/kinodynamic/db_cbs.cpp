#include "mrmp/kinodynamic/db_cbs.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace mrmp::kinodynamic {

using core::Cell;
using core::ConflictKind;
using core::DiscreteSpace;

namespace {

// Manhattan distance on the lattice — the constraint volume's gauge and the
// landing test's gauge, always per-axis-summed.
int manhattan(const Cell& a, const Cell& b) { return std::abs(a.row - b.row) + std::abs(a.col - b.col); }

// Low-level heap node: f = t + h with g == t, plus the push counter for a stable
// FIFO tie-break among equal f — the exact counterpart of Python's (f, seq). The
// state is the PAIR (cell, velocity) at an absolute step.
struct StateNode {
  double f;
  long long seq;
  Cell cell;
  Cell v;
  int t;
};

struct LowerStatePriority {
  bool operator()(const StateNode& a, const StateNode& b) const {
    if (a.f != b.f) return a.f > b.f;
    return a.seq > b.seq;
  }
};

}  // namespace

core::MultiPlanResult DbCbs::plan(const DiscreteSpace& space,
                                  const std::vector<core::AgentTask>& tasks,
                                  core::TraceRecorder* recorder) {
  const double delta_start = params_.get_float("delta_start");
  const double delta_end = params_.get_float("delta_end");
  const int max_ct = params_.get_int("max_ct_expansions");
  // The ladder only tightens — a widening bound is not what the paper's loop does (it
  // reduces the discontinuity each iteration, never grows it). Each parameter alone
  // validates fine; only plan() can see the pair.
  if (delta_end > delta_start) {
    throw std::runtime_error("param error: db_cbs: delta_end (" + std::to_string(delta_end) +
                             ") must not exceed delta_start (" + std::to_string(delta_start) +
                             ") — the ladder tightens, never loosens");
  }
  // The velocity lattice only quantizes integers.
  for (const auto& task : tasks) {
    if (std::floor(task.vmax) != task.vmax || task.vmax < 1.0) {
      throw std::runtime_error("param error: db_cbs: vmax must be a positive integer — "
                               "the velocity lattice quantizes integers only");
    }
  }

  // The capability stays the plain discrete grid, but this planner never reads its
  // 4-connected move model — the lattice needs only the map's vertex set (membership
  // is what matters; row-major order is irrelevant here).
  std::set<Cell> free_cells;
  for (const Cell& c : space.cells()) free_cells.insert(c);

  std::vector<double> ladder{delta_start};
  if (delta_end != delta_start) ladder.push_back(delta_end);
  int expanded = 0;
  std::optional<std::vector<std::vector<Cell>>> solved;
  double answered_delta = 0.0;
  for (const double delta : ladder) {
    auto [paths, n] = rung(free_cells, tasks, delta, max_ct, recorder);
    expanded += n;
    // This rung died (verdict or budget). Looser rungs are not easier (a looser bound
    // fattens the constraint volume too), so this stops nothing — the answer stays
    // whatever an earlier rung solved, and a ladder with no survivor is an honest
    // failure.
    if (!paths.has_value()) continue;
    solved = std::move(paths);
    answered_delta = delta;
  }

  if (!solved.has_value()) {
    // The delta an honest failure names is the tightest rung ever attempted — the
    // model whose every branch died.
    if (recorder != nullptr) {
      recorder->planning_finished(false, {{"delta", ladder.back()},
                                          {"expanded_nodes", static_cast<double>(expanded)},
                                          {"makespan", 0.0},
                                          {"sum_of_costs", 0.0}});
    }
    core::MultiPlanResult result;
    result.success = false;
    result.stats.expanded_nodes = expanded;
    return result;
  }

  double cost = 0.0;
  int makespan = 0;
  for (const auto& p : *solved) {
    cost += static_cast<double>(p.size()) - 1.0;
    makespan = std::max(makespan, static_cast<int>(p.size()) - 1);
  }
  if (recorder != nullptr) {
    for (size_t k = 0; k < solved->size(); ++k) recorder->path_found((*solved)[k], static_cast<int>(k));
    recorder->planning_finished(true, {{"delta", answered_delta},
                                       {"expanded_nodes", static_cast<double>(expanded)},
                                       {"makespan", static_cast<double>(makespan)},
                                       {"sum_of_costs", cost}});
  }
  core::MultiPlanResult result;
  result.success = true;
  result.paths = *solved;
  result.cost = cost;
  result.stats.expanded_nodes = expanded;
  return result;
}

std::pair<std::optional<std::vector<std::vector<core::Cell>>>, int> DbCbs::rung(
    const std::set<Cell>& free, const std::vector<core::AgentTask>& tasks, double delta,
    int max_ct, core::TraceRecorder* recorder) {
  const int f = static_cast<int>(std::floor(delta));  // lattice slack: <1 -> exact, >=1 that many cells
  int expanded = 0;

  // Root expansion (the first of the budget): every agent planned unconstrained, in
  // index order. If an unconstrained sub-search fails the goal is statically
  // unreachable — no constraint set can ever help, so this rung dies.
  std::vector<std::vector<Cell>> root_paths;
  for (size_t k = 0; k < tasks.size(); ++k) {
    auto [path, n] = plan_one(free, tasks[k], {}, f, static_cast<int>(k), recorder);
    expanded += n;
    if (!path.has_value()) return {std::nullopt, expanded};
    root_paths.push_back(std::move(*path));
  }

  // Best-first CT queue: cheapest sum-of-arrival-steps first, ties broken by creation
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
    const QueueEntry top = queue.top();
    queue.pop();
    ++ct_expansions;
    const double cost = top.cost;
    const CtNode& node = top.node;

    const auto conflict = first_conflict(node.paths);
    if (!conflict.has_value()) {
      // Best-first pops solutions by increasing sum-of-arrival-steps and every node's
      // paths are individually optimal under their constraints — the first
      // conflict-free node IS jointly optimal at this rung's delta. Done.
      return {node.paths, expanded};
    }
    const Cell cell = std::get<0>(*conflict);
    const int t = std::get<1>(*conflict);
    const std::pair<int, int> pair_ij = std::get<2>(*conflict);
    if (recorder != nullptr) recorder->conflict_found(ConflictKind::Vertex, cell, t, pair_ij);

    // Branch: one child per conflicting agent, in pair order. Each child forbids its
    // agent from THIS collision volume and re-plans only that agent; a child whose low
    // level fails dies and is never pushed.
    for (const int agent : {pair_ij.first, pair_ij.second}) {
      std::vector<std::vector<Constraint>> child_constraints = node.constraints;
      child_constraints[static_cast<size_t>(agent)].push_back(Constraint{cell, t});
      if (recorder != nullptr) recorder->constraint_added(agent, ConflictKind::Vertex, cell, t);
      auto [path, n] = plan_one(free, tasks[static_cast<size_t>(agent)],
                                child_constraints[static_cast<size_t>(agent)], f, agent, recorder);
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

  return {std::nullopt, expanded};
}

std::pair<std::optional<std::vector<core::Cell>>, int> DbCbs::plan_one(
    const std::set<Cell>& free, const core::AgentTask& task,
    const std::vector<Constraint>& constraints, int f, int agent, core::TraceRecorder* recorder) {
  const Cell start = task.start;
  const Cell goal = task.goal;
  const int vmax = static_cast<int>(task.vmax);
  // Being born inside a forbidden volume is not undoable — no move can erase step 0.
  // The instance itself is malformed for this rung's delta.
  for (const auto& c : constraints) {
    if (c.t == 0 && manhattan(c.cell, start) <= f) return {std::nullopt, 0};
  }

  const std::set<Cell> reach = reachable(free, start, vmax + f);
  if (!reach.count(goal)) return {std::nullopt, 0};  // statically unreachable — honest fail
  int constrained_until = 0;
  for (const auto& c : constraints) constrained_until = std::max(constrained_until, c.t);
  // Finiteness: the state space over reachable cells is exactly |reach| x (2*vmax+1)^2
  // states; any feasible continuation shortens to a SIMPLE state-path, at most that
  // many steps long. Past the last constrained step nothing binds anymore, so no plan
  // needs one step more.
  const int reach_span = (2 * vmax + 1) * (2 * vmax + 1);
  const int horizon = constrained_until + static_cast<int>(reach.size()) * reach_span;

  // Every state is pushed exactly once (g == t never improves), so a pop IS the
  // expansion. The frontier orders by (f, seq) alone — Python's heap compares the
  // float first and the unique counter second, never reaching the payload.
  std::priority_queue<StateNode, std::vector<StateNode>, LowerStatePriority> frontier;
  long long counter = 0;
  const Cell rest{0, 0};
  frontier.push(StateNode{static_cast<double>(heuristic(start, goal, vmax, f)), counter++, start, rest, 0});
  std::set<std::tuple<Cell, Cell, int>> seen{std::make_tuple(start, rest, 0)};
  std::map<std::tuple<Cell, Cell, int>, std::pair<Cell, Cell>> parent;
  int expanded = 0;

  while (!frontier.empty()) {
    const StateNode cur = frontier.top();
    frontier.pop();
    ++expanded;
    if (recorder != nullptr)
      recorder->node_expanded(cur.cell, static_cast<double>(cur.t), agent, cur.t);
    // Goal guard: after arrival the agent occupies `goal` forever, so the pop counts
    // only when NO constraint binds the goal cell at any t' >= t.
    bool guard = true;
    for (const auto& c : constraints) {
      if (c.t >= cur.t && manhattan(c.cell, goal) <= f) {
        guard = false;
        break;
      }
    }
    if (cur.cell == goal && guard) {
      std::vector<Cell> path;
      Cell cur_cell = cur.cell;
      Cell cur_v = cur.v;
      int cur_t = cur.t;
      while (true) {
        path.push_back(cur_cell);
        // The parent map stores the predecessor STATE (cell, velocity); its step is
        // exactly one lower — every transition costs one.
        const auto it = parent.find(std::make_tuple(cur_cell, cur_v, cur_t));
        if (it == parent.end()) break;
        cur_cell = it->second.first;
        cur_v = it->second.second;
        --cur_t;
      }
      std::reverse(path.begin(), path.end());  // index 0 == step 0
      return {std::move(path), expanded};
    }
    if (cur.t == horizon) continue;
    const int t2 = cur.t + 1;
    for (const auto& [succ_cell, succ_v] : successors(free, cur.cell, cur.v, vmax, f)) {
      bool blocked = false;
      for (const auto& c : constraints) {
        if (c.t == t2 && manhattan(c.cell, succ_cell) <= f) {
          blocked = true;
          break;
        }
      }
      if (blocked) continue;
      const auto key = std::make_tuple(succ_cell, succ_v, t2);
      if (!seen.insert(key).second) continue;
      parent[key] = std::make_pair(cur.cell, cur.v);
      frontier.push(StateNode{static_cast<double>(t2) + static_cast<double>(heuristic(succ_cell, goal, vmax, f)),
                              counter++, succ_cell, succ_v, t2});
    }
  }

  return {std::nullopt, expanded};
}

int DbCbs::heuristic(const Cell& cell, const Cell& goal, int vmax, int f) {
  // Steps to the goal cell can never undercut Chebyshev / (vmax + floor(f)): no legal
  // move shifts either axis by more than that.
  const int chebyshev = std::max(std::abs(cell.row - goal.row), std::abs(cell.col - goal.col));
  return static_cast<int>(std::ceil(static_cast<double>(chebyshev) / static_cast<double>(vmax + f)));
}

std::set<std::pair<core::Cell, core::Cell>> DbCbs::successors(const std::set<Cell>& free,
                                                              const Cell& cell, const Cell& v,
                                                              int vmax, int f) {
  // Every state reachable in one legal transition, in canonical sorted order (the set's
  // own row/col/v_row/v_col ordering — identical to Python's sorted()). A transition
  // (cell, v) -> (x', v') is legal iff SOME action u with |u - v| <= 1 per axis and
  // |u| <= vmax lands within Manhattan f of x' and has velocity within f of v'. At
  // f = 0 that collapses to the exact double integrator: x' = x + v' and v' itself is
  // the action.
  std::set<std::pair<Cell, Cell>> out;
  for (int ur = std::max(-vmax, v.row - 1); ur <= std::min(vmax, v.row + 1); ++ur) {
    for (int uc = std::max(-vmax, v.col - 1); uc <= std::min(vmax, v.col + 1); ++uc) {
      // Landing cells within Manhattan f of the ideal landing x+u.
      for (int dr = -f; dr <= f; ++dr) {
        for (int dc = -(f - std::abs(dr)); dc <= (f - std::abs(dr)); ++dc) {
          const Cell land{cell.row + ur + dr, cell.col + uc + dc};
          if (!free.count(land)) continue;
          // Landing states carry any velocity within f of the action.
          for (int vr = std::max(-vmax, ur - f); vr <= std::min(vmax, ur + f); ++vr) {
            for (int vc = std::max(-vmax, uc - f); vc <= std::min(vmax, uc + f); ++vc) {
              out.insert({land, Cell{vr, vc}});
            }
          }
        }
      }
    }
  }
  return out;
}

std::set<Cell> DbCbs::reachable(const std::set<Cell>& free, const Cell& start, int reach_r) {
  // Flood fill over the move relation itself: from a reached cell every passable cell
  // within PER-AXIS distance reach_r = vmax + floor(delta) is one transition's landing
  // away. At floor 0 and vmax 1 that is exactly the 8-neighborhood; wider bounds hop
  // cells (and walls — sampled point robots only ever ARE at sampled endpoints,
  // overflying is legal by construction). The result is a set: flood order never
  // matters, but FIFO stays for determinism.
  std::set<Cell> seen{start};
  std::deque<Cell> queue{start};
  while (!queue.empty()) {
    const Cell cell = queue.front();
    queue.pop_front();
    for (int dr = -reach_r; dr <= reach_r; ++dr) {
      for (int dc = -reach_r; dc <= reach_r; ++dc) {
        const Cell succ{cell.row + dr, cell.col + dc};
        if (!seen.count(succ) && free.count(succ)) {
          seen.insert(succ);
          queue.push_back(succ);
        }
      }
    }
  }
  return seen;
}

std::optional<std::tuple<core::Cell, int, std::pair<int, int>>> DbCbs::first_conflict(
    const std::vector<std::vector<core::Cell>>& paths) {
  // Earliest step with any co-presence (same cell at the same sampled step); ties by
  // (cell row, col), then pair i < j. A swap across a step is NOT a conflict here —
  // point robots sampled at integer steps pass through each other; that is what this
  // branch's per-timestep check actually guarantees.
  int horizon = 0;
  for (const auto& p : paths) horizon = std::max(horizon, static_cast<int>(p.size()) - 1);
  for (int t = 0; t <= horizon; ++t) {
    std::set<std::tuple<Cell, int, int>> candidates;
    for (size_t i = 0; i < paths.size(); ++i) {
      for (size_t j = i + 1; j < paths.size(); ++j) {
        const Cell now_i = occupied(paths[i], t);
        const Cell now_j = occupied(paths[j], t);
        if (now_i == now_j) candidates.insert(std::make_tuple(now_i, static_cast<int>(i), static_cast<int>(j)));
      }
    }
    if (!candidates.empty()) {
      const auto [cell, i, j] = *candidates.begin();
      return std::make_tuple(cell, t, std::make_pair(i, j));
    }
  }
  return std::nullopt;
}

Cell DbCbs::occupied(const std::vector<core::Cell>& path, int t) {
  // Cell occupied at step t by a finished path: after arrival the agent stays (its
  // state path simply ends).
  return static_cast<size_t>(t) < path.size() ? path[static_cast<size_t>(t)] : path.back();
}

}  // namespace mrmp::kinodynamic
