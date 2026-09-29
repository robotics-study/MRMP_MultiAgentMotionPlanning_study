#include "mrmp/search/push_and_swap.hpp"

#include <algorithm>
#include <cassert>
#include <deque>
#include <set>
#include <vector>

namespace mrmp::search {

using core::Cell;
using core::DiscreteSpace;

// 4-connected neighbor order — the SAME fixed convention as the map layer's move
// set. The wait self-loop there is not an action of this algorithm: every recorded
// assignment change is a real move. (Mirrors Python _MOVES_4 exactly.)
namespace {
constexpr int kMoves4[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
}  // namespace

std::vector<Cell> PushAndSwap::nbrs(const Sim& sim, const Cell& c) {
  std::vector<Cell> out;
  for (const auto& d : kMoves4) {
    Cell n{c.row + d[0], c.col + d[1]};
    if (sim.free_cells.count(n)) out.push_back(n);
  }
  return out;
}

std::optional<int> PushAndSwap::occupant(const Sim& sim, const Cell& c) {
  for (size_t i = 0; i < sim.A.size(); ++i) {
    if (sim.A[i] == c) return static_cast<int>(i);
  }
  return std::nullopt;
}

void PushAndSwap::apply_move(Sim& sim, std::vector<Move>* segment, int agent, Cell to) {
  const Cell frm = sim.A[static_cast<size_t>(agent)];
  // Invariants every call site constructs (asserted in debug builds, never
  // recovered from): a move actually moves, the target is free, nothing occupies it.
  assert(!(frm == to) && sim.free_cells.count(to) && !occupant(sim, to).has_value());
  sim.A[static_cast<size_t>(agent)] = to;
  sim.pi.push_back(sim.A);
  if (segment != nullptr) segment->push_back(Move{agent, frm, to});
}

PushAndSwap::BfsResult PushAndSwap::bfs_parent(Sim& sim, const Cell& start,
                                               const std::set<Cell>& blocked) const {
  BfsResult res;
  res.start = start;
  if (blocked.count(start)) return res;  // empty BFS — zero expansions counted
  res.order.push_back(start);
  std::deque<Cell> queue{start};
  while (!queue.empty()) {
    const Cell c = queue.front();
    queue.pop_front();
    for (const Cell& n : nbrs(sim, c)) {
      if (n == start || res.parent.count(n) || blocked.count(n)) continue;
      // Python's parent dict gains the key at discovery, so its insertion order IS
      // the enqueue order; here that needs an explicit append to `order` as well.
      // Forgetting either half breaks every "first empty cell in enqueue order" scan.
      res.parent[n] = c;
      res.order.push_back(n);
      queue.push_back(n);
    }
  }
  // Every enqueued cell is dequeued exactly once, so order.size() IS the number of
  // node expansions this call contributes to the metric.
  sim.expanded_nodes += static_cast<int>(res.order.size());
  return res;
}

std::optional<std::vector<Cell>> PushAndSwap::path_to(Sim& sim, const Cell& start,
                                                     const Cell& goal) const {
  BfsResult res = bfs_parent(sim, start, {});
  if (!res.has(goal)) return std::nullopt;
  std::vector<Cell> out;
  Cell cur = goal;
  for (;;) {
    out.push_back(cur);
    if (cur == start) break;
    cur = res.parent.at(cur);
  }
  std::reverse(out.begin(), out.end());
  return out;
}

bool PushAndSwap::push(Sim& sim, int r) const {
  if (sim.A[static_cast<size_t>(r)] == sim.T[static_cast<size_t>(r)]) return true;  // vacuous
  auto p_star = path_to(sim, sim.A[static_cast<size_t>(r)], sim.T[static_cast<size_t>(r)]);
  if (!p_star) return false;
  size_t idx = 1;
  while (!(sim.A[static_cast<size_t>(r)] == sim.T[static_cast<size_t>(r)])) {
    // r stands on p_star[idx-1] != T[r] == p_star.back(), so the next cell exists.
    const Cell v = (*p_star)[idx];
    if (!occupant(sim, v).has_value()) {
      apply_move(sim, nullptr, r, v);
      ++idx;
      continue;
    }
    // Blocked at v. A parked (U) blocker marks itself -> BFS cannot even start
    // there; otherwise the first empty cell in enqueue order is the hole and the
    // chain shifts toward it farthest-from-r occupant first.
    std::set<Cell> blocked = sim.U;
    blocked.insert(sim.A[static_cast<size_t>(r)]);
    if (!chain_push(sim, nullptr, v, blocked)) return false;
  }
  return true;
}

bool PushAndSwap::chain_push(Sim& sim, std::vector<Move>* segment, const Cell& blocker,
                             const std::set<Cell>& blocked) const {
  BfsResult res = bfs_parent(sim, blocker, blocked);
  std::optional<Cell> v_empty;
  for (const Cell& c : res.order) {  // enqueue order == dequeue order
    if (!occupant(sim, c).has_value()) {
      v_empty = c;
      break;
    }
  }
  if (!v_empty) return false;
  std::vector<Cell> chain;
  Cell cur = *v_empty;
  chain.push_back(cur);
  while (!(cur == res.start)) {
    cur = res.parent.at(cur);
    chain.push_back(cur);
  }
  std::reverse(chain.begin(), chain.end());  // blocker cell first, hole last
  // Farthest-from-r occupant moves first so each target cell is already empty.
  for (int k = static_cast<int>(chain.size()) - 2; k >= 0; --k) {
    auto a = occupant(sim, chain[static_cast<size_t>(k)]);
    if (a.has_value()) apply_move(sim, segment, *a, chain[static_cast<size_t>(k + 1)]);
  }
  return true;
}

bool PushAndSwap::swap(Sim& sim, int r) const {
  if (sim.A[static_cast<size_t>(r)] == sim.T[static_cast<size_t>(r)]) return true;  // vacuous
  auto p_star = path_to(sim, sim.A[static_cast<size_t>(r)], sim.T[static_cast<size_t>(r)]);
  if (!p_star || p_star->size() < 2) return false;
  auto s_opt = occupant(sim, (*p_star)[1]);
  if (!s_opt.has_value()) return false;  // nothing blocks r's first step — push would not have failed
  const int s = *s_opt;
  // Candidate swap vertices in BFS-dequeue order from A[r] (nearest first) — the
  // pinned reading of the paper's unspecified POP() order.
  BfsResult candidates = bfs_parent(sim, sim.A[static_cast<size_t>(r)], {});
  for (const Cell& v : candidates.order) {
    const std::vector<Cell> before = sim.A;
    const size_t pi_len = sim.pi.size();
    std::vector<Move> segment;
    if (!multipush(sim, &segment, r, s, v)) {
      sim.A = before;
      sim.pi.resize(pi_len);
      continue;
    }
    if (!clear_and_swap(sim, &segment, r, s)) {
      sim.A = before;
      sim.pi.resize(pi_len);
      continue;
    }
    // Success. The forward segment already landed on pi move by move; the
    // EXECUTE_SWAP moves went straight to pi too (they are NOT part of the
    // reversible segment). Replaying the segment reversed with r/s roles exchanged
    // restores every displaced agent — including U agents — home.
    for (auto it = segment.rbegin(); it != segment.rend(); ++it) {
      const int who = it->agent == r ? s : (it->agent == s ? r : it->agent);
      assert(sim.A[static_cast<size_t>(who)] == it->to);  // lands exactly by construction
      apply_move(sim, nullptr, who, it->from);
    }
    if (sim.U.count(sim.T[static_cast<size_t>(s)])) return resolve(sim, r, s);
    return true;
  }
  return false;
}

bool PushAndSwap::multipush(Sim& sim, std::vector<Move>* segment, int r, int s,
                            const Cell& v) const {
  auto p = path_to(sim, sim.A[static_cast<size_t>(r)], v);  // starts at A[r] — r always leads
  if (!p || p->size() < 2) return false;
  for (const Cell& c : *p) {
    if (c == sim.A[static_cast<size_t>(s)]) {
      // s sits on the path ahead of r: the composite can never walk to v without
      // dragging s past r, so this candidate simply fails.
      return false;
    }
  }
  const int lead = r, follow = s;
  {
    bool adjacent = false;
    for (const Cell& n : nbrs(sim, sim.A[static_cast<size_t>(lead)])) {
      if (n == sim.A[static_cast<size_t>(follow)]) adjacent = true;
    }
    if (!adjacent) return false;  // defensive: by construction the pair IS adjacent
  }
  size_t i = 0;  // p starts at A[lead], so lead's index on p is 0
  while (!(sim.A[static_cast<size_t>(lead)] == v)) {
    const Cell w = (*p)[i + 1];
    auto a = occupant(sim, w);
    if (a.has_value() && *a != follow) {
      const std::set<Cell> blocked{sim.A[static_cast<size_t>(lead)], sim.A[static_cast<size_t>(follow)]};
      if (!chain_push(sim, segment, w, blocked)) return false;
    }
    apply_move(sim, segment, lead, w);
    ++i;
    const Cell vacated = (*p)[i - 1];  // lead just left it — adjacent to follow by construction
    if (!occupant(sim, vacated).has_value()) apply_move(sim, segment, follow, vacated);
  }
  if (!(sim.A[static_cast<size_t>(r)] == v)) return false;
  for (const Cell& n : nbrs(sim, sim.A[static_cast<size_t>(r)])) {
    if (n == sim.A[static_cast<size_t>(s)]) return true;
  }
  return false;
}

bool PushAndSwap::clear_and_swap(Sim& sim, std::vector<Move>* segment, int r, int s) const {
  const Cell v = sim.A[static_cast<size_t>(r)];
  const Cell w1 = sim.A[static_cast<size_t>(s)];
  const std::vector<Cell> nbrs_v = nbrs(sim, v);
  bool adjacent = false;
  for (const Cell& n : nbrs_v) {
    if (n == w1) adjacent = true;
  }
  if (!adjacent) return false;
  for (const Cell& w2 : nbrs_v) {
    if (w2 == w1) continue;
    if (occupant(sim, w2).has_value() && !clear_cell(sim, segment, w2)) continue;
    for (const Cell& w4 : nbrs(sim, w1)) {
      if (w4 == v) continue;
      bool w4_shares_neighbor = false;
      for (const Cell& n : nbrs(sim, w2)) {
        if (n == w4) w4_shares_neighbor = true;
      }
      if (!w4_shares_neighbor) continue;
      if (occupant(sim, w4).has_value() && !clear_cell(sim, segment, w4)) continue;
      // EXECUTE_SWAP: r vacates v for s, then rounds the block into w1.
      apply_move(sim, nullptr, r, w2);
      apply_move(sim, nullptr, s, v);
      apply_move(sim, nullptr, r, w4);
      apply_move(sim, nullptr, r, w1);
      return true;
    }
  }
  return false;
}

bool PushAndSwap::clear_cell(Sim& sim, std::vector<Move>* segment, const Cell& x) const {
  for (const Cell& n : nbrs(sim, x)) {
    if (!occupant(sim, n).has_value()) {
      auto a = occupant(sim, x);
      assert(a.has_value());  // callers only ever call this on occupied cells
      apply_move(sim, segment, *a, n);
      return true;
    }
  }
  return false;
}

bool PushAndSwap::resolve(Sim& sim, int r, int s) const {
  while (sim.A[static_cast<size_t>(r)] == sim.T[static_cast<size_t>(s)]) {
    if (!push(sim, r) && !swap(sim, r)) return false;
  }
  while (!(sim.A[static_cast<size_t>(s)] == sim.T[static_cast<size_t>(s)])) {
    if (!push(sim, s) && !swap(sim, s)) return false;
  }
  return true;
}

core::MultiPlanResult PushAndSwap::fail(core::TraceRecorder* recorder, int expanded) const {
  // Honest failure (the prioritized branch's convention): no paths, 0 cost. BFS
  // expansions counted up to the failure are still reported.
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

core::MultiPlanResult PushAndSwap::plan(const DiscreteSpace& space,
                                        const std::vector<core::AgentTask>& tasks,
                                        core::TraceRecorder* recorder) {
  Sim sim;
  for (const Cell& c : space.cells()) sim.free_cells.insert(c);
  std::vector<Cell> starts, goals;
  for (const auto& t : tasks) {
    starts.push_back(t.start);
    goals.push_back(t.goal);
  }
  // The paper's assignment is injective by definition (unique starts AND unique
  // targets) and every involved cell must be passable. A violating input is not an
  // instance of this problem — report it honestly, no plan.
  {
    std::set<Cell> s_starts(starts.begin(), starts.end());
    std::set<Cell> s_goals(goals.begin(), goals.end());
    bool invalid = s_starts.size() < starts.size() || s_goals.size() < goals.size();
    for (const Cell& c : starts) invalid = invalid || !sim.free_cells.count(c);
    for (const Cell& c : goals) invalid = invalid || !sim.free_cells.count(c);
    if (invalid) return fail(recorder, 0);
  }
  sim.A = starts;
  sim.T = goals;
  sim.pi.push_back(starts);

  // Algorithm 1: agents in index order; PUSH first, SWAP only when pushing cannot
  // clear the way, honest failure when neither works.
  for (size_t r = 0; r < tasks.size(); ++r) {
    while (!(sim.A[static_cast<size_t>(r)] == sim.T[static_cast<size_t>(r)])) {
      if (!push(sim, static_cast<int>(r)) && !swap(sim, static_cast<int>(r))) {
        return fail(recorder, sim.expanded_nodes);
      }
    }
    sim.U.insert(sim.T[static_cast<size_t>(r)]);
  }

  // Full-horizon space-time paths: paths[k][t] is the cell agent k occupies at
  // global step t. U agents get displaced mid-later-agent swaps and ride the
  // replayed segment back home — a trimmed path would misrepresent that.
  std::vector<std::vector<Cell>> paths(tasks.size());
  double cost = 0.0;
  for (size_t k = 0; k < tasks.size(); ++k) {
    for (size_t t = 0; t < sim.pi.size(); ++t) {
      if (t > 0 && !(sim.pi[t][k] == sim.pi[t - 1][k])) cost += 1.0;
      paths[k].push_back(sim.pi[t][k]);
    }
  }
  if (recorder != nullptr) {
    for (size_t k = 0; k < paths.size(); ++k) recorder->path_found(paths[k], static_cast<int>(k));
    recorder->planning_finished(true, {
      {"expanded_nodes", static_cast<double>(sim.expanded_nodes)},
      {"makespan", static_cast<double>(sim.pi.size() - 1)},
      {"sum_of_costs", cost},
    });
  }
  core::MultiPlanResult result;
  result.success = true;
  result.paths = std::move(paths);
  result.cost = cost;
  result.stats.expanded_nodes = sim.expanded_nodes;
  return result;
}

}  // namespace mrmp::search
