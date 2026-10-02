#include "mrmp/decentralized/winpibt.hpp"

#include <algorithm>
#include <cassert>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace mrmp::decentralized {

using core::Cell;
using core::DiscreteSpace;

// 4-connected neighbor order — the SAME fixed convention as the map layer's move set.
// The wait action is not a neighbor: staying put is the current cell itself, which the
// ideal path expresses by repeating its cell (the paper's padding with waits).
namespace {
constexpr int kMoves4[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

// Distance from an unreachable goal: sorts after every reachable cell; among equals
// the fixed row-major order still decides. IEEE-754 infinity compares identically to
// Python's float('inf') by construction.
double inf_dist() { return std::numeric_limits<double>::infinity(); }
}  // namespace

std::vector<Cell> Winpibt::nbrs(const std::set<Cell>& free_cells, const Cell& c) {
  std::vector<Cell> out;
  for (const auto& d : kMoves4) {
    Cell n{c.row + d[0], c.col + d[1]};
    if (free_cells.count(n)) out.push_back(n);
  }
  return out;
}

std::map<Cell, int> Winpibt::bfs(const std::set<Cell>& free_cells, core::Cell goal) {
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

// The paper's Algorithm 1, line for line. Note β is fixed at ENTRY (before anything
// this call registers), the whole extension registers at once but secures step by
// step, and a failed inheritance retracts only the UNSECURED suffix and replans from
// the last secured step — then retries the same step against the new route.
bool Winpibt::winpibt(Sim& sim, int i, int alpha) const {
  ++sim.calls;
  const size_t iu = static_cast<size_t>(i);
  const int ell_i = sim.ell[iu];
  if (ell_i >= alpha) return true;  // line 2: already registered beyond alpha
  // line 3: the prophetic timestep β, fixed at entry
  int beta = alpha;
  for (const auto& p : sim.paths) beta = std::max(beta, static_cast<int>(p.size()) - 1);

  std::optional<std::vector<Cell>> path = ideal_path(sim, i, ell_i, beta);
  if (!path.has_value()) {
    // lines 4-7: no valid walk — copeStuck pins waits to alpha
    const Cell v = sim.paths[iu][static_cast<size_t>(ell_i)];
    for (int s = 0; s < alpha - ell_i; ++s) sim.paths[iu].push_back(v);
    sim.ell[iu] = alpha;
    return false;
  }

  // line 8 (hard mode): register the whole ideal path up to alpha at once; only the
  // secured prefix (ℓ_i, advanced step by step below) is visible to others. The path
  // is RELATIVE-indexed: index 0 is step ell_i, so steps ell_i+1..alpha are indices
  // 1..alpha-ell_i.
  for (int s = 1; s <= alpha - ell_i; ++s) sim.paths[iu].push_back((*path)[static_cast<size_t>(s)]);

  int t = ell_i + 1;
  while (t <= alpha) {
    const Cell v = sim.paths[iu][static_cast<size_t>(t)];
    sim.ell[iu] = t;  // the secured step becomes visible right now, not before
    // lines 12-14 (retroactive): anyone parked on v with ℓ_j < t-1 extends one step at
    // a time until their path ends past t-1 or off v — return ignored.
    std::optional<int> j = target(sim, v, t - 1);
    while (j.has_value()) {
      winpibt(sim, *j, static_cast<int>(sim.ell[static_cast<size_t>(*j)]) + 1);
      j = target(sim, v, t - 1);
    }
    // lines 15-26 (the PIBT core): the occupant parked exactly at step t-1 inherits and
    // must vacate by t; failure backtracks this call's extension.
    j = target(sim, v, t);
    if (j.has_value()) {
      if (!winpibt(sim, *j, t)) {
        while (static_cast<int>(sim.paths[iu].size()) > t) sim.paths[iu].pop_back();
        sim.ell[iu] = t - 1;
        std::optional<std::vector<Cell>> replan = ideal_path(sim, i, t - 1, beta);
        if (!replan.has_value()) {  // copeStuck: pin waits at the last secured cell
          const Cell v2 = sim.paths[iu][static_cast<size_t>(t - 1)];
          for (int s = 0; s < alpha - (t - 1); ++s) sim.paths[iu].push_back(v2);
          sim.ell[iu] = alpha;
          return false;
        }
        // re-register from the last secured step (replan is relative: index 0 is step
        // t-1, so its steps t..alpha live at indices 1..alpha-t+1)
        for (int s = 1; s <= alpha - t + 1; ++s) {
          sim.paths[iu].push_back((*replan)[static_cast<size_t>(s)]);
        }
        continue;  // retry securing step t against the replanned route
      }
    }
    if (v == sim.goals[iu] && t < alpha) {
      // Hard mode with a fixed goal (the paper's classical-MAPF framing: leaving g_i
      // re-issues task {g_i}): from the secured arrival onward the tail is re-planned
      // FROM the goal, so the agent parks there as soon as waiting is disentangled
      // instead of wandering on to β. This replan can never fail (the docstring argues
      // why; the assert keeps the argument honest).
      std::optional<std::vector<Cell>> replan = ideal_path(sim, i, t, beta);
      assert(replan.has_value());
      for (int s = 1; s <= alpha - t; ++s) {
        sim.paths[iu][static_cast<size_t>(t + s)] = (*replan)[static_cast<size_t>(s)];
      }
    }
    ++t;
  }
  return true;
}

std::optional<int> Winpibt::target(const Sim& sim, const Cell& v, int t) {
  for (size_t j = 0; j < sim.paths.size(); ++j) {
    if (sim.ell[j] < t && sim.paths[j][static_cast<size_t>(sim.ell[j])] == v) {
      return static_cast<int>(j);
    }
  }
  return std::nullopt;
}

std::optional<std::vector<Cell>> Winpibt::ideal_path(const Sim& sim, int i, int t1,
                                                    int beta) const {
  const Cell start = sim.paths[static_cast<size_t>(i)][static_cast<size_t>(t1)];
  const std::map<Cell, int>& dist = sim.dists[static_cast<size_t>(i)];

  // Heap entries (f, -step, insertion order, cell): min f first; ties resolved by LATER
  // step first (the reference's heap comparator tie), then FIFO. The sequence number is
  // unique per push, so the tuple never reaches a Cell comparison — exactly Python's
  // heapq behaviour with a unique third element.
  struct Entry {
    double f;
    int neg_t;
    long seq;
    Cell cell;
  };
  struct Greater {
    bool operator()(const Entry& a, const Entry& b) const {
      return std::tie(a.f, a.neg_t, a.seq) > std::tie(b.f, b.neg_t, b.seq);
    }
  };
  std::priority_queue<Entry, std::vector<Entry>, Greater> heap;
  auto push = [&](const Entry& e) { heap.push(e); };

  long seq = 0;
  const std::pair<Cell, int> start_state{start, t1};
  std::map<std::pair<Cell, int>, std::optional<std::pair<Cell, int>>> parent{
      {start_state, std::nullopt}};
  std::set<std::pair<Cell, int>> closed;
  push(Entry{dist.count(start) ? static_cast<double>(dist.at(start)) : inf_dist(), -t1, seq++, start});

  while (!heap.empty()) {
    const Entry e = heap.top();
    heap.pop();
    const int g_t = -e.neg_t;
    const Cell v = e.cell;
    if (g_t >= beta) return chain(parent, std::make_pair(v, g_t));  // survived to the horizon
    std::optional<std::vector<Cell>> seg = static_chain(sim, i, v, g_t, beta);
    if (seg.has_value() && valid(sim, i, *seg, g_t, beta)) {
      std::vector<Cell> out = chain(parent, std::make_pair(v, g_t));
      for (size_t s = 1; s < seg->size(); ++s) out.push_back((*seg)[s]);
      return out;
    }
    closed.insert(std::make_pair(v, g_t));
    std::vector<Cell> moves = nbrs(sim.free_cells, v);
    moves.push_back(v);  // fixed order, wait last
    for (const Cell& m : moves) {
      const std::pair<Cell, int> state{m, g_t + 1};
      if (closed.count(state) || parent.count(state)) continue;
      if (!valid_step(sim, i, v, m, g_t + 1, beta)) continue;
      parent[state] = std::make_pair(v, g_t);
      push(Entry{static_cast<double>(g_t + 1) + (dist.count(m) ? static_cast<double>(dist.at(m)) : inf_dist()),
                 -(g_t + 1), seq++, m});
    }
  }
  return std::nullopt;
}

std::vector<Cell> Winpibt::chain(
    const std::map<std::pair<Cell, int>, std::optional<std::pair<Cell, int>>>& parent,
    const std::pair<Cell, int>& state) {
  std::vector<Cell> cells;
  std::optional<std::pair<Cell, int>> cur = state;
  while (cur.has_value()) {
    cells.push_back(cur->first);
    cur = parent.at(*cur);  // every chain link is in the map by construction
  }
  std::reverse(cells.begin(), cells.end());
  return cells;
}

std::optional<std::vector<Cell>> Winpibt::static_chain(const Sim& sim, int i, const Cell& start,
                                                      int t1, int beta) const {
  const std::map<Cell, int>& dist = sim.dists[static_cast<size_t>(i)];
  if (!dist.count(start)) return std::nullopt;  // unreachable goal — no ideal path exists
  std::vector<Cell> cells{start};
  Cell c = start;
  while (!(c == sim.goals[static_cast<size_t>(i)])) {
    const int d = dist.at(c);
    bool found = false;
    Cell next{};
    for (const Cell& n : nbrs(sim.free_cells, c)) {
      auto it = dist.find(n);
      if (it != dist.end() && it->second == d - 1) {
        next = n;  // fixed order decides the parent — pinned by construction
        found = true;
        break;
      }
    }
    assert(found && "a reachable non-goal cell always has a distance-decreasing neighbor");
    (void)found;  // the assert compiles out in release builds — unreachable by construction
    c = next;
    cells.push_back(c);
  }
  while (static_cast<int>(cells.size()) - 1 + t1 < beta) {
    cells.push_back(sim.goals[static_cast<size_t>(i)]);
  }
  while (static_cast<int>(cells.size()) - 1 + t1 > beta) {
    cells.pop_back();
  }
  return cells;
}

bool Winpibt::valid(const Sim& sim, int i, const std::vector<Cell>& seg, int t1, int beta) {
  for (size_t j = 1; j < seg.size(); ++j) {
    if (!valid_step(sim, i, seg[j - 1], seg[j], t1 + static_cast<int>(j), beta)) return false;
  }
  return true;
}

bool Winpibt::valid_step(const Sim& sim, int i, const Cell& v1, const Cell& v2, int tau, int beta) {
  for (size_t j = 0; j < sim.paths.size(); ++j) {
    if (static_cast<int>(j) == i) continue;
    const int ell_j = static_cast<int>(sim.ell[j]);
    if (ell_j >= tau) {
      const std::vector<Cell>& path_j = sim.paths[j];
      for (int x = tau; x <= std::min(beta, ell_j); ++x) {
        if (path_j[static_cast<size_t>(x)] == v2) return false;  // secured cell stays poison
      }
      if (path_j[static_cast<size_t>(tau)] == v1 && path_j[static_cast<size_t>(tau - 1)] == v2) {
        return false;  // swap with a fully-secured move at tau
      }
    }
  }
  return true;
}

core::MultiPlanResult Winpibt::fail(core::TraceRecorder* recorder, int calls) const {
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

core::MultiPlanResult Winpibt::plan(const DiscreteSpace& space,
                                    const std::vector<core::AgentTask>& tasks,
                                    core::TraceRecorder* recorder) {
  Sim sim;
  for (const Cell& c : space.cells()) sim.free_cells.insert(c);
  std::vector<Cell> starts, goals;
  for (const auto& t : tasks) {
    starts.push_back(t.start);
    goals.push_back(t.goal);
  }
  // The paper's instance is well-formed by definition (unique starts AND unique goals on
  // passable cells). A violating input is not an instance of this problem — report it
  // honestly, no plan.
  {
    std::set<Cell> s_starts(starts.begin(), starts.end());
    std::set<Cell> s_goals(goals.begin(), goals.end());
    bool invalid = s_starts.size() < starts.size() || s_goals.size() < goals.size();
    for (const Cell& c : starts) invalid = invalid || !sim.free_cells.count(c);
    for (const Cell& c : goals) invalid = invalid || !sim.free_cells.count(c);
    if (invalid) return fail(recorder, 0);
  }

  const int k = static_cast<int>(tasks.size());
  const int w = params_.get_int("window");
  const int max_steps = params_.get_int("max_steps");
  sim.goals = goals;
  for (const Cell& g : goals) sim.dists.push_back(bfs(sim.free_cells, g));
  for (const Cell& s : starts) sim.paths.push_back(std::vector<Cell>{s});
  sim.ell.assign(static_cast<size_t>(k), 0);
  // Pinned priorities: agent 0 highest (ε closest to 1). Distinct by construction — the
  // paper requires distinct ε and only asks ε ∈ [0,1). Integer division would be a bug;
  // this is exactly Python's true division of the same integer operands.
  for (int i = 0; i < k; ++i) {
    sim.eps.push_back(static_cast<double>(k - 1 - i) / static_cast<double>(k));
  }
  std::vector<double> p = sim.eps;  // priorities start at ε

  int t = 0;
  int kappa = 0;  // κ: cap on what lower priorities may still reserve (Algorithm 2 line 14/15)
  auto all_at_goal = [&]() {
    for (int i = 0; i < k; ++i) {
      if (!(sim.paths[static_cast<size_t>(i)][static_cast<size_t>(t)] ==
            sim.goals[static_cast<size_t>(i)])) {
        return false;
      }
    }
    return true;
  };
  while (!all_at_goal()) {
    if (t >= max_steps) {
      // Honest budget exhaustion — not a proof of unsolvability.
      return fail(recorder, sim.calls);
    }
    // Priority update (Algorithm 2 line 3): on goal → reset to ε; travelling → +1.
    for (int i = 0; i < k; ++i) {
      const size_t iu = static_cast<size_t>(i);
      p[iu] = (sim.paths[iu][static_cast<size_t>(t)] == sim.goals[iu]) ? sim.eps[iu] : p[iu] + 1.0;
    }
    // Decreasing priority; values are distinct so the order is total (stable sort keeps
    // index order for any equal pair, exactly like Python's sorted(key=-p)).
    std::vector<int> order(static_cast<size_t>(k));
    for (int i = 0; i < k; ++i) order[static_cast<size_t>(i)] = i;
    std::stable_sort(order.begin(), order.end(),
                     [&](int a, int b) { return p[static_cast<size_t>(a)] > p[static_cast<size_t>(b)]; });
    for (size_t j = 0; j < order.size(); ++j) {
      const int i = order[j];
      const size_t iu = static_cast<size_t>(i);
      if (sim.ell[iu] <= t) {  // path not yet registered beyond the current step (line 7)
        const int alpha = j == 0 ? t + w : std::min(t + w, kappa);
        winpibt(sim, i, alpha);  // top-level call — its verdict ends here
      }
      kappa = j == 0 ? static_cast<int>(sim.ell[iu]) : std::min(kappa, static_cast<int>(sim.ell[iu]));
    }
    ++t;
  }

  double cost = 0.0;
  std::vector<std::vector<Cell>> paths;
  for (size_t i = 0; i < sim.paths.size(); ++i) {
    std::vector<Cell> full(sim.paths[i].begin(), sim.paths[i].begin() + t + 1);
    for (int s = 1; s <= t; ++s) {
      if (!(full[static_cast<size_t>(s)] == full[static_cast<size_t>(s - 1)])) cost += 1.0;
    }
    paths.push_back(std::move(full));
  }
  if (recorder != nullptr) {
    for (size_t i = 0; i < paths.size(); ++i) recorder->path_found(paths[i], static_cast<int>(i));
    recorder->planning_finished(
        true, {{"expanded_nodes", static_cast<double>(sim.calls)},
               {"makespan", static_cast<double>(t)}, {"sum_of_costs", cost}});
  }
  core::MultiPlanResult result;
  result.success = true;
  result.paths = std::move(paths);
  result.cost = cost;
  result.stats.expanded_nodes = sim.calls;
  return result;
}

}  // namespace mrmp::decentralized
