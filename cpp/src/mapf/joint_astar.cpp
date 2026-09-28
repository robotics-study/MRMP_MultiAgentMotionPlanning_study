#include "mrmp/mapf/joint_astar.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <utility>

namespace mrmp::mapf {

using core::Cell;
using core::DiscreteSpace;

namespace {

// The joint state: one Cell per agent, ordered by agent index. std::vector's
// lexicographic operator< (via Cell::operator<) gives the total order the
// ordered containers need — a hash-free, deterministic key everywhere.
using State = std::vector<Cell>;

// Heap node: f = g + h with g the accumulated sum-of-costs, plus the push
// counter for a stable FIFO tie-break among equal f — the exact counterpart of
// Python's (f, seq) tuples (counter starts at 1; unique seq means the state is
// never compared by the heap).
struct Node {
  double f;
  long long seq;
  State state;
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

core::MultiPlanResult JointAStar::plan(const DiscreteSpace& space,
                                       const std::vector<core::AgentTask>& tasks,
                                       core::TraceRecorder* recorder) {
  State start, goals;
  for (const auto& task : tasks) {
    start.push_back(task.start);
    goals.push_back(task.goal);
  }

  // Two agents sharing a cell at t=0 is an unsolvable instance — the joint
  // state space contains no conflict-free state that could ever separate them.
  for (size_t i = 0; i < start.size(); ++i) {
    for (size_t j = i + 1; j < start.size(); ++j) {
      if (start[i] == start[j]) {
        if (recorder != nullptr) {
          recorder->planning_finished(false, {{"expanded_nodes", 0.0}, {"makespan", 0.0},
                                              {"sum_of_costs", 0.0}});
        }
        core::MultiPlanResult result;
        result.stats.expanded_nodes = 0;
        return result;
      }
    }
  }

  // Sum of per-agent Manhattan distances — admissible (each unarrived agent
  // needs at least that many steps, each paying 1) and consistent (one joint
  // step moves any agent at most one cell).
  auto h = [&](const State& s) {
    double total = 0.0;
    for (size_t k = 0; k < s.size(); ++k) total += space.heuristic(s[k], goals[k]);
    return total;
  };

  std::priority_queue<Node, std::vector<Node>, LowerPriority> frontier;
  long long counter = 1;  // Python's itertools.count(1): the start node gets seq 1.
  frontier.push({h(start), counter++, start});
  std::map<State, int> g_score{{start, 0}};
  std::map<State, State> parent;
  std::set<State> closed;
  int expanded = 0;

  while (!frontier.empty()) {
    const Node cur = frontier.top();
    frontier.pop();
    if (closed.count(cur.state)) continue;  // stale entry — not an expansion.
    closed.insert(cur.state);
    ++expanded;
    // Every popped state was pushed with a g — at() makes that invariant loud
    // (Python's dict lookup raises KeyError on absence too).
    const int g_now = g_score.at(cur.state);
    if (recorder != nullptr)
      recorder->node_expanded(core::flatten(cur.state), static_cast<double>(g_now));

    if (cur.state == goals) {
      // Walk the parent chain goal-state -> start, reverse: chain[t] is the
      // joint position at step t. Agent k's path runs to its FIRST arrival —
      // after that it pinned in place anyway.
      std::vector<State> chain{cur.state};
      while (!(chain.back() == start)) chain.push_back(parent[chain.back()]);
      std::reverse(chain.begin(), chain.end());
      std::vector<std::vector<Cell>> paths(tasks.size());
      for (size_t k = 0; k < tasks.size(); ++k) {
        for (size_t t = 0; t < chain.size(); ++t) {
          if (chain[t][k] == goals[k]) {
            for (size_t s = 0; s <= t; ++s) paths[k].push_back(chain[s][k]);
            break;
          }
        }
      }
      double cost = 0.0;
      int makespan = 0;
      for (const auto& p : paths) {
        cost += static_cast<double>(p.size() - 1);
        makespan = std::max(makespan, static_cast<int>(p.size()) - 1);
      }
      if (recorder != nullptr) {
        for (size_t k = 0; k < paths.size(); ++k)
          recorder->path_found(paths[k], static_cast<int>(k));
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

    // One joint step: every unarrived agent takes exactly one action (a move or
    // the wait self-loop, each costing its one time step); arrived agents pin in
    // place and pay nothing. The step costs one per still-unarrived agent.
    int step_cost = 0;
    std::vector<std::vector<Cell>> actions;
    for (size_t k = 0; k < cur.state.size(); ++k) {
      if (cur.state[k] == goals[k]) {
        actions.push_back({cur.state[k]});  // arrived: self-loop only, pays nothing.
      } else {
        ++step_cost;
        std::vector<Cell> moves;
        for (const auto& [succ, cost] : space.neighbors(cur.state[k])) {
          (void)cost;
          moves.push_back(succ);
        }
        actions.push_back(std::move(moves));
      }
    }

    // Enumerate the cartesian product in agent-index order with the LAST agent
    // varying fastest — exactly itertools.product's output order. The odometer
    // index `idx` counts in a mixed-radix system whose digits are the agents;
    // zero agents means one empty combination, handled by the loop shape.
    std::vector<size_t> idx(actions.size(), 0);
    for (bool more = true; more;) {
      State succ;
      for (size_t k = 0; k < actions.size(); ++k) succ.push_back(actions[k][idx[k]]);

      // Vertex conflict: two agents on one cell at the new step. Edge conflict:
      // a pair swaps cells across this step. Either kills the combination.
      bool vertex = false, edge = false;
      for (size_t i = 0; i < succ.size() && !vertex; ++i)
        for (size_t j = i + 1; j < succ.size(); ++j)
          if (succ[i] == succ[j]) { vertex = true; break; }
      for (size_t i = 0; i < succ.size() && !edge; ++i)
        for (size_t j = i + 1; j < succ.size(); ++j)
          if (succ[i] == cur.state[j] && cur.state[i] == succ[j]) { edge = true; break; }

      if (!vertex && !edge) {
        const int g2 = g_now + step_cost;
        auto it = g_score.find(succ);
        // No improvement — with a consistent heuristic the state was (or will
        // be) settled at a better g; skip the push.
        if (it == g_score.end() || g2 < it->second) {
          parent[succ] = cur.state;
          g_score[succ] = g2;
          frontier.push({static_cast<double>(g2) + h(succ), counter++, succ});
        }
      }

      // Odometer tick: last agent's digit increments fastest, carrying leftward;
      // a carry past the first agent means every combination was processed.
      bool advanced = false;
      for (size_t k = idx.size(); k > 0;) {
        --k;
        if (idx[k] + 1 < actions[k].size()) {
          ++idx[k];
          advanced = true;
          break;
        }
        idx[k] = 0;
      }
      more = advanced;
    }
  }

  // Frontier exhausted: no joint plan exists (complete search — this is a
  // verdict on the instance, not on any priority order). Report honestly.
  if (recorder != nullptr) {
    recorder->planning_finished(false, {{"expanded_nodes", static_cast<double>(expanded)},
                                        {"makespan", 0.0},
                                        {"sum_of_costs", 0.0}});
  }
  core::MultiPlanResult result;
  result.stats.expanded_nodes = expanded;
  return result;
}

}  // namespace mrmp::mapf
