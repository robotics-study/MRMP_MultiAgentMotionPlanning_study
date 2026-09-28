// sRRT — the C++ mirror of python/mrmp/sampling/srrt.py, line for line. The
// Python docstring carries the full paper mapping (Wagner, Kang & Choset 2012;
// framework Wagner & Choset 2015); this file repeats only what a C++ reader
// needs. Cross-language bit-identity rests on: int64-exact MINSTD Lehmer PRNG
// and integer-only decisions everywhere else (Manhattan sums, cell equality) —
// no float arithmetic participates in any decision except the rng draw itself.
#include "mrmp/sampling/srrt.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <map>
#include <set>
#include <utility>

namespace mrmp::sampling {

using core::Cell;
using core::DiscreteSpace;

namespace {

// The joint state: one Cell per agent, ordered by agent index (std::vector's
// lexicographic operator< via Cell::operator< keys the ordered index map).
using State = std::vector<Cell>;

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

// Manhattan distance over integer cells — the same metric DiscreteSpace
// advertises; integer arithmetic, so identical in every language.
int manhattan(const Cell& a, const Cell& b) {
  return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

}  // namespace

core::MultiPlanResult Srrt::plan(const DiscreteSpace& space,
                                 const std::vector<core::AgentTask>& tasks,
                                 core::TraceRecorder* recorder) {
  const int seed = params_.get_int("seed");
  const double p_goal_prob = params_.get_float("goal_sampling_probability");
  const int max_iterations = params_.get_int("max_iterations");

  State goals, start;
  for (const auto& task : tasks) {
    start.push_back(task.start);
    goals.push_back(task.goal);
  }
  const size_t k = tasks.size();

  // Two agents sharing a cell at t=0 is an unsolvable instance — no
  // conflict-free joint trajectory can ever separate them (the same honest
  // verdict every other planner here gives).
  for (size_t i = 0; i < k; ++i) {
    for (size_t j = i + 1; j < k; ++j) {
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

  Lehmer rng{static_cast<std::int64_t>(seed)};
  const std::vector<Cell> cells = space.cells();  // the motion graph's vertex set

  // Passable 4-connected moves in fixed order (up/down/left/right), then the
  // wait self-loop — identical contract to DiscreteSpace.
  auto neighbors = [](const DiscreteSpace& s, const Cell& c) {
    std::vector<Cell> out;
    for (const auto& [n, cost] : s.neighbors(c)) {
      (void)cost;
      out.push_back(n);
    }
    return out;
  };

  // --- individual policies (§II) ---------------------------------------------
  // φᵢ = BFS tree grown backward from q_F^i over free cells. parent_i[c] is the
  // next step of the individually-optimal path from c to the goal; the goal's
  // own parent is itself (wait at one's own goal — the framework paper's "loop
  // at the goal state"). Fixed neighbor order = deterministic BFS tie-break,
  // identical in every language. A start cell missing from its agent's policy
  // tree means the goal is unreachable from that start (BFS covered the whole
  // component), so no joint solution exists — an instance verdict, not a budget
  // failure.
  std::vector<std::map<Cell, Cell>> policies;
  for (size_t i = 0; i < k; ++i) {
    std::map<Cell, Cell> tree{{goals[i], goals[i]}};
    std::vector<Cell> frontier{goals[i]};
    while (!frontier.empty()) {
      std::vector<Cell> nxt_frontier;
      for (const Cell& c : frontier) {
        for (const Cell& n : neighbors(space, c)) {
          if (!tree.count(n)) {  // fixed order ⇒ first parent wins (BFS tie-break)
            tree[n] = c;
            nxt_frontier.push_back(n);
          }
        }
      }
      frontier = nxt_frontier;
    }
    if (!tree.count(start[i])) {
      if (recorder != nullptr) {
        recorder->planning_finished(false, {{"expanded_nodes", 0.0}, {"makespan", 0.0},
                                            {"sum_of_costs", 0.0}});
      }
      core::MultiPlanResult result;
      result.stats.expanded_nodes = 0;
      return result;
    }
    policies.push_back(std::move(tree));
  }

  // Σ_i Manhattan — the joint distance NEAREST runs on, summed in agent order
  // so every language sums identical integers.
  auto dist_joint = [&](const State& a, const State& b) {
    int total = 0;
    for (size_t i = 0; i < k; ++i) total += manhattan(a[i], b[i]);
    return total;
  };

  // --- joint tree T_f ---------------------------------------------------------
  // Parallel containers indexed by insertion order (insertion order IS the
  // NEAREST tie-break). Root = start tuple, collision set ∅. Each node also
  // stores its local-path segment (per-agent cells, both endpoints included) so
  // the result chain can be concatenated without re-walking anything.
  std::vector<State> states{start};
  std::vector<int> parents_joint{-1};
  std::vector<std::set<int>> colsets{std::set<int>{}};
  std::vector<std::vector<Cell>> root_seg;
  for (const Cell& c : start) root_seg.push_back(std::vector<Cell>{c});
  std::vector<std::vector<std::vector<Cell>>> segments{root_seg};
  std::map<State, int> index_of{{start, 0}};

  bool goal_found = (start == goals);
  int goal_index = goal_found ? 0 : -1;

  if (recorder != nullptr) recorder->node_expanded(core::flatten(start));

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    if (goal_found) {
      // The tree already contains the goal tuple — stop growing (sRRT has no
      // anytime refinement to keep running for).
      break;
    }
    // --- SAMPLE: with probability p_goal the goal tuple itself (the repo's
    // informed-sampling convention); otherwise one waypoint per agent, in agent
    // order, uniform over the vertex set.
    State sample;
    if (rng.uniform() < p_goal_prob) {
      sample = goals;
    } else {
      for (size_t i = 0; i < k; ++i) {
        sample.push_back(cells[static_cast<size_t>(
            rng.uniform() * static_cast<double>(cells.size()))]);
      }
    }

    // --- NEAREST over the whole tree; ties break to the lowest index.
    size_t nearest_idx = 0;
    int nearest_d = dist_joint(states[0], sample);
    for (size_t idx = 1; idx < states.size(); ++idx) {
      const int d = dist_joint(states[idx], sample);
      if (d < nearest_d) {
        nearest_d = d;
        nearest_idx = idx;
      }
    }

    // --- LOCAL PLANNER (formula (2) + the lockstep walk). Pinned robots take
    // exactly their policy step; free robots greedy-step toward their sampled
    // cell and wait on arrival. A robot stuck at a Manhattan local minimum of
    // its target can never arrive — that expansion honestly fails instead of
    // looping forever (a strict-min step strictly decreases the distance, so a
    // walk without such a step is already at its target).
    std::set<int> colset = colsets[nearest_idx];
    State targets;
    for (size_t i = 0; i < k; ++i) {
      // The lookup always hits: every cell the walk can ever reach is in the
      // same free component as the agent's goal, and the BFS tree covers it.
      targets.push_back(colset.count(static_cast<int>(i)) == 0
                            ? policies[i].find(states[nearest_idx][i])->second
                            : sample[i]);
    }
    State cur = states[nearest_idx];
    std::vector<std::vector<Cell>> seg;  // per-agent cells, both endpoints
    for (const Cell& c : cur) seg.push_back(std::vector<Cell>{c});
    bool aborted = false;
    while (true) {
      bool all_arrived = true;
      for (size_t i = 0; i < k; ++i) {
        if (!(cur[i] == targets[i])) all_arrived = false;
      }
      if (all_arrived) break;
      const State prev = cur;
      std::vector<Cell> nxt_list;
      bool stuck = false;
      for (size_t i = 0; i < k && !stuck; ++i) {
        if (cur[i] == targets[i]) {
          nxt_list.push_back(cur[i]);  // arrived — the lockstep step is a wait
          continue;
        }
        Cell best_child{0, 0};
        int best_d = std::numeric_limits<int>::max();
        for (const Cell& child : neighbors(space, prev[i])) {
          const int d = manhattan(child, targets[i]);
          if (d < best_d) {  // first strict min = fixed-order tie-break
            best_d = d;
            best_child = child;
          }
        }
        // The wait self-loop is always a candidate, so best_child is always set.
        if (best_child == prev[i]) {
          stuck = true;  // local minimum: this target is unreachable here
          break;
        }
        nxt_list.push_back(best_child);
      }
      if (stuck) {
        aborted = true;
        break;
      }
      const State nxt = nxt_list;
      // Collision check on this step (vertex + swap), per agent pair.
      std::set<int> involved;
      for (size_t i = 0; i < k; ++i) {
        for (size_t j = i + 1; j < k; ++j) {
          if (nxt[i] == nxt[j] || (nxt[i] == prev[j] && prev[i] == nxt[j])) {
            involved.insert(static_cast<int>(i));
            involved.insert(static_cast<int>(j));
          }
        }
      }
      if (!involved.empty()) {
        // ANY collision on the local path kills this expansion (the paper: a
        // node is added only "if no collisions are found"). An INFORMATIVE one
        // (some involved robot not yet in C_r) additionally enlarges the set:
        // every involved robot joins C_r and back-propagates up the chain until
        // an ancestor already lists it (that ancestor's ancestors list it too
        // by induction); the next sample re-expands with enlarged sets. A
        // conflict between robots both already free carries no information:
        // nothing updates, the expansion just fails.
        bool informative = false;
        for (int robot : involved) {
          if (!colset.count(robot)) informative = true;
        }
        if (informative) {
          std::set<int> new_set = colset;
          for (int robot : involved) new_set.insert(robot);
          colsets[nearest_idx] = new_set;
          colset = new_set;
          int node = parents_joint[nearest_idx];
          while (node != -1) {
            bool covers_all = true;
            for (int robot : involved) {
              if (!colsets[static_cast<size_t>(node)].count(robot)) covers_all = false;
            }
            if (covers_all) break;
            std::set<int> updated = colsets[static_cast<size_t>(node)];
            for (int robot : involved) updated.insert(robot);
            colsets[static_cast<size_t>(node)] = updated;
            node = parents_joint[static_cast<size_t>(node)];
          }
        }
        aborted = true;
        break;
      }
      for (size_t i = 0; i < k; ++i) seg[i].push_back(nxt[i]);
      cur = nxt;
    }
    if (aborted) continue;

    const State new_state = cur;
    if (index_of.count(new_state)) continue;  // duplicate vertex — no-op (keeps T_f a tree)
    const int new_idx = static_cast<int>(states.size());
    states.push_back(new_state);
    parents_joint.push_back(static_cast<int>(nearest_idx));
    colsets.push_back(colset);  // the nearest node's post-update set, unchanged
    segments.push_back(std::move(seg));
    index_of[new_state] = new_idx;
    if (recorder != nullptr) recorder->node_expanded(core::flatten(new_state));

    // Goal check on the new vertex (per-coordinate equality — the joint goal
    // tuple is reached exactly).
    if (!goal_found && new_state == goals) {
      goal_found = true;
      goal_index = new_idx;
    }
  }

  // --- result -----------------------------------------------------------------
  if (!goal_found) {
    // Budget exhausted before the goal tuple ever became a vertex. An honest
    // "no solution found within budget" — sRRT's completeness is unproven (the
    // paper says so); this is not an instance verdict.
    if (recorder != nullptr) {
      recorder->planning_finished(false, {{"expanded_nodes", static_cast<double>(states.size())},
                                          {"makespan", 0.0},
                                          {"sum_of_costs", 0.0}});
    }
    core::MultiPlanResult result;
    result.stats.expanded_nodes = static_cast<int>(states.size());
    return result;
  }

  // Chain root → goal node concatenated per agent (each segment's head is the
  // previous segment's tail — skip duplicated heads), then trim each agent's
  // path after its LAST move: trailing steps are waits at the agent's own goal,
  // so trimming cannot change any cost or any other agent's motion.
  std::vector<int> chain_indices;
  for (int node = goal_index; node != -1; node = parents_joint[static_cast<size_t>(node)]) {
    chain_indices.push_back(node);
  }
  std::reverse(chain_indices.begin(), chain_indices.end());
  std::vector<std::vector<Cell>> paths(k);
  for (int node : chain_indices) {
    const auto& seg = segments[static_cast<size_t>(node)];
    for (size_t i = 0; i < k; ++i) {
      const auto& cells_of_agent = seg[i];
      // The segment includes both endpoints; the head duplicates what the
      // previous segment already ended on.
      size_t from = paths[i].empty() ? 0 : 1;
      for (size_t t = from; t < cells_of_agent.size(); ++t) {
        paths[i].push_back(cells_of_agent[t]);
      }
    }
  }
  std::vector<std::vector<Cell>> trimmed(k);
  double cost = 0.0;
  for (size_t i = 0; i < k; ++i) {
    size_t last_move = 0;
    for (size_t t = 1; t < paths[i].size(); ++t) {
      if (!(paths[i][t] == paths[i][t - 1])) last_move = t;
    }
    trimmed[i].assign(paths[i].begin(),
                      paths[i].begin() + static_cast<long>(last_move) + 1);
    // Per-agent cost: every action costs one time step except waiting at one's
    // own goal (the same metric every planner here reports).
    for (size_t t = 1; t < trimmed[i].size(); ++t) {
      if (!(trimmed[i][t] == trimmed[i][t - 1] && trimmed[i][t] == goals[i])) cost += 1.0;
    }
  }

  int makespan = 0;
  for (const auto& p : trimmed) {
    makespan = std::max(makespan, static_cast<int>(p.size()) - 1);
  }
  if (recorder != nullptr) {
    for (size_t i = 0; i < trimmed.size(); ++i) {
      recorder->path_found(trimmed[i], static_cast<int>(i));
    }
    recorder->planning_finished(true, {{"expanded_nodes", static_cast<double>(states.size())},
                                       {"makespan", static_cast<double>(makespan)},
                                       {"sum_of_costs", cost}});
  }
  core::MultiPlanResult result;
  result.success = true;
  result.paths = std::move(trimmed);
  result.cost = cost;
  result.stats.expanded_nodes = static_cast<int>(states.size());
  return result;
}

}  // namespace mrmp::sampling
