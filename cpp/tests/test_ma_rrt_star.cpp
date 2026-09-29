// MA-RRT*: seeded determinism (the PRNG is part of the algorithm's identity),
// valid conflict-free joint paths on every scenario the sampler can solve within
// budget, honest sub-optimality where finite budgets cannot converge, honest
// failure when the budget dies before the goal tuple becomes a tree vertex, and
// the trace shape of tree growth (joint-state node_expanded without agent/t).
// Mirrors python/tests/test_ma_rrt_star.py case for case.

#include <gtest/gtest.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include "mrmp/core/params.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/maps/loader.hpp"
#include "mrmp/maps/occupancy_grid.hpp"
#include "mrmp/sampling/ma_rrt_star.hpp"
#include "test_util.hpp"

using namespace mrmp;

namespace {

// Stay-at-goal occupancy of a finished path (same rule as the planner).
core::Cell occ(const std::vector<core::Cell>& p, int t) {
  return static_cast<size_t>(t) < p.size() ? p[static_cast<size_t>(t)] : p.back();
}

// No vertex conflict and no swap at any step, across every agent pair.
void assert_joint_valid(const std::vector<std::vector<core::Cell>>& paths) {
  int horizon = 0;
  for (const auto& p : paths) horizon = std::max(horizon, static_cast<int>(p.size()) - 1);
  for (size_t i = 0; i < paths.size(); ++i) {
    for (size_t j = i + 1; j < paths.size(); ++j) {
      for (int t = 1; t <= horizon; ++t) {
        core::Cell a_now = occ(paths[i], t), a_prev = occ(paths[i], t - 1);
        core::Cell b_now = occ(paths[j], t), b_prev = occ(paths[j], t - 1);
        ASSERT_TRUE(!(a_now == b_now)) << "vertex conflict at step " << t;
        ASSERT_TRUE(!(a_now == b_prev && a_prev == b_now)) << "swap at step " << t;
      }
    }
  }
}

core::ParamSet config() {
  return core::ParamSet::from_yaml(test::repo_path("configs/sampling/ma_rrt_star.yaml"));
}

// The declared ma_rrt_star config with the seed default overridden (same
// declarations, different default — the loader validates it like the real file).
core::ParamSet seed_config(int seed) {
  return core::ParamSet::from_yaml(test::write_temp(
      "ma_rrt_star_seed.yaml",
      "algorithm: ma_rrt_star\nsection: sampling\nscenarios: []\nparams:\n"
      "  - name: seed\n    type: int\n    default: " +
          std::to_string(seed) +
          "\n    min: 1\n    max: 2147483646\n    description: test seed\n"
      "  - name: gamma\n    type: float\n    default: 5.0\n    min: 0.1\n"
      "    description: test gamma\n"
      "  - name: goal_sampling_probability\n    type: float\n    default: 0.5\n    min: 0.0\n"
      "    max: 1.0\n    description: test bias\n"
      "  - name: greedy_cost_budget\n    type: int\n    default: 200\n    min: 1\n"
      "    description: test budget\n"
      "  - name: max_iterations\n    type: int\n    default: 300\n    min: 1\n"
      "    description: test iterations\n"));
}

// Load a repo scenario and convert its world-coord endpoints to cells (exactly
// what the demo driver does), so both languages solve identical tasks.
struct ScenarioSampling {
  maps::OccupancyGrid2D grid;
  std::vector<core::AgentTask> tasks;
};
ScenarioSampling scenario_sampling(const std::string& name) {
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/" + name + ".yaml"));
  auto base = maps::load_map(sc.map_path);
  auto* grid = dynamic_cast<maps::OccupancyGrid2D*>(base.get());
  if (grid == nullptr) throw std::runtime_error("scenario map is not an occupancy grid");
  // Convert tasks BEFORE moving the grid out of base.
  std::vector<core::AgentTask> tasks;
  for (const auto& a : sc.agents) {
    tasks.push_back(core::AgentTask{grid->world_to_cell(a.start.x, a.start.y),
                                    grid->world_to_cell(a.goal.x, a.goal.y)});
  }
  return ScenarioSampling{std::move(*grid), std::move(tasks)};
}

// Count the numbers inside the "state":[...] array of a node_expanded line
// (a flattened joint state of k agents has exactly 2k numbers).
int state_len(const std::string& line) {
  size_t open = line.find("\"state\":[");
  if (open == std::string::npos) return -1;
  size_t close = line.find(']', open);
  int commas = 0;
  for (size_t i = open + 9; i < close; ++i) {
    if (line[i] == ',') ++commas;
  }
  return commas + 1;
}

}  // namespace

TEST(MaRrtStar, ContractMatchesConfig) {
  sampling::MaRrtStar planner(config());
  EXPECT_EQ(planner.name(), config().algorithm());
  EXPECT_TRUE(planner.required_capabilities().count(core::Capability::DISCRETE_SPACE) > 0);
  EXPECT_EQ(planner.required_capabilities().size(), 1u);
}

TEST(MaRrtStar, SingleAgentConvergesToTheManhattanOptimalOnEverySeed) {
  // One agent on an open grid: every greedy segment is a Manhattan-shortest
  // straight walk, so the first goal-biased sample already yields cost 8 and
  // informed pruning (cost + admissible bound > incumbent) freezes the tree at
  // that optimum — seed-independence IS the convergence property here.
  auto grid = test::make_grid({".....", ".....", ".....", ".....", "....."});
  std::vector<core::AgentTask> tasks{{core::Cell{4, 0}, core::Cell{0, 4}}};
  for (int seed : {42, 7, 123}) {
    sampling::MaRrtStar planner(seed_config(seed));
    core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
    ASSERT_TRUE(r.success) << "seed " << seed;
    EXPECT_DOUBLE_EQ(r.cost, 8.0);
    ASSERT_EQ(r.paths.size(), 1u);
    EXPECT_TRUE((r.paths[0].front() == core::Cell{4, 0}));
    EXPECT_TRUE((r.paths[0].back() == core::Cell{0, 4}));
  }
}

TEST(MaRrtStar, SameSeedReplaysByteIdenticalPlans) {
  // The MINSTD stream is part of the algorithm's identity: all three language
  // engines must replay identical trees from one seed. Same-seed reruns are the
  // first line of that contract.
  ScenarioSampling s = scenario_sampling("open01_swap");
  core::MultiPlanResult a = sampling::MaRrtStar(config()).plan(s.grid, s.tasks, nullptr);
  core::MultiPlanResult b = sampling::MaRrtStar(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(a.success);
  ASSERT_TRUE(b.success);
  EXPECT_EQ(a.paths, b.paths);
  EXPECT_DOUBLE_EQ(a.cost, b.cost);
  EXPECT_EQ(a.stats.expanded_nodes, b.stats.expanded_nodes);
}

TEST(MaRrtStar, Open01CrossHitsTheJointOptimum) {
  // The crossing is timed apart without either agent slowing down: both greedy
  // walks are straight and pass the crossing at different steps, so the first
  // goal-tuple sample already yields 16 + 17 = 33 — exactly what joint-space A*
  // proves optimal. Where the metric's lower bound is tight, sampling lands on
  // the optimum immediately.
  ScenarioSampling s = scenario_sampling("open01_cross");
  core::MultiPlanResult r = sampling::MaRrtStar(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_DOUBLE_EQ(r.cost, 33.0);
  int makespan = 0;
  for (const auto& p : r.paths) makespan = std::max(makespan, static_cast<int>(p.size()) - 1);
  EXPECT_EQ(makespan, 17);
}

TEST(MaRrtStar, Open01SwapSolvesHonestlySuboptimally) {
  // Head-on swap: greedy walks collide head-on, so the tree must grow through
  // intermediate joint states before any chain reaches the goal tuple. The
  // search branch proves 30 is optimal; a seeded sampler at its default budget
  // lands above it — the honest price of sampling with a finite iteration
  // budget (asymptotic optimality is a limit statement, not a budget one).
  ScenarioSampling s = scenario_sampling("open01_swap");
  core::MultiPlanResult r = sampling::MaRrtStar(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_GT(r.cost, 30.0);
}

TEST(MaRrtStar, Maze01TwoFindsACorridorPassBy) {
  // The corridor swap the search branch solves at cost 66: MA-RRT* finds A pass-
  // by solution within budget on the default seed (the paper's own honest finding
  // is that sampling scales worse in tight mazes — the cost here sits far above
  // the coupled optimum, and that is the point of the branch).
  ScenarioSampling s = scenario_sampling("maze01_two");
  core::MultiPlanResult r = sampling::MaRrtStar(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_GT(r.cost, 66.0);
}

TEST(MaRrtStar, WalledOffGoalFailsHonestlyOnBudget) {
  // A goal behind a wall is unreachable by any greedy walk: the tree grows and
  // grows but the goal tuple never becomes a vertex, so the budget dies first.
  // That is "no solution found within budget" — NOT a verdict on the instance
  // (MA-RRT* is only probabilistically complete; unlike joint-space A*, an
  // unsolvable instance never gets a closed-set verdict).
  auto grid = test::make_grid({"######", "#.#..#", "######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 1}, core::Cell{1, 4}}};
  core::MultiPlanResult r = sampling::MaRrtStar(config()).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
  EXPECT_GT(r.stats.expanded_nodes, 0);
}

TEST(MaRrtStar, DuplicateStartFailsImmediately) {
  // Two agents on one cell at t=0 is no joint state at all — the tree never
  // grows past a root that cannot exist (same honest verdict as the search side).
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{1, 1}},
                                     {core::Cell{0, 0}, core::Cell{1, 0}}};
  core::MultiPlanResult r = sampling::MaRrtStar(config()).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.stats.expanded_nodes, 0);
}

TEST(MaRrtStar, TraceEventsAreJointStatesWithoutAgentOrStepFields) {
  std::ostringstream os;
  auto grid = test::make_grid({"...", "...", "..."});
  std::vector<core::AgentTask> tasks{{core::Cell{2, 0}, core::Cell{2, 2}},
                                     {core::Cell{0, 2}, core::Cell{2, 0}}};
  core::TraceRecorder rec(os);
  sampling::MaRrtStar planner(config());
  core::MultiPlanResult r = planner.plan(grid, tasks, &rec);
  ASSERT_TRUE(r.success);

  // Joint-state expansions carry the flattened position tuple and NO agent/t —
  // a joint state has no single owning agent and no single timestep. The root
  // (the start tuple at cost 0) is the first event; the final event reports
  // success with the three standard metrics.
  std::istringstream in(os.str());
  std::string line;
  int expansions = 0;
  bool saw_path_found = false;
  std::vector<std::string> lines;
  while (std::getline(in, line)) lines.push_back(line);
  ASSERT_FALSE(lines.empty());
  for (size_t i = 0; i + 1 < lines.size(); ++i) {
    if (lines[i].find("\"event\":\"node_expanded\"") != std::string::npos) {
      EXPECT_EQ(lines[i].find("\"agent\":"), std::string::npos);
      EXPECT_EQ(lines[i].find("\"t\":"), std::string::npos);
      EXPECT_EQ(state_len(lines[i]), 4);
      ++expansions;
    }
    if (lines[i].find("\"event\":\"path_found\"") != std::string::npos) saw_path_found = true;
  }
  EXPECT_GT(expansions, 0);
  EXPECT_TRUE(saw_path_found);
  // The root event is the start tuple at cost 0.0 ([2,0,0,2] flattened).
  EXPECT_NE(lines[0].find("\"state\":[2,0,0,2]"), std::string::npos);
  EXPECT_NE(lines[0].find("\"cost\":0"), std::string::npos);
  const std::string& last = lines.back();
  EXPECT_NE(last.find("\"event\":\"planning_finished\""), std::string::npos);
  EXPECT_NE(last.find("\"success\":true"), std::string::npos);
  EXPECT_NE(last.find("\"expanded_nodes\":"), std::string::npos);
  EXPECT_NE(last.find("\"makespan\":"), std::string::npos);
  EXPECT_NE(last.find("\"sum_of_costs\":"), std::string::npos);
}
