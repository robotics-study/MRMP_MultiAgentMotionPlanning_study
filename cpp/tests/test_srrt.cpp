// sRRT: seeded determinism (the PRNG is part of the algorithm's identity), valid
// conflict-free joint paths on every scenario, honest sub-optimality where
// coupling forces detours above the coupled optimum, both failure verdicts (an
// instant instance verdict for an unreachable goal, an honest budget failure
// that is NOT a verdict on the instance), and the trace shape of tree growth
// (joint-state node_expanded without cost/agent/t). Mirrors
// python/tests/test_srrt.py case for case.

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
#include "mrmp/sampling/srrt.hpp"
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
  return core::ParamSet::from_yaml(test::repo_path("configs/sampling/srrt.yaml"));
}

// The declared srrt config with defaults overridden (same declarations,
// different defaults — the loader validates them like the real file).
core::ParamSet override_config(int seed, int max_iterations) {
  const std::string params =
      "  - name: seed\n    type: int\n    default: " + std::to_string(seed) +
      "\n    min: 1\n    max: 2147483646\n    description: test seed\n"
      "  - name: goal_sampling_probability\n    type: float\n    default: 0.5\n    min: 0.0\n"
      "    max: 1.0\n    description: test bias\n"
      "  - name: max_iterations\n    type: int\n    default: " +
      std::to_string(max_iterations) + "\n    min: 1\n    description: test iterations\n";
  return core::ParamSet::from_yaml(
      test::write_temp("srrt_override.yaml", "algorithm: srrt\nsection: sampling\nparams:\n" + params));
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

TEST(Srrt, ContractMatchesConfig) {
  sampling::Srrt planner(config());
  EXPECT_EQ(planner.name(), config().algorithm());
  EXPECT_TRUE(planner.required_capabilities().count(core::Capability::DISCRETE_SPACE) > 0);
  EXPECT_EQ(planner.required_capabilities().size(), 1u);
}

TEST(Srrt, SingleAgentFollowsItsBfsPolicyOnEverySeed) {
  // One agent: no robot-robot collision can ever fire, so the collision set
  // stays empty forever and every expansion just takes one more policy step —
  // the tree degenerates to a single line along the BFS-tree policy from start
  // to goal. Sampling only decides WHEN the line extends, never where; cost is
  // the individually-optimal Manhattan length on an open grid, seed-independently.
  auto grid = test::make_grid({".....", ".....", ".....", ".....", "....."});
  std::vector<core::AgentTask> tasks{{core::Cell{4, 0}, core::Cell{0, 4}}};
  for (int seed : {42, 7, 123}) {
    sampling::Srrt planner(override_config(seed, 300));
    core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
    ASSERT_TRUE(r.success) << "seed " << seed;
    EXPECT_DOUBLE_EQ(r.cost, 8.0);
    ASSERT_EQ(r.paths.size(), 1u);
    EXPECT_TRUE((r.paths[0].front() == core::Cell{4, 0}));
    EXPECT_TRUE((r.paths[0].back() == core::Cell{0, 4}));
  }
}

TEST(Srrt, SameSeedReplaysByteIdenticalPlans) {
  // The MINSTD stream is part of the algorithm's identity: all three language
  // engines must replay identical trees from one seed. Same-seed reruns are the
  // first line of that contract.
  ScenarioSampling s = scenario_sampling("open01_swap");
  core::MultiPlanResult a = sampling::Srrt(config()).plan(s.grid, s.tasks, nullptr);
  core::MultiPlanResult b = sampling::Srrt(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(a.success);
  ASSERT_TRUE(b.success);
  EXPECT_EQ(a.paths, b.paths);
  EXPECT_DOUBLE_EQ(a.cost, b.cost);
  EXPECT_EQ(a.stats.expanded_nodes, b.stats.expanded_nodes);
}

TEST(Srrt, Open01CrossStaysDecoupledAtTheTightBound) {
  // The crossing is timed apart by the policies themselves: both BFS policy
  // paths pass the crossing cell at different steps, so no collision ever
  // fires, the collision sets stay empty (pure decoupled execution), and each
  // agent's path is its individually-optimal one — 16 + 17 = 33, exactly what
  // joint-space A* proves optimal. Where the metric's lower bound is tight,
  // sRRT lands on it by construction, not by luck of sampling.
  ScenarioSampling s = scenario_sampling("open01_cross");
  core::MultiPlanResult r = sampling::Srrt(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_DOUBLE_EQ(r.cost, 33.0);
  int makespan = 0;
  for (const auto& p : r.paths) makespan = std::max(makespan, static_cast<int>(p.size()) - 1);
  EXPECT_EQ(makespan, 17);
}

TEST(Srrt, Open01SwapCouplesAndSolvesHonestlySuboptimally) {
  // Head-on along one open row: no timing offset can save two policies walking
  // into each other, so the first head-on step fires an informative collision,
  // both robots join every ancestor's collision set, and from there sRRT IS a
  // plain joint-space RRT steered by samples. The search branch proves 30 is
  // optimal; this seeded run lands above it — the honest price of sampling with
  // an unproven guarantee (the paper leaves even probabilistic completeness
  // unproven).
  ScenarioSampling s = scenario_sampling("open01_swap");
  core::MultiPlanResult r = sampling::Srrt(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_GT(r.cost, 30.0);
}

TEST(Srrt, Maze01TwoPassesTheCorridorByTimingNotCoupling) {
  // The corridor swap the search branch solves at cost 66: here each agent's
  // individually-optimal policy path (33 steps each) passes the shared corridor
  // one step apart, so no collision ever fires and sRRT never couples — the
  // result is exactly the sum of individual optima, which happens to EQUAL the
  // coupled optimum. That is timing luck on this map geometry, not a property
  // of sRRT: the algorithm cannot promise it on other instances.
  ScenarioSampling s = scenario_sampling("maze01_two");
  core::MultiPlanResult r = sampling::Srrt(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_DOUBLE_EQ(r.cost, 66.0);
}

TEST(Srrt, UnreachableGoalFailsImmediatelyAsAnInstanceVerdict) {
  // A goal whose free component does not contain the start makes the individual
  // policy undefined at the start cell — no collision-free joint trajectory can
  // exist for ANY sample, so sRRT stops with a verdict on the instance itself
  // (unlike MA-RRT*, which can only ever say "budget exhausted").
  auto grid = test::make_grid({"######", "#.#..#", "######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 1}, core::Cell{1, 4}}};
  core::MultiPlanResult r = sampling::Srrt(config()).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
  EXPECT_EQ(r.stats.expanded_nodes, 0);
}

TEST(Srrt, BudgetExhaustionIsNotAnInstanceVerdict) {
  // The same solvable instance with a tiny budget fails too — but as "no
  // solution found within budget", not an instance verdict (the tree grew, the
  // policies exist; the goal tuple just never became a vertex in 3 draws).
  ScenarioSampling s = scenario_sampling("open01_swap");
  core::MultiPlanResult r = sampling::Srrt(override_config(42, 3)).plan(s.grid, s.tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
  EXPECT_GT(r.stats.expanded_nodes, 0);
}

TEST(Srrt, DuplicateStartFailsImmediately) {
  // Two agents on one cell at t=0 is no joint state at all — the tree never
  // grows past a root that cannot exist (same honest verdict as the search side).
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{1, 1}},
                                     {core::Cell{0, 0}, core::Cell{1, 0}}};
  core::MultiPlanResult r = sampling::Srrt(config()).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.stats.expanded_nodes, 0);
}

TEST(Srrt, TraceEventsAreJointStatesWithoutCostAgentOrStepFields) {
  std::ostringstream os;
  auto grid = test::make_grid({"...", "...", "..."});
  std::vector<core::AgentTask> tasks{{core::Cell{2, 0}, core::Cell{2, 2}},
                                     {core::Cell{0, 2}, core::Cell{2, 0}}};
  core::TraceRecorder rec(os);
  sampling::Srrt planner(config());
  core::MultiPlanResult r = planner.plan(grid, tasks, &rec);
  ASSERT_TRUE(r.success);

  // Joint-state expansions carry the flattened position tuple and NO cost,
  // agent or t — sRRT nodes are joint states with no g-value (no optimality to
  // count toward), no owning agent, and no single timestep. The root (the start
  // tuple) is the first event; the final event reports success with the three
  // standard metrics.
  std::istringstream in(os.str());
  std::string line;
  int expansions = 0;
  bool saw_path_found = false;
  std::vector<std::string> lines;
  while (std::getline(in, line)) lines.push_back(line);
  ASSERT_FALSE(lines.empty());
  for (size_t i = 0; i + 1 < lines.size(); ++i) {
    if (lines[i].find("\"event\":\"node_expanded\"") != std::string::npos) {
      EXPECT_EQ(lines[i].find("\"cost\":"), std::string::npos);
      EXPECT_EQ(lines[i].find("\"agent\":"), std::string::npos);
      EXPECT_EQ(lines[i].find("\"t\":"), std::string::npos);
      EXPECT_EQ(state_len(lines[i]), 4);
      ++expansions;
    }
    if (lines[i].find("\"event\":\"path_found\"") != std::string::npos) saw_path_found = true;
  }
  EXPECT_GT(expansions, 0);
  EXPECT_TRUE(saw_path_found);
  // The root event is the start tuple flattened ([2,0,0,2]).
  EXPECT_NE(lines[0].find("\"state\":[2,0,0,2]"), std::string::npos);
  const std::string& last = lines.back();
  EXPECT_NE(last.find("\"event\":\"planning_finished\""), std::string::npos);
  EXPECT_NE(last.find("\"success\":true"), std::string::npos);
  EXPECT_NE(last.find("\"expanded_nodes\":"), std::string::npos);
  EXPECT_NE(last.find("\"makespan\":"), std::string::npos);
  EXPECT_NE(last.find("\"sum_of_costs\":"), std::string::npos);
}
