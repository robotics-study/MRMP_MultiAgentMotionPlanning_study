// Joint-space A*: joint-optimal costs on every solvable scenario (including the
// head-on corridor swap prioritized honestly fails), duplicate-start failure,
// and the trace shape of joint-state expansion (no agent/t fields). Mirrors
// python/tests/test_joint_astar.py case for case.

#include <gtest/gtest.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include "mrmp/core/params.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/mapf/joint_astar.hpp"
#include "mrmp/maps/loader.hpp"
#include "mrmp/maps/occupancy_grid.hpp"
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
  return core::ParamSet::from_yaml(test::repo_path("configs/mapf/joint_astar.yaml"));
}

// Load a repo scenario and convert its world-coord endpoints to cells (exactly
// what the demo driver does), so both languages solve identical tasks.
struct ScenarioSetupJoint {
  maps::OccupancyGrid2D grid;
  std::vector<core::AgentTask> tasks;
};
ScenarioSetupJoint scenario_joint(const std::string& name) {
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
  return ScenarioSetupJoint{std::move(*grid), std::move(tasks)};
}

int cost_of(const std::vector<core::Cell>& path) { return static_cast<int>(path.size()) - 1; }

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

TEST(JointAStar, ContractMatchesConfig) {
  mapf::JointAStar planner(config());
  EXPECT_EQ(planner.name(), config().algorithm());
  EXPECT_TRUE(planner.required_capabilities().count(core::Capability::DISCRETE_SPACE) > 0);
  EXPECT_EQ(planner.required_capabilities().size(), 1u);
}

TEST(JointAStar, SingleAgentIsPlainOptimalAStar) {
  // With one agent the joint state degenerates to a plain cell: the search must
  // return the Manhattan optimum, exactly like prioritized's single-agent case.
  mapf::JointAStar planner(config());
  auto grid = test::make_grid({".....", ".....", ".....", ".....", "....."});
  std::vector<core::AgentTask> tasks{{core::Cell{4, 0}, core::Cell{0, 4}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  EXPECT_DOUBLE_EQ(r.cost, 8.0);
  ASSERT_EQ(r.paths.size(), 1u);
  EXPECT_TRUE((r.paths[0].front() == core::Cell{4, 0}));
  EXPECT_TRUE((r.paths[0].back() == core::Cell{0, 4}));
}

TEST(JointAStar, Maze01TwoHeadOnPassesByTiming) {
  // The coupled optimum equals what prioritized achieved by luck of timing:
  // both agents still achieve their unconstrained shortest cost (33 + 33 = 66).
  ScenarioSetupJoint s = scenario_joint("maze01_two");
  core::MultiPlanResult r = mapf::JointAStar(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_DOUBLE_EQ(r.cost, 66.0);
  int makespan = 0;
  for (const auto& p : r.paths) makespan = std::max(makespan, cost_of(p));
  EXPECT_EQ(makespan, 33);
}

TEST(JointAStar, Open01CrossBothGoStraight) {
  // The crossing at (10,9) is timed apart without either agent slowing down:
  // both costs equal their Manhattan distance (16 + 17 = 33) — the joint optimum.
  ScenarioSetupJoint s = scenario_joint("open01_cross");
  core::MultiPlanResult r = mapf::JointAStar(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_DOUBLE_EQ(r.cost, 33.0);
  int makespan = 0;
  for (const auto& p : r.paths) makespan = std::max(makespan, cost_of(p));
  EXPECT_EQ(makespan, 17);
}

TEST(JointAStar, Open01SwapDetoursAroundTheMovingWall) {
  // Head-on swap on row 10: parity rules out cost 15, so the detour costs
  // exactly 16 and the sum 30 is the joint optimum (prioritized hit it too —
  // here that is a theorem, not luck).
  ScenarioSetupJoint s = scenario_joint("open01_swap");
  core::MultiPlanResult r = mapf::JointAStar(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_DOUBLE_EQ(r.cost, 30.0);
  int makespan = 0;
  for (const auto& p : r.paths) makespan = std::max(makespan, cost_of(p));
  EXPECT_EQ(makespan, 16);
}

TEST(JointAStar, HeadOnCorridorSwapIsUnsolvable) {
  // One-wide corridor, agents swapping ends: no wait or detour exists anywhere,
  // so the complete search exhausts its state space and honestly reports
  // failure — this is exactly where prioritized's incompleteness was a limit.
  auto grid = test::make_grid({"#####", ".....", "#####"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 0}, core::Cell{1, 4}},
                                     {core::Cell{1, 4}, core::Cell{1, 0}}};
  core::MultiPlanResult r = mapf::JointAStar(config()).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
}

TEST(JointAStar, PocketYieldMakesTheSwapSolvable) {
  // Same head-on swap with one pocket cell at (1,4): solvable, and the joint
  // optimum equals what prioritized achieved by priority luck — agent 0 keeps
  // its straight path (cost 6), agent 1 ducks through the pocket (cost 9).
  auto grid = test::make_grid({"#######", "####.##", ".......", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{2, 0}, core::Cell{2, 6}},
                                     {core::Cell{2, 6}, core::Cell{2, 0}}};
  core::MultiPlanResult r = mapf::JointAStar(config()).plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  ASSERT_EQ(r.paths.size(), 2u);
  EXPECT_EQ(cost_of(r.paths[0]), 6);
  EXPECT_EQ(cost_of(r.paths[1]), 9);
}

TEST(JointAStar, ParkedGoalBlocksTheOnlyLane) {
  // Agent 0's goal is the corridor cell agent 1 must pass through to reach its
  // own goal. Stay-at-goal semantics are shared across both planners: once
  // agent 0 parks there forever, no joint plan exists — the complete search
  // exhausts and says so (after actually searching, unlike the duplicate start).
  auto grid = test::make_grid({"#####", ".....", "#####" });
  std::vector<core::AgentTask> tasks{{core::Cell{1, 0}, core::Cell{1, 2}},
                                     {core::Cell{1, 4}, core::Cell{1, 0}}};
  core::MultiPlanResult r = mapf::JointAStar(config()).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_GT(r.stats.expanded_nodes, 0);
}

TEST(JointAStar, DuplicateStartFailsImmediately) {
  // Two agents on one cell at t=0 is no joint state at all — the search does
  // not even expand.
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{1, 1}},
                                     {core::Cell{0, 0}, core::Cell{1, 0}}};
  core::MultiPlanResult r = mapf::JointAStar(config()).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.stats.expanded_nodes, 0);
}

TEST(JointAStar, TraceEventsAreJointStatesWithoutAgentOrStepFields) {
  std::ostringstream os;
  auto grid = test::make_grid({"...", "...", "..."});
  std::vector<core::AgentTask> tasks{{core::Cell{2, 0}, core::Cell{2, 2}},
                                     {core::Cell{0, 2}, core::Cell{2, 0}}};
  core::TraceRecorder rec(os);
  mapf::JointAStar planner(config());
  core::MultiPlanResult r = planner.plan(grid, tasks, &rec);
  ASSERT_TRUE(r.success);

  // Joint-state expansions carry the flattened position tuple and NO agent/t —
  // a joint state has no single owning agent and no single timestep. The final
  // event reports success with the three standard metrics.
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
  const std::string& last = lines.back();
  EXPECT_NE(last.find("\"event\":\"planning_finished\""), std::string::npos);
  EXPECT_NE(last.find("\"success\":true"), std::string::npos);
  EXPECT_NE(last.find("\"expanded_nodes\":"), std::string::npos);
  EXPECT_NE(last.find("\"makespan\":"), std::string::npos);
  EXPECT_NE(last.find("\"sum_of_costs\":"), std::string::npos);
}
