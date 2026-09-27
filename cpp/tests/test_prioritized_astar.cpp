// Prioritized A*: per-agent optimality against reservations, conflict-freedom,
// the stay-at-goal guard, and the honest failure of an unsolvable priority order.
// Mirrors python/tests/test_prioritized_astar.py case for case.

#include <gtest/gtest.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include "mrmp/core/params.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/mapf/prioritized_astar.hpp"
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
  return core::ParamSet::from_yaml(test::repo_path("configs/mapf/prioritized_astar.yaml"));
}

// Load a repo scenario and convert its world-coord endpoints to cells (exactly
// what the demo driver does), so both languages solve identical tasks.
struct ScenarioSetup {
  maps::OccupancyGrid2D grid;
  std::vector<core::AgentTask> tasks;
};
ScenarioSetup scenario(const std::string& name) {
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
  return ScenarioSetup{std::move(*grid), std::move(tasks)};
}

int cost_of(const std::vector<core::Cell>& path) { return static_cast<int>(path.size()) - 1; }

}  // namespace

TEST(PrioritizedAStar, ContractMatchesConfig) {
  mapf::PrioritizedAStar planner(config());
  EXPECT_EQ(planner.name(), config().algorithm());
  EXPECT_TRUE(planner.required_capabilities().count(core::Capability::DISCRETE_SPACE) > 0);
  EXPECT_EQ(planner.required_capabilities().size(), 1u);
}

TEST(PrioritizedAStar, SingleAgentIsPlainOptimalAStar) {
  // With one agent there are no reservations: cost must equal the Manhattan
  // distance (the unconstrained optimum) on an open grid.
  mapf::PrioritizedAStar planner(config());
  auto grid = test::make_grid({".....", ".....", ".....", ".....", "....."});
  std::vector<core::AgentTask> tasks{{core::Cell{4, 0}, core::Cell{0, 4}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  EXPECT_DOUBLE_EQ(r.cost, 8.0);
  ASSERT_EQ(r.paths.size(), 1u);
  EXPECT_TRUE((r.paths[0].front() == core::Cell{4, 0}));
  EXPECT_TRUE((r.paths[0].back() == core::Cell{0, 4}));
}

TEST(PrioritizedAStar, Maze01TwoHeadOnPassesByTiming) {
  // Both agents share the one corridor but their timings stagger naturally:
  // each still achieves its unconstrained shortest cost (33 + 33 = 66).
  ScenarioSetup s = scenario("maze01_two");
  core::MultiPlanResult r = mapf::PrioritizedAStar(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_DOUBLE_EQ(r.cost, 66.0);
  int makespan = 0;
  for (const auto& p : r.paths) makespan = std::max(makespan, cost_of(p));
  EXPECT_EQ(makespan, 33);
}

TEST(PrioritizedAStar, Open01CrossBothGoStraight) {
  // The crossing at (10,9) is timed apart without either agent slowing down:
  // both costs equal their Manhattan distance (16 + 17 = 33).
  ScenarioSetup s = scenario("open01_cross");
  core::MultiPlanResult r = mapf::PrioritizedAStar(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_DOUBLE_EQ(r.cost, 33.0);
  int makespan = 0;
  for (const auto& p : r.paths) makespan = std::max(makespan, cost_of(p));
  EXPECT_EQ(makespan, 17);
}

TEST(PrioritizedAStar, Open01SwapDetoursAroundTheMovingWall) {
  // Head-on swap on row 10: agent 0's straight path is a moving wall; agent 1
  // cannot pass it head-on (parity rules out cost 15) and must detour via an
  // adjacent row — 14 + 16 = 30, which here equals the joint optimum.
  ScenarioSetup s = scenario("open01_swap");
  core::MultiPlanResult r = mapf::PrioritizedAStar(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_DOUBLE_EQ(r.cost, 30.0);
  int makespan = 0;
  for (const auto& p : r.paths) makespan = std::max(makespan, cost_of(p));
  EXPECT_EQ(makespan, 16);
}

TEST(PrioritizedAStar, HeadOnCorridorSwapIsUnsolvable) {
  // One-wide corridor, agents swapping ends: no wait or detour exists, so the
  // second agent honestly fails — prioritized planning is not complete.
  auto grid = test::make_grid({"#####", ".....", "#####"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 0}, core::Cell{1, 4}},
                                     {core::Cell{1, 4}, core::Cell{1, 0}}};
  core::MultiPlanResult r = mapf::PrioritizedAStar(config()).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
}

TEST(PrioritizedAStar, PocketYieldMakesTheSwapSolvable) {
  // Same head-on swap, but one pocket cell at (1,4) beside the corridor: agent 1
  // ducks into it while agent 0 sweeps past, then continues. Agent 0 keeps its
  // straight path (cost 6); agent 1's detour is forced to cost exactly 9.
  auto grid = test::make_grid({"#######", "####.##", ".......", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{2, 0}, core::Cell{2, 6}},
                                     {core::Cell{2, 6}, core::Cell{2, 0}}};
  core::MultiPlanResult r = mapf::PrioritizedAStar(config()).plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  ASSERT_EQ(r.paths.size(), 2u);
  EXPECT_EQ(cost_of(r.paths[0]), 6);
  EXPECT_EQ(cost_of(r.paths[1]), 9);
}

TEST(PrioritizedAStar, GoalStaysOccupiedUntilArrival) {
  // Agent 0 passes through (0,2) at step 2 and parks at (0,4). Agent 1's goal IS
  // (0,2): arriving before step 2 would park it on top of agent 0's pass-through,
  // so the goal guard must force arrival after it (cost 3, not 1).
  auto grid = test::make_grid({".....", "##.##"});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{0, 4}},
                                     {core::Cell{1, 2}, core::Cell{0, 2}}};
  core::MultiPlanResult r = mapf::PrioritizedAStar(config()).plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  ASSERT_EQ(r.paths.size(), 2u);
  EXPECT_EQ(cost_of(r.paths[1]), 3);
}

TEST(PrioritizedAStar, StartCellOccupiedFailsImmediately) {
  // An earlier path standing on the later agent's start at t=0 is an unavoidable
  // joint conflict — no search can undo it.
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{1, 1}},
                                     {core::Cell{0, 0}, core::Cell{1, 0}}};
  core::MultiPlanResult r = mapf::PrioritizedAStar(config()).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
}

TEST(PrioritizedAStar, TraceEventsAndMetrics) {
  std::ostringstream os;
  auto grid = test::make_grid({"...", "...", "..."});
  std::vector<core::AgentTask> tasks{{core::Cell{2, 0}, core::Cell{2, 2}},
                                     {core::Cell{0, 2}, core::Cell{2, 0}}};
  core::TraceRecorder rec(os);
  mapf::PrioritizedAStar planner(config());
  core::MultiPlanResult r = planner.plan(grid, tasks, &rec);
  ASSERT_TRUE(r.success);

  // Every expansion carries its agent index and space-time step; the final event
  // reports success with the three standard metrics.
  std::istringstream in(os.str());
  std::string line;
  int expansions = 0;
  bool saw_path_found = false, saw_finished = false;
  std::vector<std::string> lines;
  while (std::getline(in, line)) lines.push_back(line);
  ASSERT_FALSE(lines.empty());
  for (size_t i = 0; i + 1 < lines.size(); ++i) {
    if (lines[i].find("\"event\":\"node_expanded\"") != std::string::npos) {
      EXPECT_NE(lines[i].find("\"agent\":"), std::string::npos);
      EXPECT_NE(lines[i].find("\"t\":"), std::string::npos);
      ++expansions;
    }
    if (lines[i].find("\"event\":\"path_found\"") != std::string::npos) saw_path_found = true;
  }
  EXPECT_GT(expansions, 0);
  EXPECT_TRUE(saw_path_found);
  const std::string& last = lines.back();
  saw_finished = last.find("\"event\":\"planning_finished\"") != std::string::npos;
  EXPECT_TRUE(saw_finished);
  EXPECT_NE(last.find("\"success\":true"), std::string::npos);
  EXPECT_NE(last.find("\"expanded_nodes\":"), std::string::npos);
  EXPECT_NE(last.find("\"makespan\":"), std::string::npos);
  EXPECT_NE(last.find("\"sum_of_costs\":"), std::string::npos);
}
