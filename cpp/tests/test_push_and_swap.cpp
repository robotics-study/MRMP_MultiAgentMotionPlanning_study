// Push and Swap: push-only routing at unconstrained cost, swap-solved head-on
// swaps (and the honest failure where no 2x2 block exists), single-move validity
// of the emitted assignment sequence, and the trace shape of a search-free
// planner. Mirrors python/tests/test_push_and_swap.py case for case.

#include <gtest/gtest.h>

#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include "mrmp/core/params.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/search/push_and_swap.hpp"
#include "mrmp/maps/loader.hpp"
#include "mrmp/maps/occupancy_grid.hpp"
#include "test_util.hpp"

using namespace mrmp;

namespace {

// Push and Swap emits full-horizon paths: every agent's path spans the whole
// makespan, at every step EXACTLY one agent moves (the assignment changes by one
// agent), always into a 4-adjacent cell. There is never simultaneous motion —
// this replaces the pairwise conflict checks of the simultaneous-move planners.
void assert_single_moves(const std::vector<std::vector<core::Cell>>& paths,
                        const std::vector<core::AgentTask>& tasks) {
  ASSERT_FALSE(paths.empty());
  const int horizon = static_cast<int>(paths[0].size()) - 1;
  for (const auto& p : paths) ASSERT_EQ(static_cast<int>(p.size()), horizon + 1);
  for (size_t k = 0; k < paths.size(); ++k) {
    ASSERT_TRUE(paths[k].front() == tasks[k].start);
    ASSERT_TRUE(paths[k].back() == tasks[k].goal);
  }
  for (int t = 1; t <= horizon; ++t) {
    int movers = 0, mover = -1;
    for (size_t k = 0; k < paths.size(); ++k) {
      if (!(paths[k][static_cast<size_t>(t)] == paths[k][static_cast<size_t>(t - 1)])) {
        ++movers;
        mover = static_cast<int>(k);
      }
    }
    ASSERT_EQ(movers, 1) << "step " << t << " moved multiple agents at once";
    const core::Cell prev = paths[static_cast<size_t>(mover)][static_cast<size_t>(t - 1)];
    const core::Cell now = paths[static_cast<size_t>(mover)][static_cast<size_t>(t)];
    ASSERT_EQ(std::abs(now.row - prev.row) + std::abs(now.col - prev.col), 1)
        << "step " << t << " moved more than one grid step";
  }
}

// Per-agent move counts (waits do not count — the metric is real moves).
std::vector<int> per_agent_costs(const std::vector<std::vector<core::Cell>>& paths) {
  std::vector<int> costs;
  for (const auto& p : paths) {
    int c = 0;
    for (size_t t = 1; t < p.size(); ++t) {
      if (!(p[t] == p[t - 1])) ++c;
    }
    costs.push_back(c);
  }
  return costs;
}

core::ParamSet config() {
  return core::ParamSet::from_yaml(test::repo_path("configs/search/push_and_swap.yaml"));
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

}  // namespace

TEST(PushAndSwap, ContractMatchesConfig) {
  search::PushAndSwap planner(config());
  EXPECT_EQ(planner.name(), config().algorithm());
  EXPECT_TRUE(planner.required_capabilities().count(core::Capability::DISCRETE_SPACE) > 0);
  EXPECT_EQ(planner.required_capabilities().size(), 1u);
}

TEST(PushAndSwap, SingleAgentWalksItsShortestPath) {
  // One agent, nobody to push: the BFS shortest path walked straight through —
  // cost equals the Manhattan distance exactly like the prioritized baseline.
  search::PushAndSwap planner(config());
  auto grid = test::make_grid({".....", ".....", ".....", ".....", "....."});
  std::vector<core::AgentTask> tasks{{core::Cell{4, 0}, core::Cell{0, 4}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_single_moves(r.paths, tasks);
  EXPECT_DOUBLE_EQ(r.cost, 8.0);
}

TEST(PushAndSwap, AlreadyAtGoalIsVacuouslyDone) {
  // start == goal: the while-loop body never runs; cost 0, one-step path.
  search::PushAndSwap planner(config());
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{0, 0}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  ASSERT_EQ(r.paths.size(), 1u);
  ASSERT_EQ(r.paths[0].size(), 1u);
  EXPECT_TRUE((r.paths[0][0] == core::Cell{0, 0}));
  EXPECT_DOUBLE_EQ(r.cost, 0.0);
}

TEST(PushAndSwap, DuplicateAssignmentIsNotAnInstance) {
  // The paper's assignment is injective by definition (unique starts AND unique
  // targets). A violating input is not an instance of this problem at all — the
  // planner reports it honestly instead of running on a contradiction.
  search::PushAndSwap planner(config());
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{1, 1}},
                                     {core::Cell{0, 0}, core::Cell{0, 1}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
}

TEST(PushAndSwap, Open01CrossPushOnlyAtUnconstrainedCost) {
  // Crossing paths with nobody in the other's way long enough to matter: agent 0
  // walks its straight path (agent 1 is pushed out of row 10 first), then agent 1
  // walks down column 9 — both at exactly their Manhattan distance (16 + 17 = 33).
  ScenarioSetup s = scenario("open01_cross");
  core::MultiPlanResult r = search::PushAndSwap(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_single_moves(r.paths, s.tasks);
  EXPECT_DOUBLE_EQ(r.cost, 33.0);
  EXPECT_EQ(per_agent_costs(r.paths), std::vector<int>({16, 17}));
}

TEST(PushAndSwap, Open01SwapSolvedByARealSwap) {
  // Head-on swap on row 10 — the instance prioritized planning cannot solve at
  // any cost. Here agent 1 is chain-pushed off the row and the pair exchanges at
  // a free 2x2 block; the maneuvers cost more than the joint optimum (30):
  // 18 + 20 = 38 moves total, honestly suboptimal.
  ScenarioSetup s = scenario("open01_swap");
  core::MultiPlanResult r = search::PushAndSwap(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_single_moves(r.paths, s.tasks);
  EXPECT_EQ(per_agent_costs(r.paths), std::vector<int>({18, 20}));
  EXPECT_DOUBLE_EQ(r.cost, 38.0);
}

TEST(PushAndSwap, Maze01TwoSolvedThroughOneCorridorGap) {
  // Both agents thread the single corridor gap in opposite directions (37 + 37 =
  // 74 — above the joint optimum 66; push and swap buys completeness, not speed).
  ScenarioSetup s = scenario("maze01_two");
  core::MultiPlanResult r = search::PushAndSwap(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_single_moves(r.paths, s.tasks);
  EXPECT_DOUBLE_EQ(r.cost, 74.0);
}

TEST(PushAndSwap, HeadOnCorridorSwapIsHonestlyUnsolvable) {
  // One-wide corridor, agents swapping ends: a swap physically needs a free 2x2
  // block (grid graphs have no triangles), and a width-1 corridor has none — the
  // planner fails honestly even though n = |V| - 2 holds here.
  search::PushAndSwap planner(config());
  auto grid = test::make_grid({"#######", "#.....#", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 1}, core::Cell{1, 5}},
                                     {core::Cell{1, 5}, core::Cell{1, 1}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
}

TEST(PushAndSwap, PocketYieldMakesTheSwapExecutable) {
  // The same head-on swap on a map with one pocket: the 2x2 block
  // {(0,3),(0,4),(1,3),(1,4)} hosts the exchange (8 + 6 = 14 moves).
  search::PushAndSwap planner(config());
  auto grid = test::make_grid({"###..##", "#.....#", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 1}, core::Cell{1, 5}},
                                     {core::Cell{1, 5}, core::Cell{1, 1}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_single_moves(r.paths, tasks);
  EXPECT_EQ(per_agent_costs(r.paths), std::vector<int>({8, 6}));
}

TEST(PushAndSwap, RefilledW2FailsTheAttemptNotThePlanner) {
  // Three agents on the pocket map: a third body makes a new case reachable —
  // clearing w4 steps its occupant into the just-cleared w2, refilling it. That
  // attempt now fails like any other clear failure (the old code moved into the
  // occupied cell and tripped the move invariant). The planner keeps exploring
  // candidates and solves: 21 + 16 + 10 = 47 moves, honestly suboptimal.
  search::PushAndSwap planner(config());
  auto grid = test::make_grid({"###..##", "#.....#", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 5}, core::Cell{1, 2}},
                                     {core::Cell{1, 3}, core::Cell{1, 1}},
                                     {core::Cell{1, 1}, core::Cell{1, 3}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_single_moves(r.paths, tasks);
  EXPECT_EQ(per_agent_costs(r.paths), std::vector<int>({21, 16, 10}));
  EXPECT_DOUBLE_EQ(r.cost, 47.0);
}

TEST(PushAndSwap, TraceEventsAndMetrics) {
  // No search frontier exists here: the trace carries NO node_expanded events —
  // path_found (full-horizon, one event per agent) then planning_finished only.
  std::ostringstream os;
  auto grid = test::make_grid({"###..##", "#.....#", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 1}, core::Cell{1, 5}},
                                     {core::Cell{1, 5}, core::Cell{1, 1}}};
  core::TraceRecorder rec(os);
  search::PushAndSwap planner(config());
  core::MultiPlanResult r = planner.plan(grid, tasks, &rec);
  ASSERT_TRUE(r.success);

  std::istringstream in(os.str());
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(in, line)) lines.push_back(line);
  // The planner itself emits exactly three events; planning_started is the demo
  // runner's line, not the planner's (same split as every other algorithm here).
  ASSERT_EQ(lines.size(), 3u);  // two path_found + planning_finished
  EXPECT_NE(lines[0].find("\"event\":\"path_found\""), std::string::npos);
  EXPECT_NE(lines[0].find("\"agent\":0"), std::string::npos);
  EXPECT_NE(lines[1].find("\"event\":\"path_found\""), std::string::npos);
  EXPECT_NE(lines[1].find("\"agent\":1"), std::string::npos);
  EXPECT_EQ(os.str().find("node_expanded"), std::string::npos);  // search-free planner
  const std::string& last = lines.back();
  EXPECT_NE(last.find("\"event\":\"planning_finished\""), std::string::npos);
  EXPECT_NE(last.find("\"success\":true"), std::string::npos);
  EXPECT_NE(last.find("\"makespan\":14"), std::string::npos);
  EXPECT_NE(last.find("\"sum_of_costs\":14"), std::string::npos);
}
