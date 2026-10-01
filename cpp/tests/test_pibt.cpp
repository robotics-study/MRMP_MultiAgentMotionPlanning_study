// PIBT: per-timestep priority-inheritance decisions on a grid — simultaneous motion
// with no vertex conflict and no swap by construction, honest deadlock (honest failure)
// wherever an edge lies on no cycle, and the pinned determinism of everything the paper
// leaves free (ε = index order, candidate order distance → occupancy → row-major).
// Mirrors python/tests/test_pibt.py case for case.

#include <gtest/gtest.h>

#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include "mrmp/core/params.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/decentralized/pibt.hpp"
#include "mrmp/maps/loader.hpp"
#include "mrmp/maps/occupancy_grid.hpp"
#include "test_util.hpp"

using namespace mrmp;

namespace {

// PIBT emits full-horizon paths: every agent's path spans the whole makespan, every
// step every agent either stays or steps one cell — and across every pair, no vertex
// conflict (same cell at one step) and no swap (pair exchanges cells), which is
// exactly what inheritance + claim-exclusion make structurally impossible.
void assert_simultaneous(const std::vector<std::vector<core::Cell>>& paths,
                         const std::vector<core::AgentTask>& tasks) {
  ASSERT_FALSE(paths.empty());
  const int horizon = static_cast<int>(paths[0].size()) - 1;
  for (const auto& p : paths) ASSERT_EQ(static_cast<int>(p.size()), horizon + 1);
  for (size_t k = 0; k < paths.size(); ++k) {
    ASSERT_TRUE(paths[k].front() == tasks[k].start);
    ASSERT_TRUE(paths[k].back() == tasks[k].goal);
  }
  for (int t = 1; t <= horizon; ++t) {
    for (const auto& p : paths) {
      const core::Cell prev = p[static_cast<size_t>(t - 1)];
      const core::Cell now = p[static_cast<size_t>(t)];
      ASSERT_LE(std::abs(now.row - prev.row) + std::abs(now.col - prev.col), 1)
          << "step " << t << " moved more than one grid step";
    }
  }
  for (size_t i = 0; i < paths.size(); ++i) {
    for (size_t j = i + 1; j < paths.size(); ++j) {
      for (int t = 1; t <= horizon; ++t) {
        const core::Cell a_now = paths[i][static_cast<size_t>(t)];
        const core::Cell a_prev = paths[i][static_cast<size_t>(t - 1)];
        const core::Cell b_now = paths[j][static_cast<size_t>(t)];
        const core::Cell b_prev = paths[j][static_cast<size_t>(t - 1)];
        ASSERT_TRUE(!(a_now == b_now)) << "vertex conflict at step " << t;
        ASSERT_TRUE(!(a_now == b_prev && a_prev == b_now)) << "swap at step " << t;
      }
    }
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
  return core::ParamSet::from_yaml(test::repo_path("configs/decentralized/pibt.yaml"));
}

// The pibt config with a smaller step budget (deterministic stops).
core::ParamSet budget_config(int budget) {
  return core::ParamSet::from_yaml(test::write_temp(
      "pibt_budget.yaml",
      "algorithm: pibt\nsection: decentralized\nscenarios: []\nparams:\n"
      "  - name: max_steps\n    type: int\n"
      "    default: " +
          std::to_string(budget) + "\n    min: 1\n    description: test budget\n"));
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

TEST(Pibt, ContractMatchesConfig) {
  decentralized::Pibt planner(config());
  EXPECT_EQ(planner.name(), config().algorithm());
  EXPECT_TRUE(planner.required_capabilities().count(core::Capability::DISCRETE_SPACE) > 0);
  EXPECT_EQ(planner.required_capabilities().size(), 1u);
}

TEST(Pibt, SingleAgentWalksTheGradientPinnedByTieBreaks) {
  // One agent, nobody to inherit from: pure gradient descent. The path itself pins the
  // tie-break — distance-to-goal alone admits any monotone 8-move route, and row-major
  // order on equal distances picks straight up column 0 first, then along row 0 (a cell
  // at row 0 always sorts before a same-distance cell at row > 0).
  decentralized::Pibt planner(config());
  auto grid = test::make_grid({".....", ".....", ".....", ".....", "....."});
  std::vector<core::AgentTask> tasks{{core::Cell{4, 0}, core::Cell{0, 4}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  const std::vector<core::Cell> pinned{core::Cell{4, 0}, core::Cell{3, 0}, core::Cell{2, 0},
                                      core::Cell{1, 0}, core::Cell{0, 0}, core::Cell{0, 1},
                                      core::Cell{0, 2}, core::Cell{0, 3}, core::Cell{0, 4}};
  ASSERT_EQ(r.paths.size(), 1u);
  EXPECT_TRUE(r.paths[0] == pinned);
  EXPECT_DOUBLE_EQ(r.cost, 8.0);
  // One decision invocation per timestep: the single agent never inherits anyone.
  EXPECT_EQ(r.stats.expanded_nodes, 8);
}

TEST(Pibt, AlreadyAtGoalIsVacuouslyDone) {
  // start == goal: the timestep loop never runs; one-step path, zero cost, and zero
  // decision invocations (the loop body is where every call lives).
  decentralized::Pibt planner(config());
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{0, 0}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  ASSERT_EQ(r.paths.size(), 1u);
  ASSERT_EQ(r.paths[0].size(), 1u);
  EXPECT_TRUE((r.paths[0][0] == core::Cell{0, 0}));
  EXPECT_DOUBLE_EQ(r.cost, 0.0);
  EXPECT_EQ(r.stats.expanded_nodes, 0);
}

TEST(Pibt, DuplicateAssignmentIsNotAnInstance) {
  // The paper's instance is well-formed by definition (unique starts AND unique goals
  // on passable cells). A violating input is not an instance of this problem at all —
  // the planner reports it honestly instead of running on a contradiction.
  decentralized::Pibt planner(config());
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{1, 1}},
                                     {core::Cell{0, 0}, core::Cell{0, 1}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
}

TEST(Pibt, Open01CrossPureGradientWithYielding) {
  // Crossing paths: both agents just descend their distance gradient; the pinned ε
  // order (agent 0 highest) makes agent 1 yield at the crossing instead of waiting for
  // a planned push. The TOTAL equals the offline planners' unconstrained cost
  // (16 + 17 = 33), but nobody was ever pushed — every move was self-chosen.
  ScenarioSetup s = scenario("open01_cross");
  core::MultiPlanResult r = decentralized::Pibt(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_simultaneous(r.paths, s.tasks);
  EXPECT_EQ(per_agent_costs(r.paths), std::vector<int>({16, 17}));
  EXPECT_DOUBLE_EQ(r.cost, 33.0);
}

TEST(Pibt, Open01SwapYieldingHitsTheJointOptimum) {
  // Head-on swap on row 10 — the instance prioritized planning cannot solve at any
  // cost and Push and Swap paid 38 (rigid chain-push) / Rotate 36 for. PIBT needs no
  // primitive: agent 1 simply keeps walking while agent 0's higher priority makes it
  // flow around, landing on exactly the joint optimum 14 + 16 = 30.
  ScenarioSetup s = scenario("open01_swap");
  core::MultiPlanResult r = decentralized::Pibt(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_simultaneous(r.paths, s.tasks);
  EXPECT_EQ(per_agent_costs(r.paths), std::vector<int>({14, 16}));
  EXPECT_DOUBLE_EQ(r.cost, 30.0);
}

TEST(Pibt, PocketYieldMakesTheSwapExecutable) {
  // The same head-on swap on a map WITH a pocket: agent 1 inherits the claim and ducks
  // into (0,4), lets agent 0 pass, then walks out the far side — 4 + 6 = 10 moves.
  // The pocket edge lies on a cycle; that is all completeness ever asks for.
  decentralized::Pibt planner(config());
  auto grid = test::make_grid({"###..##", "#.....#", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 1}, core::Cell{1, 5}},
                                     {core::Cell{1, 5}, core::Cell{1, 1}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_simultaneous(r.paths, tasks);
  EXPECT_EQ(per_agent_costs(r.paths), std::vector<int>({4, 6}));
  EXPECT_DOUBLE_EQ(r.cost, 10.0);
}

TEST(Pibt, TeeJunctionHeadOnDeadlocksHonestly) {
  // The SAME coordinates where Push and Rotate's rotate primitive paid 8 + 8 = 16: a
  // single dead-end stub at (0,3). A retreating agent can only vacate into a cell
  // adjacent to the one it holds — when its back is the corridor end, the stub is not
  // adjacent to where the inheritance chain reaches it, so the claim backtracks and
  // both agents freeze. The edge on no cycle defeats PIBT exactly where Rotate had a
  // repair; PIBT has none, and the budget turns deadlock into honest failure.
  decentralized::Pibt planner(config());
  auto grid = test::make_grid({"###.###", "#.....#", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 1}, core::Cell{1, 5}},
                                     {core::Cell{1, 5}, core::Cell{1, 1}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
}

TEST(Pibt, HeadOnCorridorSwapIsHonestlyUnsolvable) {
  // One-wide corridor, agents swapping ends: no cell has two free neighbors at all, so
  // the retreating agent is always cornered against the wall and every claim on its
  // cell backtracks. Honest failure — a budget exhaustion, never a proof of
  // unsolvability (the same convention as CBS's tree budget).
  decentralized::Pibt planner(config());
  auto grid = test::make_grid({"#######", "#.....#", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 1}, core::Cell{1, 5}},
                                     {core::Cell{1, 5}, core::Cell{1, 1}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
}

TEST(Pibt, BudgetExhaustionIsHonestAndCounted) {
  // Same deadlock as the corridor scenario at a tiny budget: failure after exactly
  // max_steps timesteps × 2 agents = 10 decision invocations, reported honestly.
  decentralized::Pibt planner(budget_config(5));
  auto grid = test::make_grid({"#######", "#.....#", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 1}, core::Cell{1, 5}},
                                     {core::Cell{1, 5}, core::Cell{1, 1}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
  EXPECT_EQ(r.stats.expanded_nodes, 10);
}

TEST(Pibt, TraceEventsAndMetrics) {
  // No search frontier exists here: the trace carries NO node_expanded events —
  // path_found (full-horizon, one event per agent in index order) then
  // planning_finished only. The execution replay IS the demo for this branch.
  std::ostringstream os;
  auto grid = test::make_grid({"###..##", "#.....#", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 1}, core::Cell{1, 5}},
                                     {core::Cell{1, 5}, core::Cell{1, 1}}};
  core::TraceRecorder rec(os);
  decentralized::Pibt planner(config());
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
  EXPECT_NE(last.find("\"makespan\":7"), std::string::npos);
  EXPECT_NE(last.find("\"sum_of_costs\":10"), std::string::npos);
  // one top-level decision per agent per step, no inheritance chain here: 2 × 7
  EXPECT_NE(last.find("\"expanded_nodes\":14"), std::string::npos);
}
