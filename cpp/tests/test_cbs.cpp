// CBS: joint-optimal costs on every solvable scenario, honest failures on the
// unsolvable ones (finite-tree exhaustion AND budget stop), and the trace shape
// of constraint-tree search. Mirrors python/tests/test_cbs.py case for case.

#include <gtest/gtest.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include "mrmp/core/params.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/search/cbs.hpp"
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
  return core::ParamSet::from_yaml(test::repo_path("configs/search/cbs.yaml"));
}

// The cbs config with a smaller CT-expansion budget (deterministic stops).
core::ParamSet budget_config(int budget) {
  return core::ParamSet::from_yaml(test::write_temp(
      "cbs_budget.yaml",
      "algorithm: cbs\nsection: search\nscenarios: []\nparams:\n"
      "  - name: max_ct_expansions\n    type: int\n"
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

int cost_of(const std::vector<core::Cell>& path) { return static_cast<int>(path.size()) - 1; }

int makespan_of(const std::vector<std::vector<core::Cell>>& paths) {
  int m = 0;
  for (const auto& p : paths) m = std::max(m, cost_of(p));
  return m;
}

}  // namespace

TEST(Cbs, ContractMatchesConfig) {
  search::Cbs planner(config());
  EXPECT_EQ(planner.name(), config().algorithm());
  EXPECT_TRUE(planner.required_capabilities().count(core::Capability::DISCRETE_SPACE) > 0);
  EXPECT_EQ(planner.required_capabilities().size(), 1u);
  // The semi-decidability budget is the one declared parameter.
  EXPECT_EQ(config().get_int("max_ct_expansions"), 64);
}

TEST(Cbs, SingleAgentIsPlainOptimalAStar) {
  // One agent, no possible conflict: the CT never branches and the root's
  // unconstrained sub-search IS the answer — the Manhattan optimum.
  search::Cbs planner(config());
  auto grid = test::make_grid({".....", ".....", ".....", ".....", "....."});
  std::vector<core::AgentTask> tasks{{core::Cell{4, 0}, core::Cell{0, 4}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  EXPECT_DOUBLE_EQ(r.cost, 8.0);
  ASSERT_EQ(r.paths.size(), 1u);
  EXPECT_TRUE((r.paths[0].front() == core::Cell{4, 0}));
  EXPECT_TRUE((r.paths[0].back() == core::Cell{0, 4}));
}

TEST(Cbs, Maze01TwoHeadOnPassesByTiming) {
  // The joint optimum equals what prioritized achieved by luck of timing; CBS
  // reaches it by branching on the corridor meeting instead of by priority.
  ScenarioSetup s = scenario("maze01_two");
  core::MultiPlanResult r = search::Cbs(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_DOUBLE_EQ(r.cost, 66.0);
  EXPECT_EQ(makespan_of(r.paths), 33);
}

TEST(Cbs, Open01CrossBothGoStraight) {
  // The crossing is timed apart without either agent slowing down: both costs
  // equal their Manhattan distance (16 + 17 = 33) — the joint optimum.
  ScenarioSetup s = scenario("open01_cross");
  core::MultiPlanResult r = search::Cbs(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_DOUBLE_EQ(r.cost, 33.0);
  EXPECT_EQ(makespan_of(r.paths), 17);
}

TEST(Cbs, Open01SwapDetoursAroundTheMovingWall) {
  // Head-on swap on row 10: parity rules out cost 15 per agent, so the sum is
  // 30 and one of them detours — CBS finds it by branching on the head-on
  // collision instead of hoping a priority order stumbles into it.
  ScenarioSetup s = scenario("open01_swap");
  core::MultiPlanResult r = search::Cbs(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_DOUBLE_EQ(r.cost, 30.0);
  EXPECT_EQ(makespan_of(r.paths), 16);
}

TEST(Cbs, DuplicateStartFailsByBranchingToDeadEnds) {
  // Two agents on one cell at t=0 is unsolvable — and here the tree really does
  // die on its own constraints: both children constrain an agent off its own
  // start cell (instantly dead sub-searches) and the queue empties far under the
  // budget. That is what a genuine unsolvability verdict looks like for CBS.
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{1, 1}},
                                     {core::Cell{0, 0}, core::Cell{1, 0}}};
  core::MultiPlanResult r = search::Cbs(config()).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
  // Root expansions only: both children died before expanding anything.
  EXPECT_GT(r.stats.expanded_nodes, 0);
}

TEST(Cbs, CorridorSwapBudgetStop) {
  // One-wide corridor, agents swapping ends: no joint plan exists — but CBS
  // cannot SEE that. Every branch just pushes the crossing later, so only
  // max_ct_expansions stops it. The verdict is honest: "no solution found within
  // budget", pinned here to exactly 8 CT expansions' worth of work —
  // deterministic in every language, which is why this is a count not a clock.
  auto grid = test::make_grid({"#####", ".....", "#####"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 0}, core::Cell{1, 4}},
                                     {core::Cell{1, 4}, core::Cell{1, 0}}};
  core::MultiPlanResult r = search::Cbs(budget_config(8)).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
  // Root (both agents' unconstrained A*s) + seven more CT expansions before the
  // budget trips — pinned exactly because every language replays identically.
  EXPECT_EQ(r.stats.expanded_nodes, 132);
}

TEST(Cbs, BudgetExhaustionIsNotAVerdict) {
  // The SAME solvable cross scenario with a budget of 1 fails after the root
  // alone. A budget stop says "no solution found within budget", never
  // "unsolvable" — the pair to the default-config test above.
  ScenarioSetup s = scenario("open01_cross");
  core::MultiPlanResult r = search::Cbs(budget_config(1)).plan(s.grid, s.tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_GT(r.stats.expanded_nodes, 0);
}

TEST(Cbs, PocketYieldMakesTheSwapSolvable) {
  // Same head-on swap with one pocket cell at (1,4): solvable, and the joint
  // optimum is what joint-space A* proved — sum of costs exactly 15. CBS must
  // match it.
  auto grid = test::make_grid({"#######", "####.##", ".......", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{2, 0}, core::Cell{2, 6}},
                                     {core::Cell{2, 6}, core::Cell{2, 0}}};
  core::MultiPlanResult r = search::Cbs(config()).plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths);
  EXPECT_DOUBLE_EQ(r.cost, 15.0);
}

TEST(Cbs, TraceShowsTheConstraintTreeLifecycle) {
  // A crossing pair forces at least one branch: the trace must show low-level
  // expansions carrying agent + t, then conflict_found, then constraint_added
  // for BOTH agents of the conflicting pair, and path_found only for the final
  // solution paths in agent-index order.
  std::ostringstream os;
  auto grid = test::make_grid({"...", "...", "..."});
  std::vector<core::AgentTask> tasks{{core::Cell{2, 0}, core::Cell{2, 2}},
                                     {core::Cell{0, 2}, core::Cell{2, 0}}};
  core::TraceRecorder rec(os);
  search::Cbs planner(config());
  core::MultiPlanResult r = planner.plan(grid, tasks, &rec);
  ASSERT_TRUE(r.success);

  std::istringstream in(os.str());
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(in, line)) lines.push_back(line);
  ASSERT_FALSE(lines.empty());

  int expansions = 0;
  size_t first_conflict = lines.size(), last_tree = 0, first_path_found = lines.size();
  bool saw_agent0_constraint = false, saw_agent1_constraint = false;
  for (size_t i = 0; i + 1 < lines.size(); ++i) {
    const std::string& l = lines[i];
    if (l.find("\"event\":\"node_expanded\"") != std::string::npos) {
      EXPECT_NE(l.find("\"agent\":"), std::string::npos);
      EXPECT_NE(l.find("\"t\":"), std::string::npos);
      ++expansions;
    }
    if (l.find("\"event\":\"conflict_found\"") != std::string::npos) {
      bool vertex = l.find("\"kind\":\"vertex\"") != std::string::npos;
      bool edge = l.find("\"kind\":\"edge\"") != std::string::npos;
      EXPECT_TRUE(vertex || edge);
      // The only pair is (0, 1), ordered.
      EXPECT_NE(l.find("\"agents\":[0,1]"), std::string::npos);
      if (first_conflict == lines.size()) first_conflict = i;
    }
    if (l.find("\"event\":\"constraint_added\"") != std::string::npos) {
      if (l.find("\"agent\":0,") != std::string::npos) saw_agent0_constraint = true;
      if (l.find("\"agent\":1,") != std::string::npos) saw_agent1_constraint = true;
    }
    if (l.find("\"event\":\"path_found\"") != std::string::npos && first_path_found == lines.size()) {
      first_path_found = i;
    }
    if (l.find("\"event\":\"conflict_found\"") != std::string::npos ||
        l.find("\"event\":\"constraint_added\"") != std::string::npos) {
      last_tree = i;
    }
  }
  EXPECT_GT(expansions, 0);
  // Both sides of the first conflict get a constraint (both branches exist).
  EXPECT_TRUE(saw_agent0_constraint && saw_agent1_constraint);
  // path_found comes after every conflict/constraint event — the solution is only
  // ever emitted once the popped node is conflict-free.
  ASSERT_LT(first_conflict, lines.size());
  EXPECT_GT(first_path_found, last_tree);

  const std::string& last = lines.back();
  EXPECT_NE(last.find("\"event\":\"planning_finished\""), std::string::npos);
  EXPECT_NE(last.find("\"success\":true"), std::string::npos);
  EXPECT_NE(last.find("\"expanded_nodes\":"), std::string::npos);
  EXPECT_NE(last.find("\"makespan\":"), std::string::npos);
  EXPECT_NE(last.find("\"sum_of_costs\":"), std::string::npos);
}
