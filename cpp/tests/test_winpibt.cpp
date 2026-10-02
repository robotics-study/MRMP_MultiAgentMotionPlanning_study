// winPIBT: PIBT's priority inheritance generalized along the time window — every agent
// holds a provisional space-time path, secures its steps one by one in priority order,
// and pulls parked occupants forward (retroactively) before stepping onto their cell. The
// pinned numbers ARE the algorithm and the window IS the axis: at w = 1 the negotiation is
// per-cell again and reproduces PIBT's pinned totals on every shared scenario; at w = 2
// lookahead already buys a step PIBT could not get; at w ≥ 3 the greedy reservation of the
// highest-priority agent walls off the pocket entrance and the swap that PIBT negotiates
// becomes an honest deadlock — prioritized planning's failure mode arriving continuously as
// the window grows. Head-on width-1 corridors deadlock at every window (an edge on no
// cycle defeats every member of this branch). Mirrors python/tests/test_winpibt.py case
// for case.

#include <gtest/gtest.h>

#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include "mrmp/core/params.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/decentralized/winpibt.hpp"
#include "mrmp/maps/loader.hpp"
#include "mrmp/maps/occupancy_grid.hpp"
#include "test_util.hpp"

using namespace mrmp;

namespace {

// winPIBT emits full-horizon paths like the PIBT page: every agent's path spans the whole
// makespan, every step every agent either stays or steps one cell — and across every pair,
// no vertex conflict (same cell at one step) and no swap (pair exchanges cells), which is
// exactly what secured reservations + inheritance make structurally impossible.
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

// The makespan: the arrival step of the last agent (paths span the full horizon).
int makespan_of(const std::vector<std::vector<core::Cell>>& paths) {
  int horizon = 0;
  for (const auto& p : paths) horizon = std::max(horizon, static_cast<int>(p.size()) - 1);
  return horizon;
}

core::ParamSet config() {
  return core::ParamSet::from_yaml(test::repo_path("configs/decentralized/winpibt.yaml"));
}

// The winpibt config with a pinned window/budget (the window is the axis; tests pin it).
core::ParamSet window_config(int window, int max_steps) {
  return core::ParamSet::from_yaml(test::write_temp(
      "winpibt_window.yaml",
      "algorithm: winpibt\nsection: decentralized\nscenarios: []\nparams:\n"
      "  - name: window\n    type: int\n"
      "    default: " +
          std::to_string(window) +
          "\n    min: 1\n    description: test window\n"
      "  - name: max_steps\n    type: int\n"
      "    default: " +
          std::to_string(max_steps) + "\n    min: 1\n    description: test budget\n"));
}

// Load a repo scenario and convert its world-coord endpoints to cells (exactly what the
// demo driver does), so both languages solve identical tasks.
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

TEST(Winpibt, ContractMatchesConfig) {
  decentralized::Winpibt planner(config());
  EXPECT_EQ(planner.name(), config().algorithm());
  EXPECT_TRUE(planner.required_capabilities().count(core::Capability::DISCRETE_SPACE) > 0);
  EXPECT_EQ(planner.required_capabilities().size(), 1u);
}

TEST(Winpibt, SingleAgentWalksTheGradientPinnedByTieBreaks) {
  // One agent, nobody to inherit from: the ideal path is just the pinned BFS parent chain
  // (fixed neighbor order picks column 0 first, then along row 0 — landing on the same
  // route PIBT's candidate sort produced on this grid). The call count pins the window: at
  // w = 2 one winpibt() invocation every second round extends to t + 2.
  decentralized::Winpibt planner(config());
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
  // Rounds t = 0, 2, 4, 6 extend; the goal step lands at t = 8 and the loop stops.
  EXPECT_EQ(r.stats.expanded_nodes, 4);
}

TEST(Winpibt, AlreadyAtGoalIsVacuouslyDone) {
  // start == goal: the round loop never runs; one-step path, zero cost, zero calls.
  decentralized::Winpibt planner(config());
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

TEST(Winpibt, DuplicateAssignmentIsNotAnInstance) {
  // The paper's instance is well-formed by definition (unique starts AND unique goals on
  // passable cells). A violating input is not an instance of this problem at all — the
  // planner reports it honestly instead of running on a contradiction.
  decentralized::Winpibt planner(config());
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{1, 1}},
                                     {core::Cell{0, 0}, core::Cell{0, 1}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
}

TEST(Winpibt, Open01CrossTheWindowChangesNothingHere) {
  // The crossing where PIBT pinned [16, 17]: agent 1 yields at the crossing here too.
  // Nobody ever needs more lookahead on this map, so every window from 1 up pins the same
  // totals — the window is inert when negotiation alone suffices.
  ScenarioSetup s = scenario("open01_cross");
  core::MultiPlanResult r = decentralized::Winpibt(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_simultaneous(r.paths, s.tasks);
  EXPECT_EQ(per_agent_costs(r.paths), std::vector<int>({16, 17}));
  EXPECT_DOUBLE_EQ(r.cost, 33.0);
}

TEST(Winpibt, Open01SwapTheWindowBuysOneStep) {
  // The head-on swap on row 10. At w = 1 the negotiation is per-cell and replays PIBT's
  // run exactly (makespan 17, same [14, 16] split). At the default w = 2 agent 1 sees
  // agent 0's reservation arriving two steps ahead and ducks into row 9 one step earlier —
  // same 30 moves, makespan 16. The window buys foresight, not speed: the cost total is
  // identical, only the schedule tightens.
  ScenarioSetup s = scenario("open01_swap");
  core::MultiPlanResult wide = decentralized::Winpibt(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(wide.success);
  assert_simultaneous(wide.paths, s.tasks);
  EXPECT_EQ(per_agent_costs(wide.paths), std::vector<int>({14, 16}));
  EXPECT_DOUBLE_EQ(wide.cost, 30.0);
  EXPECT_EQ(makespan_of(wide.paths), 16);

  core::MultiPlanResult narrow =
      decentralized::Winpibt(window_config(1, 500)).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(narrow.success);
  assert_simultaneous(narrow.paths, s.tasks);
  EXPECT_EQ(per_agent_costs(narrow.paths), std::vector<int>({14, 16}));  // PIBT's pinned split
  EXPECT_EQ(makespan_of(narrow.paths), 17);                               // ... one step slower
}

TEST(Winpibt, PocketYieldMakesTheSwapExecutable) {
  // The same head-on swap on a map WITH a pocket, at the default window: agent 1 reaches
  // the pocket entrance (1,4) before agent 0's shorter window reserves it, ducks into
  // (0,4), lets agent 0 pass, and walks out the far side — [4, 6], the same totals PIBT
  // pinned. The route inside the pocket is winPIBT's own pinned convention ((1,4) exit, not
  // PIBT's (0,3)); only the totals coincide.
  ScenarioSetup s = scenario("pocket01_swap");
  core::MultiPlanResult r = decentralized::Winpibt(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_simultaneous(r.paths, s.tasks);
  const std::vector<core::Cell> pinned_a0{core::Cell{1, 1}, core::Cell{1, 2}, core::Cell{1, 3},
                                          core::Cell{1, 4}, core::Cell{1, 5}, core::Cell{1, 5},
                                          core::Cell{1, 5}, core::Cell{1, 5}};
  const std::vector<core::Cell> pinned_a1{core::Cell{1, 5}, core::Cell{1, 4}, core::Cell{1, 4},
                                          core::Cell{0, 4}, core::Cell{1, 4}, core::Cell{1, 3},
                                          core::Cell{1, 2}, core::Cell{1, 1}};
  ASSERT_EQ(r.paths.size(), 2u);
  EXPECT_TRUE(r.paths[0] == pinned_a0);
  EXPECT_TRUE(r.paths[1] == pinned_a1);
  EXPECT_EQ(per_agent_costs(r.paths), std::vector<int>({4, 6}));
  EXPECT_DOUBLE_EQ(r.cost, 10.0);
}

TEST(Winpibt, TheWindowIsTheKnifeEdge) {
  // The pocket scenario at w = 3: agent 0 (higher priority) extends its reservation three
  // cells deep on the first round — (1,2),(1,3),(1,4) all secured by step 3 — and the
  // pocket entrance is gone before agent 1 can reach it. Agent 1 stays pinned at (1,5);
  // every later inheritance fails because vacating means swapping with a move that is
  // already secured. Honest deadlock: the greedy window IS prioritized planning, and this
  // is prioritized planning's failure on a tight swap. Same map, same priorities as the
  // passing test above — only the window moved.
  ScenarioSetup s = scenario("pocket01_swap");
  core::MultiPlanResult r =
      decentralized::Winpibt(window_config(3, 500)).plan(s.grid, s.tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
}

TEST(Winpibt, HeadOnCorridorSwapIsHonestlyUnsolvable) {
  // One-wide corridor, agents swapping ends: no cell has two free neighbors at all, so the
  // retreating agent is always cornered against the wall and every claim on its cell
  // backtracks. Honest failure at every window — the same edge-on-no-cycle condition that
  // defeats PIBT; widening the window changes greediness, not topology.
  decentralized::Winpibt planner(config());
  auto grid = test::make_grid({"#######", "#.....#", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 1}, core::Cell{1, 5}},
                                     {core::Cell{1, 5}, core::Cell{1, 1}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
}

TEST(Winpibt, BudgetExhaustionIsHonestAndCounted) {
  // The same deadlock at a tiny budget: failure after exactly max_steps rounds. At w = 2
  // only rounds t ≡ 0 (mod 2) extend anything: round 0 fires both top-level calls, and
  // from round 2 on the higher-priority agent's extension drags the other along through
  // inheritance calls that fail and backtrack — 2 + 3 + 3 = 8 pinned invocations before
  // the budget cuts the run off.
  decentralized::Winpibt planner(window_config(2, 5));
  auto grid = test::make_grid({"#######", "#.....#", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 1}, core::Cell{1, 5}},
                                     {core::Cell{1, 5}, core::Cell{1, 1}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
  EXPECT_EQ(r.stats.expanded_nodes, 8);
}

TEST(Winpibt, TraceEventsAndMetrics) {
  // No search frontier exists here either: the trace carries NO node_expanded events —
  // path_found (full-horizon, one event per agent in index order) then planning_finished
  // only. The execution replay IS the demo for this branch.
  std::ostringstream os;
  auto grid = test::make_grid({"###..##", "#.....#", "#######"});
  std::vector<core::AgentTask> tasks{{core::Cell{1, 1}, core::Cell{1, 5}},
                                     {core::Cell{1, 5}, core::Cell{1, 1}}};
  core::TraceRecorder rec(os);
  decentralized::Winpibt planner(config());
  core::MultiPlanResult r = planner.plan(grid, tasks, &rec);
  ASSERT_TRUE(r.success);

  std::istringstream in(os.str());
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(in, line)) lines.push_back(line);
  // The planner itself emits exactly three events; planning_started is the demo runner's
  // line, not the planner's (same split as every other algorithm here).
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
  // one top-level call per extension round (t = 0, 2, 4, 6) and no inheritance here: the
  // ducking agent moves before its cell is ever claimed
  EXPECT_NE(last.find("\"expanded_nodes\":9"), std::string::npos);
}
