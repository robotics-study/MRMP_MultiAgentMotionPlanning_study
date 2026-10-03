// RHCR: the hybrid folded into time — a Windowed MAPF solver (plain CBS inside) re-run
// from the ACTUAL positions every h steps, resolving only collisions whose arrival step
// falls in (t_now, t_now + w]. The pinned numbers ARE the algorithm: when the window
// covers an instance's whole horizon a single episode IS plain CBS — field for field,
// expanded_nodes included — and rolling at h = 1 on top of that window replays the very
// same trajectory. Narrow the window and the myopia is the point: at w = 1 every conflict
// is resolved by a wait that merely defers itself into the next window, so head-on swaps
// pin both agents forever (honest budget stop, never a verdict) and even open01_swap goes
// suboptimal. Mirrors python/tests/test_rhcr.py case for case.

#include <gtest/gtest.h>

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mrmp/core/params.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/maps/loader.hpp"
#include "mrmp/maps/occupancy_grid.hpp"
#include "mrmp/search/cbs.hpp"
#include "mrmp/search/rhcr.hpp"
#include "test_util.hpp"

using namespace mrmp;

namespace {

// Full-horizon contract of an EXECUTED trajectory (the decentralized branch's shape):
// every path spans the makespan, every step stays or steps one cell, and across every
// pair no vertex conflict and no swap at any step.
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

core::ParamSet config() { return core::ParamSet::from_yaml(test::repo_path("configs/search/rhcr.yaml")); }

// The real CBS config (its own file — the rhcr config has no max_ct_expansions, and the
// comparison side of the equivalence test must be plain CBS with its own pinned budget).
core::ParamSet cbs_config() { return core::ParamSet::from_yaml(test::repo_path("configs/search/cbs.yaml")); }

// The rhcr config with a pinned window/period/budget (the window is the axis; tests pin it).
core::ParamSet window_config(int window, int period, int max_steps) {
  return core::ParamSet::from_yaml(test::write_temp(
      "rhcr_window.yaml",
      "algorithm: rhcr\nsection: search\nscenarios: []\nparams:\n"
      "  - name: window\n    type: int\n"
      "    default: " +
          std::to_string(window) +
          "\n    min: 1\n    description: test window\n"
      "  - name: replan_period\n    type: int\n"
      "    default: " + std::to_string(period) +
          "\n    min: 1\n    description: test period\n"
      "  - name: max_steps\n    type: int\n"
      "    default: " + std::to_string(max_steps) +
          "\n    min: 1\n    description: test budget\n"));
}

// Load a repo scenario and convert its world-coord endpoints to cells (exactly what
// the demo driver does), so both languages solve identical tasks.
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

int makespan_of(const std::vector<std::vector<core::Cell>>& paths) {
  int m = 0;
  for (const auto& p : paths) m = std::max(m, static_cast<int>(p.size()) - 1);
  return m;
}

}  // namespace

TEST(Rhcr, ContractMatchesConfig) {
  search::Rhcr planner(config());
  EXPECT_EQ(planner.name(), config().algorithm());
  EXPECT_TRUE(planner.required_capabilities().count(core::Capability::DISCRETE_SPACE) > 0);
  EXPECT_EQ(planner.required_capabilities().size(), 1u);
  // The window and its replanning period are the axis; the budget is the honest stop.
  EXPECT_EQ(config().get_int("window"), 2);
  EXPECT_EQ(config().get_int("replan_period"), 1);
  EXPECT_EQ(config().get_int("max_steps"), 64);
}

TEST(Rhcr, ReplanPeriodAboveWindowThrows) {
  // w >= h is the safety condition itself: executing a step no resolved window ever
  // covered is not a harder problem, it is an unsafe one — throw, never clamp. Each
  // parameter alone validates fine (both are ints >= 1); only plan() can see the pair.
  search::Rhcr planner(window_config(2, 3, 64));
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{1, 1}},
                                     {core::Cell{1, 0}, core::Cell{0, 1}}};
  EXPECT_THROW(planner.plan(grid, tasks, nullptr), std::runtime_error);
}

TEST(Rhcr, SingleAgentIsPlainAStarRolled) {
  // One agent, nobody to conflict with: every episode's windowed search is the same
  // tie-break-pinned A* wave and h = 1 re-solves from each executed step. The executed
  // trajectory is exactly the straight-up-then-right line the CBS page pins — rolling
  // changes the route's SHAPE not at all, only how often it is recomputed.
  search::Rhcr planner(config());
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
  EXPECT_EQ(r.stats.expanded_nodes, 84);
}

TEST(Rhcr, AlreadyAtGoalIsVacuouslyDone) {
  // start == goal on a one-agent instance: the loop's first check is already true, no
  // episode ever runs — zero expansions, one-step path, zero cost.
  search::Rhcr planner(config());
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

TEST(Rhcr, DuplicateAssignmentIsNotAnInstance) {
  // Same well-formedness rule as the decentralized branch: a t=0 collision is history,
  // not a conflict any window could resolve — fail before episode zero.
  search::Rhcr planner(config());
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{1, 1}},
                                     {core::Cell{0, 0}, core::Cell{0, 1}}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
  EXPECT_EQ(r.stats.expanded_nodes, 0);
}

TEST(Rhcr, WindowCoveringTheHorizonIsExactlyCbs) {
  // The generalization pinned at its limit. With w = 99 the single episode's window
  // covers every horizon here, so RHCR IS plain CBS — same tie-breaks, so not merely
  // the same optimum but the SAME paths (cut each padded trajectory at its own first
  // arrival) and the SAME honest expansion count. The pinned numbers are CBS's own.
  search::Cbs cbs(cbs_config());
  for (const std::string name : {"open01_cross", "open01_swap", "maze01_two"}) {
    ScenarioSetup s = scenario(name);
    core::MultiPlanResult single = search::Rhcr(window_config(99, 99, 64)).plan(s.grid, s.tasks, nullptr);
    core::MultiPlanResult joint = cbs.plan(s.grid, s.tasks, nullptr);
    ASSERT_TRUE(single.success) << name;
    ASSERT_TRUE(joint.success) << name;
    // Same paths: RHCR pads to the makespan; cut at each agent's own arrival.
    ASSERT_EQ(single.paths.size(), joint.paths.size());
    for (size_t k = 0; k < joint.paths.size(); ++k) {
      const std::vector<core::Cell> cut(single.paths[k].begin(),
                                       single.paths[k].begin() +
                                           static_cast<std::ptrdiff_t>(joint.paths[k].size()));
      EXPECT_TRUE(cut == joint.paths[k]) << name << " agent " << k;
    }
    EXPECT_DOUBLE_EQ(single.cost, joint.cost) << name;
    EXPECT_EQ(single.stats.expanded_nodes, joint.stats.expanded_nodes) << name;
  }
}

TEST(Rhcr, RollingReplaysTheSingleEpisodeTrajectory) {
  // Re-solving from the actual positions every single step replays EXACTLY the
  // trajectory the one-shot episode produced (the sum-of-costs objective is what makes
  // rolling free — each re-solve lands on the same optimal continuation). What rolling
  // is never free of is the price: 3112 low-level pops instead of 574.
  ScenarioSetup s = scenario("open01_swap");
  core::MultiPlanResult one_shot = search::Rhcr(window_config(99, 99, 64)).plan(s.grid, s.tasks, nullptr);
  core::MultiPlanResult rolled = search::Rhcr(window_config(99, 1, 64)).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(rolled.success && one_shot.success);
  EXPECT_TRUE(rolled.paths == one_shot.paths);
  EXPECT_DOUBLE_EQ(rolled.cost, 30.0);
  EXPECT_DOUBLE_EQ(one_shot.cost, 30.0);
  EXPECT_EQ(rolled.stats.expanded_nodes, 3112);
  EXPECT_EQ(one_shot.stats.expanded_nodes, 574);
}

TEST(Rhcr, Open01CrossTheWindowIsInertHere) {
  // The crossing the whole branch pins at [16, 17] = 33: timing already separates the
  // pair at (10, 9), so no window ever sees a conflict and every width from 1 up pins
  // the SAME run down to the expansion count — on this map the window is inert by
  // geometry, not by size.
  ScenarioSetup s = scenario("open01_cross");
  for (int w : {1, 2}) {
    core::MultiPlanResult r = search::Rhcr(window_config(w, 1, 64)).plan(s.grid, s.tasks, nullptr);
    ASSERT_TRUE(r.success) << "w=" << w;
    assert_simultaneous(r.paths, s.tasks);
    EXPECT_EQ(per_agent_costs(r.paths), std::vector<int>({16, 17}));
    EXPECT_DOUBLE_EQ(r.cost, 33.0);
    EXPECT_EQ(makespan_of(r.paths), 17);
    EXPECT_EQ(r.stats.expanded_nodes, 323);
  }
}

TEST(Rhcr, Open01SwapTheWindowIsTheAxis) {
  // The head-on swap on row 10. At the default w = 2 the conflict enters a window while
  // both agents can still route around each other: agent 0's straight line is never
  // constrained and agent 1... no — agent 0 detours (16 moves) while agent 1 goes
  // straight (14): the joint optimum 30, makespan 16. At w = 1 every conflict arrives one
  // step late: the only resolution left is a wait that defers itself into the next
  // window, and the alternating waits cost a step nobody can recover — 31, makespan 17.
  ScenarioSetup s = scenario("open01_swap");

  core::MultiPlanResult wide = search::Rhcr(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(wide.success);
  assert_simultaneous(wide.paths, s.tasks);
  EXPECT_EQ(per_agent_costs(wide.paths), std::vector<int>({16, 14}));
  EXPECT_DOUBLE_EQ(wide.cost, 30.0);
  EXPECT_EQ(makespan_of(wide.paths), 16);
  EXPECT_EQ(wide.stats.expanded_nodes, 576);

  core::MultiPlanResult narrow = search::Rhcr(window_config(1, 1, 64)).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(narrow.success);
  assert_simultaneous(narrow.paths, s.tasks);
  EXPECT_EQ(per_agent_costs(narrow.paths), std::vector<int>({16, 14}));  // the detour still happens...
  EXPECT_DOUBLE_EQ(narrow.cost, 31.0);                                    // ...but deferral costs one more
  EXPECT_EQ(makespan_of(narrow.paths), 17);
  EXPECT_EQ(narrow.stats.expanded_nodes, 376);
}

TEST(Rhcr, PocketAndTeeTheWindowIsTheEdgeOfSolvable) {
  // The head-on swaps with a pocket / junction: solvable only if someone sees the
  // conflict EARLY enough for the constrained-optimal re-plan to route around instead of
  // waiting. At w = 1 the wait is always locally cheaper and the alternating constraints
  // pin both agents until the budget expires — an honest stop, never a verdict (the same
  // instances solve at the default window). Both maps pin the exact same stall cost:
  // pinned adjacent, every episode spends the same handful of pops.
  struct Pin { const char* name; double cost; int expanded; };
  for (const Pin pin : {Pin{"pocket01_swap", 10.0, 165}, Pin{"tee01_head_on", 11.0, 220}}) {
    ScenarioSetup s = scenario(pin.name);
    core::MultiPlanResult r = search::Rhcr(config()).plan(s.grid, s.tasks, nullptr);
    ASSERT_TRUE(r.success) << pin.name;
    assert_simultaneous(r.paths, s.tasks);
    EXPECT_EQ(per_agent_costs(r.paths), std::vector<int>({4, 6}));
    EXPECT_DOUBLE_EQ(r.cost, pin.cost) << pin.name;
    EXPECT_EQ(makespan_of(r.paths), 6) << pin.name;
    EXPECT_EQ(r.stats.expanded_nodes, pin.expanded) << pin.name;

    core::MultiPlanResult stalled = search::Rhcr(window_config(1, 1, 64)).plan(s.grid, s.tasks, nullptr);
    EXPECT_FALSE(stalled.success) << pin.name;
    EXPECT_TRUE(stalled.paths.empty());
    EXPECT_EQ(stalled.stats.expanded_nodes, 1895) << pin.name;
  }
}

TEST(Rhcr, AParkedGoalCrossedLaterIsStillConflicted) {
  // A parked agent occupies its goal cell FOREVER, so a collision there can land beyond
  // the earliest path end. The scan horizon is min(t_now + w, MAX over paths of size - 1):
  // cut it at the MIN instead and agent 0's one-step parked path pins the horizon at
  // t_now forever — every scan comes up empty and the crossing is COMMITTED verbatim: a
  // "successful" run whose executed trajectory collides at step 2 (this exact bug shipped
  // once in this mirror; this test is its vaccine). At w = 2 the deferred conflict stays
  // inside the two-step window, so the tree keeps paying for waits until agent 0 hops out
  // of its own goal cell and back. At w = 1 every wait defers the collision one step
  // OUTSIDE the window, the hop never wins, and the budget expires honestly.
  auto grid = test::make_grid({".....", "...##"});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 2}, core::Cell{0, 2}},
                                     {core::Cell{0, 4}, core::Cell{0, 0}}};

  core::MultiPlanResult wide = search::Rhcr(window_config(2, 1, 64)).plan(grid, tasks, nullptr);
  ASSERT_TRUE(wide.success);
  assert_simultaneous(wide.paths, tasks);  // fails loudly on a committed collision
  const std::vector<core::Cell> parked{core::Cell{0, 2}, core::Cell{0, 2}, core::Cell{1, 2},
                                       core::Cell{0, 2}, core::Cell{0, 2}};
  const std::vector<core::Cell> crosser{core::Cell{0, 4}, core::Cell{0, 3}, core::Cell{0, 2},
                                        core::Cell{0, 1}, core::Cell{0, 0}};
  ASSERT_EQ(wide.paths.size(), 2u);
  EXPECT_TRUE(wide.paths[0] == parked);
  EXPECT_TRUE(wide.paths[1] == crosser);
  EXPECT_EQ(per_agent_costs(wide.paths), std::vector<int>({2, 4}));
  EXPECT_DOUBLE_EQ(wide.cost, 4.0);
  EXPECT_EQ(makespan_of(wide.paths), 4);
  EXPECT_EQ(wide.stats.expanded_nodes, 61);

  core::MultiPlanResult narrow = search::Rhcr(window_config(1, 1, 64)).plan(grid, tasks, nullptr);
  EXPECT_FALSE(narrow.success);
  EXPECT_TRUE(narrow.paths.empty());
  EXPECT_EQ(narrow.stats.expanded_nodes, 951);
}

TEST(Rhcr, Maze01TwoTheCorridorMeetingNeedsNoForesight) {
  // The two-room maze pins 66/33 at every window (the meeting point sits where a wait
  // alone resolves the head-on — foresight buys nothing here either), and the rolling
  // re-solve pays for its foresight in expansions.
  ScenarioSetup s = scenario("maze01_two");
  core::MultiPlanResult r = search::Rhcr(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_simultaneous(r.paths, s.tasks);
  EXPECT_EQ(per_agent_costs(r.paths), std::vector<int>({33, 33}));
  EXPECT_DOUBLE_EQ(r.cost, 66.0);
  EXPECT_EQ(makespan_of(r.paths), 33);
  EXPECT_EQ(r.stats.expanded_nodes, 13958);
}

TEST(Rhcr, CorridorHeadOnStallsAtEveryWindow) {
  // Width-1 corridor, swapping ends: no joint plan exists at ANY window (the same
  // tree-graph argument that defeats Push and Swap). Every episode resolves its window by
  // pinning both agents in place; the loop stops honestly at the budget. Pinned at a
  // small budget because every step re-runs a whole windowed CBS — the count is the
  // honest price of 8 episodes, not a verdict.
  ScenarioSetup s = scenario("corridor01_head_on");
  core::MultiPlanResult r = search::Rhcr(window_config(2, 1, 8)).plan(s.grid, s.tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
  EXPECT_EQ(r.stats.expanded_nodes, 976);
}

TEST(Rhcr, BudgetExhaustionIsNotAVerdict) {
  // The SAME solvable cross that succeeds at the shipped config stops after episode
  // zero's root expansions when max_steps = 1 (t_now starts at 0, so the budget is
  // already spent before a single step commits). A budget stop says "no execution within
  // budget", never "unsolvable" — the windowed instance was always satisfiable.
  ScenarioSetup s = scenario("open01_cross");
  core::MultiPlanResult r = search::Rhcr(window_config(2, 1, 1)).plan(s.grid, s.tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
  // Exactly the root's two unconstrained A* expansions — 35, the same number the
  // single-episode CBS-equivalent run spends on this scenario in total.
  EXPECT_EQ(r.stats.expanded_nodes, 35);
}

TEST(Rhcr, TraceShowsTheRollingTreeLifecycle) {
  // A crossing pair on a 3x3 grid. The window covers the whole horizon here, so the
  // FIRST episode already resolves the crossing — and because rolling replans from the
  // executed positions with NO constraints carried forward, the same collision is
  // rediscovered and re-resolved in the next episode at the SAME absolute step: two
  // conflict_found events for cell (2, 2) at t = 2, each branched on both agents. Times
  // are ABSOLUTE steps throughout; path_found appears only once execution ends.
  std::ostringstream os;
  auto grid = test::make_grid({"...", "...", "..."});
  std::vector<core::AgentTask> tasks{{core::Cell{2, 0}, core::Cell{2, 2}},
                                     {core::Cell{0, 2}, core::Cell{2, 0}}};
  core::TraceRecorder rec(os);
  search::Rhcr planner(config());
  core::MultiPlanResult r = planner.plan(grid, tasks, &rec);
  ASSERT_TRUE(r.success);
  EXPECT_DOUBLE_EQ(r.cost, 6.0);

  std::istringstream in(os.str());
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(in, line)) lines.push_back(line);
  ASSERT_FALSE(lines.empty());

  int expansions = 0, conflicts = 0, constraints = 0;
  size_t last_tree = 0, first_path_found = lines.size();
  for (size_t i = 0; i + 1 < lines.size(); ++i) {
    const std::string& l = lines[i];
    if (l.find("\"event\":\"node_expanded\"") != std::string::npos) {
      // Per-agent space-time expansion: every event names its agent and ABSOLUTE step.
      EXPECT_NE(l.find("\"agent\":"), std::string::npos);
      EXPECT_NE(l.find("\"t\":"), std::string::npos);
      ++expansions;
    }
    if (l.find("\"event\":\"conflict_found\"") != std::string::npos) {
      // The same collision, rediscovered once per episode until its step is behind them.
      EXPECT_NE(l.find("\"kind\":\"vertex\""), std::string::npos);
      EXPECT_NE(l.find("\"cell\":[2,2]"), std::string::npos);
      EXPECT_NE(l.find("\"t\":2"), std::string::npos);
      EXPECT_NE(l.find("\"agents\":[0,1]"), std::string::npos);
      ++conflicts;
    }
    if (l.find("\"event\":\"constraint_added\"") != std::string::npos) {
      // Both branches of every conflict exist, in pair order.
      EXPECT_NE(l.find("\"t\":2"), std::string::npos);
      ++constraints;
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
  EXPECT_EQ(conflicts, 2);   // once per episode — the rolling re-discovery
  EXPECT_EQ(constraints, 4);  // both agents of the pair, twice

  // path_found comes after every conflict/constraint event — the executed trajectory is
  // emitted once, when execution stops on the simultaneous-co-presence step.
  ASSERT_LT(first_path_found, lines.size());
  EXPECT_GT(first_path_found, last_tree);

  const std::string& last = lines.back();
  EXPECT_NE(last.find("\"event\":\"planning_finished\""), std::string::npos);
  EXPECT_NE(last.find("\"success\":true"), std::string::npos);
  EXPECT_NE(last.find("\"expanded_nodes\":49"), std::string::npos);
  EXPECT_NE(last.find("\"makespan\":4"), std::string::npos);
  EXPECT_NE(last.find("\"sum_of_costs\":6"), std::string::npos);
}
