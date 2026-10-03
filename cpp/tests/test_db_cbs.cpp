// db-CBS: CBS whose low level has momentum. The pins below ARE the lattice — at
// delta < 1 (floor 0) a move is exactly x' = x + v' with |v' - v| <= 1 per axis, so a
// vmax=1 agent moves like the search branch's 4-connected... no: like its 8-connected
// cousin (every axis independently steps), while a vmax=2 agent visibly ramps (the
// first step off rest is ONE cell, never two). At delta >= 1 physics negotiates: the
// landing may miss the ideal landing by one Manhattan unit and velocity jumps by up to
// one — the pinned single-agent path moves two cells on its very first step.
//
// The branch's conflict semantics differ from the search branch on purpose: only
// co-presence at a sampled step is a conflict (kind "vertex" always; there is no edge
// kind here because point robots sampled at integer steps pass through each other —
// the corridor swap that CBS honestly fails IS SOLVABLE here and pinned so below).
// The ladder [1.5, 0.5] runs two independent fresh trees and keeps the last rung that
// solved; expanded_nodes accumulates across rungs, which is why the default-config
// pins are larger than single-rung pins on the same scenario. Mirrors
// python/tests/test_db_cbs.py case for case.

#include <gtest/gtest.h>

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mrmp/core/params.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/kinodynamic/db_cbs.hpp"
#include "mrmp/maps/loader.hpp"
#include "mrmp/maps/occupancy_grid.hpp"
#include "test_util.hpp"

using namespace mrmp;

namespace {

// Stay-at-goal occupancy of a finished path (same rule as the planner).
core::Cell occ(const std::vector<core::Cell>& path, int t) {
  return static_cast<size_t>(t) < path.size() ? path[static_cast<size_t>(t)] : path.back();
}

// Every step's displacement is within that agent's integer velocity bound and changed
// by at most 1 per axis since the previous step (the EXACT double integrator — these
// pins are all read out of delta < 1 rungs), and no two agents ever co-occupy a cell
// at the same step. Swaps across a step are NOT checked: this branch has no edge
// conflicts by design.
void assert_joint_valid(const std::vector<std::vector<core::Cell>>& paths,
                        const std::vector<double>& vmaxes) {
  ASSERT_FALSE(paths.empty());
  int horizon = 0;
  for (const auto& p : paths) horizon = std::max(horizon, static_cast<int>(p.size()) - 1);
  for (size_t k = 0; k < paths.size(); ++k) {
    bool has_prev = false;
    core::Cell v_prev{0, 0};
    const auto& path = paths[k];
    for (size_t t = 1; t < path.size(); ++t) {
      const core::Cell v{path[t].row - path[t - 1].row, path[t].col - path[t - 1].col};
      ASSERT_LE(std::max(std::abs(v.row), std::abs(v.col)), vmaxes[k]) << "agent " << k << " step " << t << ": |v| > vmax";
      if (has_prev) {
        const core::Cell dv{std::abs(v.row - v_prev.row), std::abs(v.col - v_prev.col)};
        ASSERT_LE(std::max(dv.row, dv.col), 1) << "agent " << k << " step " << t << ": acceleration exceeds 1";
      }
      v_prev = v;
      has_prev = true;
    }
  }
  for (size_t i = 0; i < paths.size(); ++i) {
    for (size_t j = i + 1; j < paths.size(); ++j) {
      for (int t = 0; t <= horizon; ++t) {
        ASSERT_TRUE(!(occ(paths[i], t) == occ(paths[j], t))) << "co-presence at step " << t;
      }
    }
  }
}

// True iff some pair swaps cells across a step — legal here, impossible for the search
// branch. The corridor pin below asserts this is what makes it solvable.
bool swap_at(const std::vector<std::vector<core::Cell>>& paths) {
  int horizon = 0;
  for (const auto& p : paths) horizon = std::max(horizon, static_cast<int>(p.size()) - 1);
  for (size_t i = 0; i < paths.size(); ++i) {
    for (size_t j = i + 1; j < paths.size(); ++j) {
      for (int t = 1; t <= horizon; ++t) {
        if (occ(paths[i], t) == occ(paths[j], t - 1) && occ(paths[j], t) == occ(paths[i], t - 1)) {
          return true;
        }
      }
    }
  }
  return false;
}

struct ScenarioSetup {
  maps::OccupancyGrid2D grid;
  std::vector<core::AgentTask> tasks;
};

// Load a repo scenario and convert its world-coord endpoints to cells, carrying each
// agent's velocity limit along (this branch is the only one that reads vmax).
ScenarioSetup scenario(const std::string& name) {
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/" + name + ".yaml"));
  auto base = maps::load_map(sc.map_path);
  auto* grid = dynamic_cast<maps::OccupancyGrid2D*>(base.get());
  if (grid == nullptr) throw std::runtime_error("scenario map is not an occupancy grid");
  std::vector<core::AgentTask> tasks;
  for (const auto& a : sc.agents) {
    tasks.push_back(core::AgentTask{grid->world_to_cell(a.start.x, a.start.y),
                                    grid->world_to_cell(a.goal.x, a.goal.y), a.vmax});
  }
  return ScenarioSetup{std::move(*grid), std::move(tasks)};
}

std::vector<double> vmax_of(const std::vector<core::AgentTask>& tasks) {
  std::vector<double> out;
  for (const auto& t : tasks) out.push_back(t.vmax);
  return out;
}

int makespan_of(const std::vector<std::vector<core::Cell>>& paths) {
  int m = 0;
  for (const auto& p : paths) m = std::max(m, static_cast<int>(p.size()) - 1);
  return m;
}

core::ParamSet config() {
  return core::ParamSet::from_yaml(test::repo_path("configs/kinodynamic/db_cbs.yaml"));
}

// A three-param config decl so tests can pin a rung/budget without editing the shipped
// configs/kinodynamic/db_cbs.yaml.
core::ParamSet ladder_config(double delta_start, double delta_end, int max_ct = 256) {
  return core::ParamSet::from_yaml(test::write_temp(
      "db_cbs.yaml",
      "algorithm: db_cbs\nsection: kinodynamic\nscenarios: []\nparams:\n"
      "  - name: delta_start\n    type: float\n    default: " +
          std::to_string(delta_start) +
          "\n    min: 0.5\n    max: 1.5\n    description: test ladder start\n"
      "  - name: delta_end\n    type: float\n    default: " +
          std::to_string(delta_end) +
          "\n    min: 0.5\n    max: 1.5\n    description: test ladder end\n"
      "  - name: max_ct_expansions\n    type: int\n    default: " +
          std::to_string(max_ct) +
          "\n    min: 1\n    description: test budget\n"));
}

}  // namespace

TEST(DbCbs, ContractMatchesConfig) {
  kinodynamic::DbCbs planner(config());
  EXPECT_EQ(planner.name(), config().algorithm());
  EXPECT_TRUE(planner.required_capabilities().count(core::Capability::DISCRETE_SPACE) > 0);
  EXPECT_EQ(planner.required_capabilities().size(), 1u);
  // The ladder is the axis (loose -> exact) and the budget is the honest stop.
  EXPECT_DOUBLE_EQ(config().get_float("delta_start"), 1.5);
  EXPECT_DOUBLE_EQ(config().get_float("delta_end"), 0.5);
  EXPECT_EQ(config().get_int("max_ct_expansions"), 256);
}

TEST(DbCbs, LadderOnlyTightensThrows) {
  // A widening bound is not what the paper's loop does. Each parameter alone validates
  // fine (both are floats in [0.5, 1.5]); only plan() can see the pair.
  kinodynamic::DbCbs planner(ladder_config(0.5, 1.5));
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{1, 1}, 1.0}};
  EXPECT_THROW(planner.plan(grid, tasks, nullptr), std::runtime_error);
}

TEST(DbCbs, NonIntegerVmaxRaises) {
  // The velocity lattice quantizes integers only — a fractional limit is not a finer
  // model here, it is a different model the lattice cannot express.
  kinodynamic::DbCbs planner(config());
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{1, 1}, 0.25}};
  EXPECT_THROW(planner.plan(grid, tasks, nullptr), std::runtime_error);
}

TEST(DbCbs, SingleAgentExactRungIsTheDoubleIntegrator) {
  // One agent, no conflicts: the root's state-space A* IS the answer. At delta < 1 a
  // move is exactly x' = x + v' with |v' - v| <= 1 per axis — at vmax=1 every
  // 8-neighbor move plus wait is legal (acceleration vacuous), and the pinned path is
  // the pure diagonal. expanded_nodes counts every state-space pop (no CT branching
  // here — the root's sub-search IS the whole run).
  kinodynamic::DbCbs planner(ladder_config(0.5, 0.5));
  auto grid = test::make_grid({".....", ".....", ".....", ".....", "....."});
  std::vector<core::AgentTask> tasks{{core::Cell{4, 0}, core::Cell{0, 4}, 1.0}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  const std::vector<core::Cell> pinned{core::Cell{4, 0}, core::Cell{3, 1}, core::Cell{2, 2},
                                       core::Cell{1, 3}, core::Cell{0, 4}};
  ASSERT_EQ(r.paths.size(), 1u);
  EXPECT_TRUE(r.paths[0] == pinned);
  EXPECT_DOUBLE_EQ(r.cost, 4.0);
  EXPECT_EQ(r.stats.expanded_nodes, 5);
}

TEST(DbCbs, SingleAgentVmax2RampsFromRest) {
  // vmax=2 on the same straight shot: acceleration is still 1 per axis, so the first
  // step off rest covers ONE cell and only later steps may cover two — velocity ramps,
  // it never jumps. Cruise speed 2 still wins the trip (cost 3).
  kinodynamic::DbCbs planner(ladder_config(0.5, 0.5));
  auto grid = test::make_grid({".....", ".....", ".....", ".....", "....."});
  std::vector<core::AgentTask> tasks{{core::Cell{4, 0}, core::Cell{0, 4}, 2.0}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  // First step one cell per axis (ramp), then the velocity is free to cover two.
  const std::vector<core::Cell> pinned{core::Cell{4, 0}, core::Cell{3, 1}, core::Cell{1, 2},
                                       core::Cell{0, 4}};
  ASSERT_EQ(r.paths.size(), 1u);
  EXPECT_TRUE(r.paths[0] == pinned);
  EXPECT_DOUBLE_EQ(r.cost, 3.0);
  EXPECT_EQ(r.stats.expanded_nodes, 10);
}

TEST(DbCbs, LooseRungMakesPhysicsNegotiable) {
  // Same instance at delta >= 1: the chaining gap covers one extra cell per axis, so
  // the very first step already covers two cells and the trip costs one step less. The
  // ladder is not a refinement of the exact model — it is a DIFFERENT (negotiable)
  // physics, which is why rungs are independent attempts.
  kinodynamic::DbCbs planner(ladder_config(1.5, 1.5));
  auto grid = test::make_grid({".....", ".....", ".....", ".....", "....."});
  std::vector<core::AgentTask> tasks{{core::Cell{4, 0}, core::Cell{0, 4}, 1.0}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  const std::vector<core::Cell> pinned{core::Cell{4, 0}, core::Cell{2, 0}, core::Cell{1, 2},
                                       core::Cell{0, 4}};
  ASSERT_EQ(r.paths.size(), 1u);
  EXPECT_TRUE(r.paths[0] == pinned);
  EXPECT_DOUBLE_EQ(r.cost, 3.0);
  EXPECT_EQ(r.stats.expanded_nodes, 94);
}

TEST(DbCbs, Open01CrossMomentumTimesTheCrossing) {
  // The textbook cross on the timed scenario: at vmax=1/vmax=2 with exact physics (the
  // ladder's last rung) the two individually-optimal state-paths already miss each
  // other in time-space — NO conflict fires, and agent 1's vmax=2 ramp is visible in
  // its pinned path (one cell out of rest, then two). The constraint tree never
  // branches here; this scenario pins the motion model itself.
  ScenarioSetup s = scenario("open01_cross_timed");
  core::MultiPlanResult r = kinodynamic::DbCbs(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths, vmax_of(s.tasks));
  const std::vector<core::Cell> pinned_0{
      core::Cell{10, 1},  core::Cell{9, 2},  core::Cell{8, 3},  core::Cell{8, 4},
      core::Cell{9, 5},   core::Cell{9, 6},  core::Cell{9, 7},  core::Cell{9, 8},
      core::Cell{8, 9},   core::Cell{8, 10}, core::Cell{8, 11}, core::Cell{9, 12},
      core::Cell{10, 13}, core::Cell{10, 14}, core::Cell{10, 15}, core::Cell{10, 16},
      core::Cell{10, 17}};
  const std::vector<core::Cell> pinned_1{core::Cell{1, 9}, core::Cell{2, 9}, core::Cell{4, 9},
                                         core::Cell{6, 9}, core::Cell{8, 9}, core::Cell{10, 8},
                                         core::Cell{12, 7}, core::Cell{14, 7}, core::Cell{16, 8},
                                         core::Cell{18, 9}};
  ASSERT_EQ(r.paths.size(), 2u);
  EXPECT_TRUE(r.paths[0] == pinned_0);
  EXPECT_TRUE(r.paths[1] == pinned_1);
  EXPECT_DOUBLE_EQ(r.cost, 25.0);
  EXPECT_EQ(makespan_of(r.paths), 16);
  // Both rungs' expansions accumulate; the exact rung's answer stands alone too.
  EXPECT_EQ(r.stats.expanded_nodes, 604);
}

TEST(DbCbs, Open01SwapPassesThroughInsteadOfDetouring) {
  // The head-on swap the search branch must detour (CBS pays 30 for it): here a swap
  // across a step is NOT a conflict — sampled point robots pass through each other —
  // so both agents keep their individually-optimal paths and the crossing needs no
  // wait at all. Cost 28 = 14 + 14, below CBS's 30: the pair slides past on
  // diagonally-offset lanes without ever sharing a cell at one step.
  ScenarioSetup s = scenario("open01_swap_timed");
  core::MultiPlanResult r = kinodynamic::DbCbs(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths, vmax_of(s.tasks));
  EXPECT_DOUBLE_EQ(r.cost, 28.0);
  EXPECT_EQ(makespan_of(r.paths), 14);
}

TEST(DbCbs, PocketSwapPinsBothPaths) {
  // The pocket scenario under velocity: agent 1 ducks through the pocket cells —
  // pinned field-for-field so every language replays the same choice.
  ScenarioSetup s = scenario("pocket01_swap_timed");
  core::MultiPlanResult r = kinodynamic::DbCbs(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths, vmax_of(s.tasks));
  const std::vector<core::Cell> pinned_0{core::Cell{1, 1}, core::Cell{1, 2}, core::Cell{1, 3},
                                         core::Cell{1, 4}, core::Cell{1, 5}};
  const std::vector<core::Cell> pinned_1{core::Cell{1, 5}, core::Cell{0, 4}, core::Cell{0, 3},
                                         core::Cell{1, 2}, core::Cell{1, 1}};
  ASSERT_EQ(r.paths.size(), 2u);
  EXPECT_TRUE(r.paths[0] == pinned_0);
  EXPECT_TRUE(r.paths[1] == pinned_1);
  EXPECT_DOUBLE_EQ(r.cost, 8.0);
  EXPECT_EQ(makespan_of(r.paths), 4);
}

TEST(DbCbs, CorridorSwapIsSolvableHere) {
  // The width-1 corridor the search branch honestly FAILS (a swap is unavoidable there
  // and CBS forbids edge conflicts): with vertex-only conflicts the pair simply passes
  // through — one waits, they cross without ever sharing a step.
  ScenarioSetup s = scenario("corridor01_head_on_timed");
  core::MultiPlanResult r = kinodynamic::DbCbs(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  assert_joint_valid(r.paths, vmax_of(s.tasks));
  EXPECT_TRUE(swap_at(r.paths));
  EXPECT_DOUBLE_EQ(r.cost, 9.0);
  EXPECT_EQ(makespan_of(r.paths), 5);
}

TEST(DbCbs, LadderRungsAreIndependentFreshTrees) {
  // The ladder runs one fresh CT per rung and keeps the LAST rung that solved — on the
  // cross scenario both rungs solve to the SAME paths (the exact rung's answer is what
  // the default config reports), while expanded_nodes honestly accumulates across both
  // trees.
  ScenarioSetup s = scenario("open01_cross_timed");
  core::MultiPlanResult ladder = kinodynamic::DbCbs(config()).plan(s.grid, s.tasks, nullptr);
  core::MultiPlanResult exact_only =
      kinodynamic::DbCbs(ladder_config(0.5, 0.5)).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(ladder.success);
  ASSERT_TRUE(exact_only.success);
  EXPECT_TRUE(ladder.paths == exact_only.paths);
  EXPECT_GT(ladder.stats.expanded_nodes, exact_only.stats.expanded_nodes);
}

TEST(DbCbs, DuplicateStartFailsByBranchingToDeadEnds) {
  // Two agents on one cell at t=0 is unsolvable — and the tree really does die: both
  // children constrain an agent off its own start cell (instantly dead sub-searches)
  // and the queue empties. An honest verdict, not a budget stop; expanded 8 = the two
  // root sub-searches across both rungs, nothing more.
  kinodynamic::DbCbs planner(config());
  auto grid = test::make_grid({"..", ".."});
  std::vector<core::AgentTask> tasks{{core::Cell{0, 0}, core::Cell{1, 1}, 1.0},
                                     {core::Cell{0, 0}, core::Cell{0, 1}, 1.0}};
  core::MultiPlanResult r = planner.plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
  EXPECT_EQ(r.stats.expanded_nodes, 8);
}

TEST(DbCbs, BudgetExhaustionIsNotAVerdict) {
  // The SAME solvable swap with a budget of 1 fails after the root alone (the CT loop
  // never pops): "no solution found within budget", pinned exactly because every
  // language replays identically.
  ScenarioSetup s = scenario("open01_swap_timed");
  core::MultiPlanResult r = kinodynamic::DbCbs(ladder_config(1.5, 0.5, 1)).plan(s.grid, s.tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.stats.expanded_nodes, 498);
}

TEST(DbCbs, TraceShowsTheConstraintTreeLifecycle) {
  // A swapping pair forces branching on the exact rung: the trace must show per-agent
  // space-time expansions (state = cell pair, cost = t), conflict_found with kind
  // VERTEX only (this branch has no edge kind at any delta), a constraint_added for
  // BOTH agents of the conflicting pair, path_found only for the final solution paths,
  // and planning_finished carrying the answered delta.
  std::ostringstream os;
  ScenarioSetup s = scenario("pocket01_swap_timed");
  core::TraceRecorder rec(os);
  kinodynamic::DbCbs planner(config());
  core::MultiPlanResult r = planner.plan(s.grid, s.tasks, &rec);
  ASSERT_TRUE(r.success);

  std::istringstream in(os.str());
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(in, line)) lines.push_back(line);
  ASSERT_FALSE(lines.empty());

  int expansions = 0, conflicts = 0, constraints = 0;
  bool agent_0_constrained = false, agent_1_constrained = false;
  std::string first_conflict_agents;
  size_t last_tree = 0, first_path_found = lines.size();
  for (size_t i = 0; i + 1 < lines.size(); ++i) {
    const std::string& l = lines[i];
    if (l.find("\"event\":\"node_expanded\"") != std::string::npos) {
      // Per-agent space-time expansion: every event names its agent and ABSOLUTE step,
      // and the state is exactly a cell PAIR.
      EXPECT_NE(l.find("\"agent\":"), std::string::npos);
      EXPECT_NE(l.find("\"t\":"), std::string::npos);
      const size_t state_at = l.find("\"state\":[");
      ASSERT_NE(state_at, std::string::npos);
      const std::string state = l.substr(state_at + 9, l.find(']', state_at) - (state_at + 9));
      EXPECT_EQ(std::count(state.begin(), state.end(), ','), 1) << "state must be a pair: " << state;
      ++expansions;
    }
    if (l.find("\"event\":\"conflict_found\"") != std::string::npos) {
      // Vertex only — this branch has no edge kind at ANY delta.
      EXPECT_NE(l.find("\"kind\":\"vertex\""), std::string::npos);
      EXPECT_EQ(l.find("\"to\""), std::string::npos);
      if (first_conflict_agents.empty()) {
        const size_t at = l.find("\"agents\":[");
        ASSERT_NE(at, std::string::npos);
        first_conflict_agents = l.substr(at + 10, l.find(']', at) - (at + 10));
      }
      ++conflicts;
    }
    if (l.find("\"event\":\"constraint_added\"") != std::string::npos) {
      // Both branches of every conflict exist, in pair order.
      if (l.find("\"agent\":0") != std::string::npos) agent_0_constrained = true;
      if (l.find("\"agent\":1") != std::string::npos) agent_1_constrained = true;
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
  ASSERT_FALSE(first_conflict_agents.empty());
  EXPECT_EQ(first_conflict_agents, "0,1");  // a swapping pair — both agents branch
  EXPECT_TRUE(agent_0_constrained && agent_1_constrained);
  EXPECT_GT(conflicts, 0);
  EXPECT_GT(constraints, 1);

  // path_found comes after every conflict/constraint event — the answer is emitted once,
  // when the ladder has answered.
  ASSERT_LT(first_path_found, lines.size());
  EXPECT_GT(first_path_found, last_tree);

  const std::string& last = lines.back();
  EXPECT_NE(last.find("\"event\":\"planning_finished\""), std::string::npos);
  EXPECT_NE(last.find("\"success\":true"), std::string::npos);
  // The default ladder answers on its exact rung.
  EXPECT_NE(last.find("\"delta\":0.5"), std::string::npos);
  EXPECT_NE(last.find("\"expanded_nodes\":"), std::string::npos);
  EXPECT_NE(last.find("\"makespan\":"), std::string::npos);
  EXPECT_NE(last.find("\"sum_of_costs\":"), std::string::npos);
}
