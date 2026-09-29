// dRRT: seeded determinism (the PRNG stream — roadmap samples first, joint
// samples after — is part of the algorithm's identity), valid collision-free
// joint paths on both disc scenarios, and the two FAILURE verdicts dRRT gives —
// an instant instance verdict (a start/goal disc overlapping an obstacle cell,
// or two start discs overlapping each other: no valid initial configuration
// exists at all), and an honest budget failure that is NOT a verdict. Mirrors
// python/tests/test_drrt.py case for case.

#include <gtest/gtest.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include "mrmp/core/geometry.hpp"
#include "mrmp/core/params.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/maps/loader.hpp"
#include "mrmp/maps/occupancy_grid.hpp"
#include "mrmp/sampling/drrt.hpp"
#include "test_util.hpp"

using namespace mrmp;

namespace {

// Unit-resolution grid from ASCII rows ('.' free, '#' occupied) — the mirror of
// the Python tests' grid_from (resolution 1.0, origin (0,0)).
maps::OccupancyGrid2D unit_grid(const std::vector<std::string>& rows) {
  int h = static_cast<int>(rows.size());
  int w = static_cast<int>(rows.empty() ? 0 : rows[0].size());
  std::vector<bool> free_cells(static_cast<size_t>(h) * w);
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) free_cells[static_cast<size_t>(r) * w + c] = rows[r][c] == '.';
  }
  return maps::OccupancyGrid2D(h, w, 1.0, 0.0, 0.0, std::move(free_cells));
}

// Every waypoint free for its own disc, and every simultaneous motion segment
// pair apart by at least the radius sum (strict overlap is collision).
void assert_joint_valid(const std::vector<std::vector<core::Point>>& paths,
                       const std::vector<double>& radii, const maps::OccupancyGrid2D& grid) {
  for (size_t i = 0; i < paths.size(); ++i) {
    for (const core::Point& p : paths[i]) {
      ASSERT_TRUE(grid.free_point(p, radii[i])) << "agent " << i << " waypoint on an obstacle";
    }
  }
  int horizon = 0;
  for (const auto& p : paths) horizon = std::max(horizon, static_cast<int>(p.size()) - 1);
  for (size_t i = 0; i < paths.size(); ++i) {
    for (size_t j = i + 1; j < paths.size(); ++j) {
      for (int t = 1; t <= horizon; ++t) {
        auto occ = [](const std::vector<core::Point>& p, int k) {
          return static_cast<size_t>(k) < p.size() ? p[static_cast<size_t>(k)] : p.back();
        };
        double d = core::moving_pair_distance(occ(paths[i], t - 1), occ(paths[i], t),
                                              occ(paths[j], t - 1), occ(paths[j], t));
        ASSERT_TRUE(d >= radii[i] + radii[j]) << "discs overlap during step " << t;
      }
    }
  }
}

core::ParamSet config() {
  return core::ParamSet::from_yaml(test::repo_path("configs/sampling/drrt.yaml"));
}

// The declared drrt config with defaults overridden (same declarations,
// different defaults — the loader validates them like the real file).
core::ParamSet override_config(int samples_per_robot, int max_rounds) {
  const std::string params =
      "  - name: seed\n    type: int\n    default: 42\n    min: 1\n"
      "    max: 2147483646\n    description: test seed\n"
      "  - name: samples_per_robot\n    type: int\n    default: " +
      std::to_string(samples_per_robot) +
      "\n    min: 0\n    description: test n\n"
      "  - name: roadmap_k\n    type: int\n    default: 6\n    min: 1\n"
      "    description: test k\n"
      "  - name: max_rounds\n    type: int\n    default: " +
      std::to_string(max_rounds) + "\n    min: 1\n    max: 30\n    description: test budget\n";
  return core::ParamSet::from_yaml(test::write_temp(
      "drrt_override.yaml",
      "algorithm: drrt\nsection: sampling\nscenarios: []\nparams:\n" + params));
}

// Load a repo disc scenario as continuous tasks (world Points + radius ride along).
struct ScenarioDiscs {
  maps::OccupancyGrid2D grid;
  std::vector<core::ContinuousAgentTask> tasks;
};
ScenarioDiscs scenario_discs(const std::string& name) {
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/" + name + ".yaml"));
  auto base = maps::load_map(sc.map_path);
  auto* grid = dynamic_cast<maps::OccupancyGrid2D*>(base.get());
  if (grid == nullptr) throw std::runtime_error("scenario map is not an occupancy grid");
  std::vector<core::ContinuousAgentTask> tasks;
  for (const auto& a : sc.agents) {
    tasks.push_back(core::ContinuousAgentTask{a.start, a.goal, a.radius});
  }
  return ScenarioDiscs{std::move(*grid), std::move(tasks)};
}

}  // namespace

TEST(Drrt, ContractMatchesConfig) {
  sampling::Drrt planner(config());
  EXPECT_EQ(planner.name(), config().algorithm());
  EXPECT_TRUE(planner.required_capabilities().count(core::Capability::CONTINUOUS_SPACE) > 0);
  EXPECT_EQ(planner.required_capabilities().size(), 1u);
}

TEST(Drrt, SameSeedReplaysByteIdenticalPlans) {
  // The MINSTD stream (roadmap samples first, joint samples after) is part of
  // the algorithm's identity: all three language engines must replay identical
  // roadmaps and trees from one seed. Same-seed reruns are the first line.
  ScenarioDiscs s = scenario_discs("open01_cross_discs");
  core::ContinuousPlanResult a = sampling::Drrt(config()).plan(s.grid, s.tasks, nullptr);
  core::ContinuousPlanResult b = sampling::Drrt(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(a.success);
  ASSERT_TRUE(b.success);
  EXPECT_EQ(a.paths, b.paths);
  EXPECT_DOUBLE_EQ(a.cost, b.cost);
  EXPECT_EQ(a.stats.expanded_nodes, b.stats.expanded_nodes);
}

TEST(Drrt, Open01CrossDiscsSolvesWithValidJointPaths) {
  // The crossing at (4.75, 4.75): every tensor edge that would overlap the two
  // discs is rejected by the moving-pair check, so the tree only grows through
  // safe simultaneous motion and the prioritized connector sequences the rest.
  ScenarioDiscs s = scenario_discs("open01_cross_discs");
  core::ContinuousPlanResult r = sampling::Drrt(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  std::vector<double> radii;
  for (const auto& t : s.tasks) radii.push_back(t.radius);
  assert_joint_valid(r.paths, radii, s.grid);
  // Each path starts at its start point and ends at its goal point.
  for (size_t i = 0; i < s.tasks.size(); ++i) {
    EXPECT_TRUE(r.paths[i].front() == s.tasks[i].start);
    EXPECT_TRUE(r.paths[i].back() == s.tasks[i].goal);
  }
}

TEST(Drrt, Open01SwapDiscsSolvesWithValidJointPaths) {
  // Head-on swap on the open plane: no timing alone separates two discs on one
  // line, so the accepted tree edges route around each other through sampled
  // vertices and/or the priority DAG sequences the pass.
  ScenarioDiscs s = scenario_discs("open01_swap_discs");
  core::ContinuousPlanResult r = sampling::Drrt(config()).plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  std::vector<double> radii;
  for (const auto& t : s.tasks) radii.push_back(t.radius);
  assert_joint_valid(r.paths, radii, s.grid);
}

TEST(Drrt, GoalOnObstacleFailsImmediatelyAsAnInstanceVerdict) {
  // A goal whose disc overlaps an obstacle cell admits no valid configuration —
  // a verdict on the instance itself at zero expansions (unlike budget
  // exhaustion, which is never a verdict).
  auto grid = unit_grid({"#."});
  std::vector<core::ContinuousAgentTask> tasks{
      core::ContinuousAgentTask{core::Point{1.5, 0.5}, core::Point{0.5, 0.5}, 0.4}};
  core::ContinuousPlanResult r = sampling::Drrt(config()).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
  EXPECT_EQ(r.stats.expanded_nodes, 0);
}

TEST(Drrt, OverlappingStartDiscsFailImmediately) {
  // Two start discs overlapping each other: no valid initial configuration
  // exists at all — the same kind of instance verdict, still zero expansions.
  auto grid = unit_grid({"#.#"});
  std::vector<core::ContinuousAgentTask> tasks{
      core::ContinuousAgentTask{core::Point{1.5, 0.5}, core::Point{3.5, 0.5}, 0.4},
      core::ContinuousAgentTask{core::Point{1.5, 0.5}, core::Point{1.5, 0.5}, 0.4}};
  core::ContinuousPlanResult r = sampling::Drrt(config()).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.stats.expanded_nodes, 0);
}

TEST(Drrt, UnreachableGoalIsABudgetFailureNotAVerdict) {
  // The wall separates start from goal for the single agent — but free_point
  // holds at both endpoints, so this is NOT an instance verdict: every roadmap
  // edge crossing the wall is segment-blocked, the tree grows on the start's
  // side and CONNECT_TO_TARGET never reaches vertex 1. Honest "no solution
  // found within budget" — with a grown tree (expanded_nodes > 0).
  auto grid = unit_grid({"#.#.#"});
  std::vector<core::ContinuousAgentTask> tasks{
      core::ContinuousAgentTask{core::Point{1.5, 0.5}, core::Point{3.5, 0.5}, 0.4}};
  core::ContinuousPlanResult r = sampling::Drrt(override_config(8, 2)).plan(grid, tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.paths.empty());
  EXPECT_GT(r.stats.expanded_nodes, 0);
}

TEST(Drrt, TraceEventsAreRoadmapsThenJointStates) {
  std::ostringstream os;
  ScenarioDiscs s = scenario_discs("open01_cross_discs");
  core::TraceRecorder rec(os);
  core::ContinuousPlanResult r = sampling::Drrt(config()).plan(s.grid, s.tasks, &rec);
  ASSERT_TRUE(r.success);

  std::istringstream in(os.str());
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(in, line)) lines.push_back(line);
  ASSERT_GE(lines.size(), 4u);
  // One roadmap_built per agent first (agent order), vertices in insertion
  // order (start then goal first), edges as i<j index pairs ordered by (min,max).
  EXPECT_NE(lines[0].find("\"event\":\"roadmap_built\""), std::string::npos);
  EXPECT_NE(lines[0].find("\"agent\":0"), std::string::npos);
  EXPECT_NE(lines[1].find("\"event\":\"roadmap_built\""), std::string::npos);
  EXPECT_NE(lines[1].find("\"agent\":1"), std::string::npos);
  int expansions = 0;
  bool saw_path_found = false;
  for (const auto& l : lines) {
    if (l.find("\"event\":\"node_expanded\"") != std::string::npos) {
      // Joint-state expansion: flattened world pairs — 2 agents x 2 numbers.
      size_t open = l.find("\"state\":[");
      ASSERT_NE(open, std::string::npos);
      size_t close = l.find(']', open);
      int commas = 0;
      for (size_t i = open + 9; i < close; ++i) {
        if (l[i] == ',') ++commas;
      }
      EXPECT_EQ(commas + 1, 4);
      ++expansions;
    }
    if (l.find("\"event\":\"path_found\"") != std::string::npos) saw_path_found = true;
  }
  EXPECT_GT(expansions, 0);
  EXPECT_TRUE(saw_path_found);
  const std::string& last = lines.back();
  EXPECT_NE(last.find("\"event\":\"planning_finished\""), std::string::npos);
  EXPECT_NE(last.find("\"success\":true"), std::string::npos);
}
