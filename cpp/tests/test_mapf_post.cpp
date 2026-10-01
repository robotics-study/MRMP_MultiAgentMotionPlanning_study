// MAPF-POST: what a plan is worth once it has a clock — the byte-exact mirror of
// python/tests/test_mapf_post.py. Same pinned numbers (dyadic rationals everywhere, so both
// languages round nothing), same safety property checked on the schedules themselves, same
// honest failure. Mirrors the Python case for case.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mrmp/core/params.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/kinodynamic/mapf_post.hpp"
#include "mrmp/maps/loader.hpp"
#include "mrmp/maps/occupancy_grid.hpp"
#include "test_util.hpp"

using namespace mrmp;

namespace {

constexpr double kDelta = 0.25;

core::ParamSet config() {
  return core::ParamSet::from_yaml(test::repo_path("configs/kinodynamic/mapf_post.yaml"));
}

// The declared config with `delta` replaced (the inherited CBS budget stays).
core::ParamSet config_with_delta(double delta) {
  return core::ParamSet::from_yaml(test::write_temp(
      "mapf_post.yaml",
      "algorithm: mapf_post\nsection: kinodynamic\nscenarios: []\nparams:\n"
      "  - name: delta\n    type: float\n    default: " +
          std::to_string(delta) +
          "\n    min: 0.0\n    max: 0.5\n    description: test delta\n"
          "  - name: max_ct_expansions\n    type: int\n    default: 256\n    min: 1\n"
          "    description: test budget\n"));
}

struct ScenarioSetup {
  maps::OccupancyGrid2D grid;
  std::vector<core::AgentTask> tasks;
};

// Load a repo scenario and convert its world-coord endpoints to cells, carrying each agent's
// velocity limit along (this branch is the only one that reads vmax).
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

// The scheduled arrival time at one retained location.
double at(const core::TimedPlanResult& r, size_t agent, const core::Cell& cell) {
  for (size_t i = 0; i < r.routes[agent].size(); ++i) {
    if (r.routes[agent][i] == cell) return r.times[agent][i];
  }
  throw std::runtime_error("cell is not on that route");
}

// When that agent leaves the cell (the next arrival minus its own traversal l/v). The Type-2
// precedence reads this instant plus delta/v, never the arrival itself.
double departure(const core::TimedPlanResult& r, size_t agent, const core::Cell& cell, double v) {
  for (size_t i = 0; i + 1 < r.routes[agent].size(); ++i) {
    if (r.routes[agent][i] == cell) return r.times[agent][i + 1] - 1.0 / v;
  }
  throw std::runtime_error("cell is not a departed location on that route");
}

// Where an agent is at time tau under the uniform velocity model: dwell until its departure,
// then traverse at exactly v. Cell indices are coordinates (grid distance = position
// distance here), and a position is piecewise-linear with breakpoints only at arrivals and
// departures — so evaluating every breakpoint is exact, not sampled.
core::Point position(const core::TimedPlanResult& r, size_t agent, double v, double tau) {
  const auto& route = r.routes[agent];
  const auto& times = r.times[agent];
  if (tau <= 0.0)
    return core::Point{static_cast<double>(route.front().row), static_cast<double>(route.front().col)};
  if (tau >= times.back())
    return core::Point{static_cast<double>(route.back().row), static_cast<double>(route.back().col)};
  for (size_t i = 0; i + 1 < route.size(); ++i) {
    const double arrive = times[i + 1];
    const double depart = arrive - 1.0 / v;
    if (tau < depart)
      return core::Point{static_cast<double>(route[i].row), static_cast<double>(route[i].col)};
    if (tau < arrive) {
      const double frac = (tau - depart) * v;
      const double dr = static_cast<double>(route[i + 1].row - route[i].row);
      const double dc = static_cast<double>(route[i + 1].col - route[i].col);
      return core::Point{static_cast<double>(route[i].row) + dr * frac,
                         static_cast<double>(route[i].col) + dc * frac};
    }
  }
  throw std::runtime_error("unreachable: tau lies inside the schedule");
}

// The open time interval during which that agent lies strictly INSIDE the radius-delta cloud
// of one retained location (a final cell never leaves, so it stays unbounded).
std::pair<double, double> cloud(const core::TimedPlanResult& r, size_t agent, double v,
                               size_t index) {
  const double enter = r.times[agent][index] - kDelta / v;
  if (index + 1 >= r.times[agent].size()) return {enter, std::numeric_limits<double>::infinity()};
  const double depart = r.times[agent][index + 1] - 1.0 / v;
  return {enter, depart + kDelta / v};
}

// Tightest gap between the two visitors' cloud intervals over every shared cell. Negative
// would mean overlap; exactly 0 means the precedence binds (open intervals may touch).
double cloud_gap(const core::TimedPlanResult& r, const std::vector<double>& vmax) {
  double worst = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < r.routes.size(); ++i) {
    for (size_t j = i + 1; j < r.routes.size(); ++j) {
      for (size_t k = 0; k < r.routes[j].size(); ++k) {
        const auto it = std::find(r.routes[i].begin(), r.routes[i].end(), r.routes[j][k]);
        if (it == r.routes[i].end()) continue;
        const auto a = cloud(r, i, vmax[i], static_cast<size_t>(it - r.routes[i].begin()));
        const auto b = cloud(r, j, vmax[j], k);
        const bool flip = b.first < a.first;
        const std::pair<double, double>& first = flip ? b : a;
        const std::pair<double, double>& second = flip ? a : b;
        worst = std::min(worst, second.first - first.second);
      }
    }
  }
  return worst;
}

std::pair<double, double> min_graph_distance(const core::TimedPlanResult& r,
                                             const std::vector<double>& vmax) {
  std::vector<double> points{0.0};
  double horizon = 0.0;
  for (size_t k = 0; k < r.times.size(); ++k) {
    for (size_t i = 0; i < r.times[k].size(); ++i) {
      points.push_back(r.times[k][i]);
      if (i > 0) points.push_back(r.times[k][i] - 1.0 / vmax[k]);
    }
    horizon = std::max(horizon, r.times[k].back());
  }
  std::sort(points.begin(), points.end());
  double best = std::numeric_limits<double>::infinity();
  double at_time = 0.0;
  for (double tau : points) {
    if (tau > horizon) continue;
    for (size_t i = 0; i < r.routes.size(); ++i) {
      for (size_t j = i + 1; j < r.routes.size(); ++j) {
        const core::Point a = position(r, i, vmax[i], tau);
        const core::Point b = position(r, j, vmax[j], tau);
        const double d = std::abs(a.x - b.x) + std::abs(a.y - b.y);
        if (d < best) {
          best = d;
          at_time = tau;
        }
      }
    }
  }
  return {best, at_time};
}

}  // namespace

TEST(MapfPost, ContractMatchesConfig) {
  kinodynamic::MapfPost planner(config());
  EXPECT_EQ(planner.name(), config().algorithm());
  // The input stays discrete: cells plus a per-agent velocity limit.
  EXPECT_EQ(planner.required_capabilities().count(core::Capability::DISCRETE_SPACE), 1u);
  EXPECT_DOUBLE_EQ(config().get_float("delta"), 0.25);
  EXPECT_EQ(config().get_int("max_ct_expansions"), 256);
}

TEST(MapfPost, DeltaZeroIsRefusedRatherThanSilentlyUnsafe) {
  // The declared range includes 0 (a config may declare it), but a collapsed marker voids
  // the safety bound, so the planner itself refuses it instead of scheduling unsafely.
  kinodynamic::MapfPost planner(config_with_delta(0.0));
  ScenarioSetup s = scenario("pocket01_swap_timed");
  EXPECT_THROW(planner.plan(s.grid, s.tasks, nullptr), std::runtime_error);
}

TEST(MapfPost, SingleAgentScalesTimeByItsOwnVelocity) {
  // One agent, no Type-2 edge possible: the STN is a single chain, so every arrival is
  // exactly the previous one plus l/v. Velocity alone sets the clock — 4 cells at v = 2
  // arrive 0.5 apart, and the cost is TIME (2.0), not steps (4).
  kinodynamic::MapfPost planner(config());
  auto grid = test::make_grid({".....", ".....", "....."});
  std::vector<core::AgentTask> tasks{core::AgentTask{core::Cell{1, 0}, core::Cell{1, 4}, 2.0}};
  core::TimedPlanResult r = planner.plan(grid, tasks, nullptr);
  ASSERT_TRUE(r.success);
  ASSERT_EQ(r.routes.size(), 1u);
  EXPECT_EQ(r.routes[0], (std::vector<core::Cell>{{1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}}));
  EXPECT_EQ(r.times[0], (std::vector<double>{0.0, 0.5, 1.0, 1.5, 2.0}));
  EXPECT_DOUBLE_EQ(r.cost, 2.0);
  EXPECT_DOUBLE_EQ(r.makespan, 2.0);
  // Four segments x three edges each, every one of them strictly improving exactly once: the
  // relaxation count is a property of the chain's shape, not of the map.
  EXPECT_EQ(r.stats.expanded_nodes, 12);
}

TEST(MapfPost, UnitVelocityReproducesTheDiscretePlanExactly) {
  // Both agents at one cell per time unit and every shared location visited a whole step
  // apart: each Type-2 constraint asks for LESS than the chain already provides, so the
  // schedule IS the discrete plan's own steps — and the metrics equal CBS's cost and
  // makespan, this branch's baseline against which everything else is compared.
  kinodynamic::MapfPost planner(config());
  ScenarioSetup s = scenario("open01_swap_timed");
  core::TimedPlanResult r = planner.plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  std::vector<double> a, b;
  for (int t = 0; t <= 16; ++t) a.push_back(static_cast<double>(t));
  for (int t = 0; t <= 14; ++t) b.push_back(static_cast<double>(t));
  EXPECT_EQ(r.times[0], a);  // 16 moves at v = 1
  EXPECT_EQ(r.times[1], b);
  EXPECT_DOUBLE_EQ(r.cost, 30.0);  // CBS's sum_of_costs on this scenario
  EXPECT_DOUBLE_EQ(r.makespan, 16.0);
}

TEST(MapfPost, WaitsAreDroppedAndStepsOnlyOrderTheEdges) {
  // The discrete plan behind tee01_head_on spends a whole step waiting at (1,2); the route
  // keeps one entry per LOCATION and its arrival times stop counting steps. And dropping the
  // wait is exactly what continuous execution buys back: agent 1 leaves (1,3) at t = 2 and
  // its marker clears a delta later, so agent 0 arrives at 2.5 instead of step 3 — the pair
  // sums to 10.5 where the discrete plan cost 11.
  kinodynamic::MapfPost planner(config());
  ScenarioSetup s = scenario("tee01_head_on_timed");
  core::TimedPlanResult r = planner.plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  EXPECT_EQ(r.routes[0], (std::vector<core::Cell>{{1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}}));
  EXPECT_EQ(r.times[0], (std::vector<double>{0.0, 1.0, 2.5, 3.5, 4.5}));
  EXPECT_DOUBLE_EQ(r.cost, 10.5);
  EXPECT_DOUBLE_EQ(r.makespan, 6.0);
}

TEST(MapfPost, AWiderMarkerGivesTheDiscreteTimingBack) {
  // The same instance with delta = 0.5 (the largest legal value): the two markers around a
  // shared cell now need 2 * 0.5 = 1 time unit of separation at v = 1 — exactly the whole
  // step the discrete plan had already spent waiting. The schedule relaxes back to those
  // steps, which is what "this constraint no longer binds" means physically.
  kinodynamic::MapfPost planner(config_with_delta(0.5));
  ScenarioSetup s = scenario("tee01_head_on_timed");
  core::TimedPlanResult r = planner.plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  EXPECT_EQ(r.times[0], (std::vector<double>{0.0, 1.0, 3.0, 4.0, 5.0}));
  EXPECT_DOUBLE_EQ(r.cost, 11.0);
}

TEST(MapfPost, TheFastAgentIsHeldUntilTheSlowOneClears) {
  // The crossing pair at unequal velocities: agent 1 (v = 2) would reach the shared cell
  // (10,9) at t = 4.5 on its own chain alone, but agent 0 (v = 1) only clears ITS marker at
  // 8 + 0.25/1 = 8.25, so agent 1 arrives at 8.25 + 0.25/2 = 8.375 and every later arrival
  // inherits that offset. This schedule is not a rescaled copy of the discrete plan — which
  // is the whole point of putting time on it.
  kinodynamic::MapfPost planner(config());
  ScenarioSetup s = scenario("open01_cross_timed");
  ASSERT_DOUBLE_EQ(s.tasks[0].vmax, 1.0);
  ASSERT_DOUBLE_EQ(s.tasks[1].vmax, 2.0);
  core::TimedPlanResult r = planner.plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  // Agent 0 is never held: its own chain (one cell per unit time) stays tight.
  std::vector<double> tight;
  for (int t = 0; t <= 16; ++t) tight.push_back(static_cast<double>(t));
  EXPECT_EQ(r.times[0], tight);
  EXPECT_DOUBLE_EQ(at(r, 1, core::Cell{9, 9}), 4.0);
  // Agent 0 leaves the shared cell at T(next) - l/v = 9 - 1 = 8 and its marker clears at
  // 8 + delta/1; agent 1 needs another delta/v of its own on top of that.
  EXPECT_DOUBLE_EQ(departure(r, 0, core::Cell{10, 9}, s.tasks[0].vmax), 8.0);
  EXPECT_DOUBLE_EQ(at(r, 1, core::Cell{10, 9}), 8.375);
  // Sum 28.375 against the discrete plan's 33: the fast agent finishes in 12.375 of its own
  // time units while the slow one still needs its full 16 — makespan is the slower clock.
  EXPECT_DOUBLE_EQ(r.times[1].back(), 12.375);
  EXPECT_DOUBLE_EQ(r.cost, 28.375);
  EXPECT_DOUBLE_EQ(r.makespan, 16.0);
}

TEST(MapfPost, ASlowAgentStretchesEverythingBehindIt) {
  // The same maze swap at v = 1 and v = 1/4. The slow agent needs four time units per cell,
  // so its marker leaves a shared cell a whole unit after it does — the fast one's arrival
  // there is pushed from step 23 to 89.25 (the slow agent departs at 88 and its marker
  // clears at 88 + 0.25/0.25 = 89). The fast agent still finishes in 99.25 of ITS units; the
  // makespan is the slow agent's 132 = 4 x 33.
  kinodynamic::MapfPost planner(config());
  ScenarioSetup s = scenario("maze01_two_timed");
  ASSERT_DOUBLE_EQ(s.tasks[0].vmax, 1.0);
  ASSERT_DOUBLE_EQ(s.tasks[1].vmax, 0.25);
  core::TimedPlanResult r = planner.plan(s.grid, s.tasks, nullptr);
  ASSERT_TRUE(r.success);
  ASSERT_EQ(r.times[1].size(), 34u);
  EXPECT_DOUBLE_EQ(r.times[1][0], 0.0);
  EXPECT_DOUBLE_EQ(r.times[1][5], 20.0);  // one cell every FOUR time units
  EXPECT_DOUBLE_EQ(at(r, 1, core::Cell{2, 9}), 88.0);
  EXPECT_DOUBLE_EQ(at(r, 0, core::Cell{2, 9}), 89.25);
  EXPECT_DOUBLE_EQ(r.times[0].back(), 99.25);
  EXPECT_DOUBLE_EQ(r.cost, 231.25);
  EXPECT_DOUBLE_EQ(r.makespan, 132.0);
}

TEST(MapfPost, ScalingBothVelocitiesScalesTimeAndNothingElse) {
  // Same instance with both velocity limits doubled: every chain bound halves and the fixed
  // point with it (Type-2 edges carry LB 0, so nothing absolute survives), which is a check
  // that no absolute clock leaked into the construction.
  kinodynamic::MapfPost planner(config());
  ScenarioSetup s = scenario("pocket01_swap_timed");
  std::vector<core::AgentTask> doubled;
  for (const auto& t : s.tasks) doubled.push_back(core::AgentTask{t.start, t.goal, t.vmax * 2.0});
  core::TimedPlanResult slow = planner.plan(s.grid, s.tasks, nullptr);
  core::TimedPlanResult fast = planner.plan(s.grid, doubled, nullptr);
  ASSERT_TRUE(slow.success && fast.success);
  EXPECT_EQ(fast.routes, slow.routes);
  for (size_t k = 0; k < slow.times.size(); ++k) {
    ASSERT_EQ(fast.times[k].size(), slow.times[k].size());
    for (size_t i = 0; i < slow.times[k].size(); ++i) {
      EXPECT_DOUBLE_EQ(fast.times[k][i], slow.times[k][i] / 2.0);
    }
  }
  EXPECT_DOUBLE_EQ(fast.cost, slow.cost / 2.0);
}

TEST(MapfPost, EverySharedCellStaysProtected) {
  // Theorem 2, checked on the schedules instead of trusted: for every cell two routes share,
  // the two visitors' protected clouds are disjoint in time (a negative gap would mean
  // overlap), and equivalently the grid-graph distance never falls under 2*delta*v_min/v_max.
  // Both are evaluated at every breakpoint of the piecewise-linear motion, so they are exact
  // minima rather than samples.
  kinodynamic::MapfPost planner(config());
  for (const std::string name : {"open01_cross_timed", "open01_swap_timed", "pocket01_swap_timed",
                                 "tee01_head_on_timed", "maze01_two_timed"}) {
    ScenarioSetup s = scenario(name);
    core::TimedPlanResult r = planner.plan(s.grid, s.tasks, nullptr);
    ASSERT_TRUE(r.success) << name;
    const std::vector<double> vmax = vmax_of(s.tasks);
    EXPECT_GE(cloud_gap(r, vmax), 0.0) << name;
    const double bound =
        2 * kDelta * std::min(vmax[0], vmax[1]) / std::max(vmax[0], vmax[1]);
    const auto [distance, when] = min_graph_distance(r, vmax);
    EXPECT_GE(distance, bound - 1e-9) << name << " at t=" << when;
  }
}

TEST(MapfPost, TheBoundIsTightExactlyWhereAType2EdgeBinds) {
  // On the unit-velocity tee junction the minimum distance is EXACTLY 2*delta (0.5): the
  // later visitor reaches its marker at the instant the earlier one passes its own, which is
  // what "this precedence binds" means physically. The crossing pair's bound is looser
  // (v_min/v_max = 1/2) and not even reached — safety there comes with margin, and the test
  // says so instead of pretending every constraint binds.
  kinodynamic::MapfPost planner(config());
  ScenarioSetup tee = scenario("tee01_head_on_timed");
  core::TimedPlanResult t = planner.plan(tee.grid, tee.tasks, nullptr);
  ASSERT_TRUE(t.success);
  const auto tight = min_graph_distance(t, vmax_of(tee.tasks));
  EXPECT_DOUBLE_EQ(tight.first, 0.5);
  EXPECT_DOUBLE_EQ(tight.second, 2.0);

  ScenarioSetup cross = scenario("open01_cross_timed");
  core::TimedPlanResult c = planner.plan(cross.grid, cross.tasks, nullptr);
  ASSERT_TRUE(c.success);
  EXPECT_GT(min_graph_distance(c, vmax_of(cross.tasks)).first, 2 * kDelta * (1.0 / 2.0));
}

TEST(MapfPost, NoPlanToPostProcessIsReportedAsAFailure) {
  // The corridor swap has no discrete plan at all and this branch invents none: the inherited
  // budget stop comes through as a failure with every metric zeroed, so a reader can never
  // mistake "nothing to schedule" for an empty schedule.
  kinodynamic::MapfPost planner(config());
  ScenarioSetup s = scenario("corridor01_head_on_timed");
  core::TimedPlanResult r = planner.plan(s.grid, s.tasks, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.routes.empty());
  EXPECT_TRUE(r.times.empty());
  EXPECT_DOUBLE_EQ(r.cost, 0.0);
  EXPECT_DOUBLE_EQ(r.makespan, 0.0);
  EXPECT_EQ(r.stats.expanded_nodes, 0);
}

TEST(MapfPost, TraceCarriesSchedulesAndNoSearchEvents) {
  // The underlying CBS runs SILENTLY: a timed trace carries one schedule_found per agent in
  // index order and planning_finished — no node_expanded and no conflict_found, because this
  // branch's own work metric counts STN relaxations, not the base search's expansions.
  std::ostringstream os;
  ScenarioSetup s = scenario("pocket01_swap_timed");
  core::TraceRecorder rec(os);
  core::TimedPlanResult r = kinodynamic::MapfPost(config()).plan(s.grid, s.tasks, &rec);
  ASSERT_TRUE(r.success);

  std::istringstream in(os.str());
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(in, line)) lines.push_back(line);
  ASSERT_EQ(lines.size(), 3u);  // two schedule_found + planning_finished
  EXPECT_NE(lines[0].find("\"event\":\"schedule_found\""), std::string::npos);
  EXPECT_NE(lines[0].find("\"agent\":0"), std::string::npos);
  EXPECT_NE(lines[1].find("\"agent\":1"), std::string::npos);
  EXPECT_EQ(os.str().find("node_expanded"), std::string::npos);
  EXPECT_EQ(os.str().find("conflict_found"), std::string::npos);
  const std::string& last = lines.back();
  EXPECT_NE(last.find("\"event\":\"planning_finished\""), std::string::npos);
  EXPECT_NE(last.find("\"success\":true"), std::string::npos);
  EXPECT_NE(last.find("\"sum_of_costs\":10"), std::string::npos);
}
