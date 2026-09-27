#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mrmp/core/params.hpp"
#include "mrmp/core/planner.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/maps/loader.hpp"
#include "mrmp/maps/occupancy_grid.hpp"

// Assembly-only scaffold shared by the demo executables: parse CLI args, wire a
// planner to a loaded map + scenario, emit a trace, and print a one-line metrics
// summary. No planning logic lives here.
namespace demo {

struct Args {
  std::string map;
  std::string scenario;
  std::string params;
  std::string trace;
};

inline Args parse_args(int argc, char** argv) {
  Args a;
  auto need = [&](int& i) -> std::string {
    if (i + 1 >= argc) throw std::runtime_error("demo: missing value for " + std::string(argv[i]));
    return argv[++i];
  };
  for (int i = 1; i < argc; ++i) {
    std::string f = argv[i];
    if (f == "--map") {
      a.map = need(i);
    } else if (f == "--scenario") {
      a.scenario = need(i);
    } else if (f == "--params") {
      a.params = need(i);
    } else if (f == "--trace") {
      a.trace = need(i);
    } else {
      throw std::runtime_error("demo: unknown flag " + f);
    }
  }
  if (a.map.empty() || a.scenario.empty() || a.params.empty() || a.trace.empty()) {
    throw std::runtime_error("demo: --map --scenario --params --trace are required");
  }
  return a;
}

inline mrmp::maps::OccupancyGrid2D& as_grid(mrmp::core::MapBase& map) {
  auto* grid = dynamic_cast<mrmp::maps::OccupancyGrid2D*>(&map);
  if (!grid) throw std::runtime_error("demo: map is not an occupancy grid");
  return *grid;
}

// One demo run: load map + scenario, convert world-coord agent tasks to cells
// (coordinate frames stay owned by the map layer, per the repo rule), plan with
// a live recorder, and print the one-line JSON summary (sum_of_costs / makespan
// / expanded_nodes — same fields as the Python driver).
template <class Planner>
inline int run(const Args& a, const mrmp::core::ParamSet& params, Planner& planner) {
  auto map = mrmp::maps::load_map(a.map);
  auto& grid = as_grid(*map);
  mrmp::maps::Scenario sc = mrmp::maps::load_scenario(a.scenario);

  std::vector<mrmp::core::AgentTask> tasks;
  for (const auto& spec : sc.agents) {
    tasks.push_back(mrmp::core::AgentTask{grid.world_to_cell(spec.start.x, spec.start.y),
                                          grid.world_to_cell(spec.goal.x, spec.goal.y)});
  }

  std::ofstream fs(a.trace);
  if (!fs) throw std::runtime_error("demo: cannot open trace file " + a.trace);
  mrmp::core::TraceRecorder rec(fs);
  rec.planning_started(planner.name(), a.map, params.values());
  auto res = planner.plan(grid, tasks, &rec);

  // makespan = arrival step of the last agent; an empty path list reports 0.
  int makespan = 0;
  for (const auto& p : res.paths) makespan = std::max(makespan, static_cast<int>(p.size()) - 1);
  if (res.paths.empty()) makespan = 0;

  std::cout << "{\"algorithm\":\"" << planner.name() << "\",\"success\":"
            << (res.success ? "true" : "false") << ",\"sum_of_costs\":" << res.cost
            << ",\"makespan\":" << makespan << ",\"expanded_nodes\":" << res.stats.expanded_nodes
            << "}\n";
  return 0;
}

}  // namespace demo
