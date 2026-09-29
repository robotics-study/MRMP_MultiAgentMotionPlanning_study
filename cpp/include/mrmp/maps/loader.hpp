#pragma once

#include <memory>
#include <string>
#include <vector>

#include "mrmp/core/capabilities.hpp"
#include "mrmp/core/types.hpp"

namespace mrmp::maps {

// One agent's world-coord start/goal and disc radius — the discrete demo driver
// converts to Cells via world_to_cell (coordinate frames stay owned by the map
// layer); continuous planners use the raw Points and the radius (0.0 = point
// robot when the scenario omits it).
struct AgentSpec {
  core::Point start;
  core::Point goal;
  double radius = 0.0;
};

// Multi-agent problem definition resolved from a scenario yaml: one entry per
// agent, in agent-index order. World coords like the single-robot nav_study
// scenarios — only the `agents:` schema is accepted here.
struct Scenario {
  std::string map_path;  // absolute, resolved relative to the scenario file
  std::vector<AgentSpec> agents;
};

// Dispatches on the yaml `type` field. Only occupancy_grid is implemented; other
// types throw a clear error.
std::unique_ptr<core::MapBase> load_map(const std::string& path);

Scenario load_scenario(const std::string& path);

}  // namespace mrmp::maps
