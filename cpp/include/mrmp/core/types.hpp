#pragma once

#include <cstddef>
#include <functional>
#include <vector>

namespace mrmp::core {

// Discrete state: grid cell (row, col), integers. row 0 = top image row. Every
// agent shares one grid; a joint state is just several Cells (flattened
// [r0,c0,r1,c1,...] on the trace wire), so there is no separate joint-state type.
struct Cell {
  int row = 0;
  int col = 0;
  bool operator==(const Cell& o) const { return row == o.row && col == o.col; }
  // Row-major total order so a Cell can key an ordered std::set (reservation and
  // constraint tables). Deterministic across runs/languages.
  bool operator<(const Cell& o) const { return row < o.row || (row == o.row && col < o.col); }
};

// World point (x, y), meters — map-layer coordinates only (scenario start/goal).
struct Point {
  double x = 0.0;
  double y = 0.0;
};

// One agent's planning task on the shared grid (the demo driver converts the
// scenario's world-coord start/goal into Cells — coordinate frames stay owned by
// the map layer, per the repo rule).
struct AgentTask {
  Cell start;
  Cell goal;
};

struct PlanStats {
  int expanded_nodes = 0;
};

struct MultiPlanResult {
  bool success = false;
  // paths[k] is agent k's space-time path: paths[k][t] is the cell occupied at
  // time step t. A finished agent's path simply ends; later steps do not exist.
  std::vector<std::vector<Cell>> paths;
  // sum of costs: every action (move or wait) costs one time step, so an agent's
  // cost is paths[k].size() - 1 and this is the sum over agents (standard MAPF
  // metric; makespan is derived from paths by callers).
  double cost = 0.0;
  PlanStats stats;
};

// A state serializes to trace as a numeric JSON array of ints ([row, col]);
// to_trace gives the pair for one Cell — joint states flatten at the call site.
inline std::vector<double> to_trace(const Cell& c) {
  return {static_cast<double>(c.row), static_cast<double>(c.col)};
}

// Flattens a joint state's per-agent cells into the trace wire form
// [r0, c0, r1, c1, ...] (what node_expanded carries for joint-space search).
inline std::vector<double> flatten(const std::vector<Cell>& cells) {
  std::vector<double> out;
  out.reserve(cells.size() * 2);
  for (const Cell& c : cells) {
    out.push_back(static_cast<double>(c.row));
    out.push_back(static_cast<double>(c.col));
  }
  return out;
}

}  // namespace mrmp::core

namespace std {
template <>
struct hash<mrmp::core::Cell> {
  size_t operator()(const mrmp::core::Cell& c) const noexcept {
    size_t h1 = std::hash<int>()(c.row);
    size_t h2 = std::hash<int>()(c.col);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 6));
  }
};
}  // namespace std
