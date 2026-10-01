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

// World point (x, y), meters — map-layer coordinates only (scenario start/goal,
// continuous planner states). Exact equality is a real predicate here: every float
// decision upstream is bit-identical across languages, so equal points are exactly
// equal doubles (dRRT's path trimming relies on it).
struct Point {
  double x = 0.0;
  double y = 0.0;
  bool operator==(const Point& o) const { return x == o.x && y == o.y; }
};

// One agent's planning task on the shared grid (the demo driver converts the
// scenario's world-coord start/goal into Cells — coordinate frames stay owned by
// the map layer, per the repo rule). `vmax` is the kinodynamic branch's per-agent
// velocity limit in CELLS per time unit (default 1.0 = one cell per time unit,
// which reproduces the discrete step timing up to the safety distance). The
// search/sampling branches never read it; only KinodynamicPlanner implementations do.
struct AgentTask {
  Cell start;
  Cell goal;
  double vmax = 1.0;
};

// One agent's planning task on the shared continuous space: a disc robot of
// `radius` (meters) with world-coord start/goal. The scenario file carries the
// radius; discrete planners never see these tasks.
struct ContinuousAgentTask {
  Point start;
  Point goal;
  double radius = 0.0;
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

// Continuous-space counterpart of MultiPlanResult: paths[k][t] is agent k's world
// point at time step t (motion between consecutive waypoints is linear). Unlike
// the discrete result, the METRICS are whatever each planner's own paper defines
// as cost: dRRT counts steps like the discrete branch (cost = steps that move,
// makespan = arrival step of the last agent); dRRT* reports geometric arc lengths
// (sum and max over agents) because its own cost functions are reparameterization-
// invariant lengths. Both fields are explicit — no caller derives a metric from
// paths by convention.
struct ContinuousPlanResult {
  bool success = false;
  std::vector<std::vector<Point>> paths;
  double cost = 0.0;
  double makespan = 0.0;
  PlanStats stats;
};

// Kinodynamic-branch result: a plan-execution SCHEDULE, not a space-time path.
// routes[k] is agent k's route — the collision-free plan's cell sequence with the
// wait actions removed (consecutive cells are adjacent; every move edge has unit
// length). times[k][i] is the earliest arrival time at routes[k][i]; times[k][0]
// is 0 for every agent (every start event is pinned to t = 0 by the STN source).
// Execution under the uniform velocity model: an agent dwells on a cell until its
// departure (arrival of the next location minus l(e)/vmax) and traverses at exactly
// vmax — so arrival lands exactly on the scheduled time. cost is the sum over
// agents of their goal arrival times (the flow-time analogue), makespan t(X_F).
struct TimedPlanResult {
  bool success = false;
  std::vector<std::vector<Cell>> routes;
  std::vector<std::vector<double>> times;
  double cost = 0.0;
  double makespan = 0.0;
  PlanStats stats;
};

// A state serializes to trace as a numeric JSON array of two numbers (cell pair
// for grid algorithms, world-point pair when planning_started says coords=world);
// to_trace gives the pair — joint states flatten at the call site.
inline std::vector<double> to_trace(const Cell& c) {
  return {static_cast<double>(c.row), static_cast<double>(c.col)};
}

inline std::vector<double> to_trace(const Point& p) { return {p.x, p.y}; }

// Flattens a joint state's per-agent points into the trace wire form
// [x0, y0, x1, y1, ...] (what node_expanded carries for joint-space search).
inline std::vector<double> flatten_points(const std::vector<Point>& pts) {
  std::vector<double> out;
  out.reserve(pts.size() * 2);
  for (const Point& p : pts) {
    out.push_back(p.x);
    out.push_back(p.y);
  }
  return out;
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
