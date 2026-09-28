#pragma once

#include <set>
#include <utility>
#include <vector>

#include "mrmp/core/types.hpp"

namespace mrmp::core {

// Capability model: algorithms require capabilities, map types provide them. MRMP
// has exactly one capability: every planner searches the same discrete grid.
enum class Capability {
  DISCRETE_SPACE,
};

// Graph-search view: enumerable successors + admissible heuristic.
//
// The move set IS the MAPF action model (Silver 2005): 4-connected moves plus a
// wait action — a self-loop on the current cell — and every action costs exactly
// one time step, so g-values count elapsed steps and sum-of-costs / makespan are
// directly comparable across agents. Successor order is fixed (up, down, left,
// right, then wait) because tie-breaking must be identical across languages.
//
// cells() exposes every passable cell in the same canonical row-major order on
// every platform: sampling planners (MA-RRT*) draw waypoints uniformly from the
// motion graph's vertex set, and a uniform draw needs that enumeration to be part
// of the contract, not an implementation detail.
class DiscreteSpace {
 public:
  virtual ~DiscreteSpace() = default;
  // Passable 4-connected moves (fixed order up/down/left/right) then the wait
  // self-loop; every action costs one time step.
  virtual std::vector<std::pair<Cell, double>> neighbors(const Cell& s) const = 0;
  // Manhattan distance: admissible + consistent for the unit-cost 4-connected
  // move set (diagonals are not moves here).
  virtual double heuristic(const Cell& a, const Cell& b) const = 0;
  // Every passable cell in canonical row-major order (row ascending, then column)
  // — the motion graph's vertex set for uniform waypoint sampling.
  virtual std::vector<Cell> cells() const = 0;
};

// Base for concrete maps. Owns the single `supports` implementation.
class MapBase {
 public:
  virtual ~MapBase() = default;
  virtual std::set<Capability> capabilities() const = 0;
  bool supports(Capability c) const { return capabilities().count(c) > 0; }
};

}  // namespace mrmp::core
