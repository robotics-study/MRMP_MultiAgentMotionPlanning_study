#pragma once

#include <array>
#include <set>
#include <utility>
#include <vector>

#include "mrmp/core/types.hpp"

namespace mrmp::core {

// Capability model: algorithms require capabilities, map types provide them. MRMP
// has exactly two capabilities: every search-branch planner searches the same
// discrete grid; every sampling-branch planner whose robots are geometric discs
// plans on that same map's continuous free space instead.
enum class Capability {
  DISCRETE_SPACE,
  CONTINUOUS_SPACE,
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
// motion graph's vertex set, and a uniform draw needs that enumeration to be
// part of the contract, not an implementation detail.
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

// Continuous free-space view of a map for disc robots. A configuration q is free
// iff the disc of radius r around q overlaps no obstacle cell (touching counts as
// free — collision means strict overlap). A segment is free iff its swept disc
// stays clear. Every predicate is an exact float expression evaluated in an
// identical operation order in Python and C++, so all engines decide every
// boundary case on identical bits. extent() returns the world rectangle
// (x_min, y_min, x_max, y_max) planners sample configurations uniformly from.
class ContinuousSpace {
 public:
  virtual ~ContinuousSpace() = default;
  // World footprint (x_min, y_min, x_max, y_max) of the map — the sampling rect.
  virtual std::array<double, 4> extent() const = 0;
  // True iff the disc of `radius` around q overlaps no obstacle cell.
  virtual bool free_point(const Point& q, double radius) const = 0;
  // True iff every point of segment a->b is free for a disc of `radius`.
  virtual bool segment_free(const Point& a, const Point& b, double radius) const = 0;
};

// Base for concrete maps. Owns the single `supports` implementation.
class MapBase {
 public:
  virtual ~MapBase() = default;
  virtual std::set<Capability> capabilities() const = 0;
  bool supports(Capability c) const { return capabilities().count(c) > 0; }
};

}  // namespace mrmp::core
