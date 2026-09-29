#pragma once

#include <array>
#include <set>
#include <utility>
#include <vector>

#include "mrmp/core/capabilities.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/maps/pgm.hpp"

namespace mrmp::maps {

using core::Capability;
using core::Cell;
using core::Point;

// 2D occupancy grid providing BOTH capabilities. World<->cell conversion lives
// only here. Geometry follows ROS map_server: origin is the world pose of the
// bottom-left pixel and row 0 is the top image row. The move set is the MAPF
// action model: 4-connected moves plus a wait self-loop, every action costing
// one time step (see core/capabilities.hpp).
//
// The same raster also answers the ContinuousSpace queries for disc robots: a
// configuration q is free iff the disc of radius r around q overlaps no obstacle
// cell, and a segment is free iff its swept disc stays clear. Every predicate is
// an exact float expression (core/geometry primitives, fixed operation order) so
// all engines decide every boundary case on identical bits; collision means STRICT
// overlap — touching counts as free.
class OccupancyGrid2D final : public core::MapBase,
                              public core::DiscreteSpace,
                              public core::ContinuousSpace {
 public:
  OccupancyGrid2D(int rows, int cols, double resolution, double origin_x, double origin_y,
                  std::vector<bool> free_cells);

  // Applies occupancy thresholds: occ = 1 - pixel/255; a cell is traversable
  // only when occ <= free_thresh (occupied or unknown cells are blocked).
  static OccupancyGrid2D from_image(const PgmImage& img, double resolution, double origin_x,
                                    double origin_y, double occupied_thresh, double free_thresh);

  std::set<Capability> capabilities() const override;

  Point cell_to_world(const Cell& c) const;
  Cell world_to_cell(double x, double y) const;

  // Passable 4-connected moves (fixed order up/down/left/right) then the wait
  // self-loop; every action costs one time step. Fixed order = deterministic
  // tie-breaking across languages.
  std::vector<std::pair<Cell, double>> neighbors(const Cell& s) const override;
  // Manhattan distance — admissible + consistent for the unit-cost move set.
  double heuristic(const Cell& a, const Cell& b) const override;
  // Every passable cell in canonical row-major order (row ascending, then column)
  // — the motion graph's vertex set for uniform waypoint sampling.
  std::vector<Cell> cells() const override;

  // World footprint (x_min, y_min, x_max, y_max) — the rectangle continuous
  // planners sample configurations uniformly from.
  std::array<double, 4> extent() const override;
  // mu(C_f): free cell count x resolution^2, evaluated left-to-right
  // ((count * res) * res) so every language lands on identical bits — the
  // asymptotic-optimality radius bound reads this.
  double area() const override;
  // Free iff the disc of `radius` around q overlaps no obstacle cell in more than
  // a boundary point — blocked iff dist(q, cell) < radius (and for a point robot,
  // radius 0, blocked exactly when q lies on/inside a cell: distance 0). Exact
  // per-cell point-to-rect distance; cells provably closer-cleared are skipped on
  // the exact lower bound dx > radius.
  bool free_point(const Point& q, double radius) const override;
  // Free iff every point of segment a->b is free for a disc of `radius` — blocked
  // iff the segment's distance to some obstacle cell is strictly below the radius,
  // or zero (touching/entering the closed cell). Cells whose axis-aligned
  // separation already exceeds the radius are skipped on that exact lower bound.
  bool segment_free(const Point& a, const Point& b, double radius) const override;

  int rows() const { return rows_; }
  int cols() const { return cols_; }
  double resolution() const { return resolution_; }
  bool is_free(int row, int col) const;
  // Row-major free mask (row 0 = top), width-wide — the viz-side mirror of
  // Python's free_mask().
  const std::vector<bool>& free_cells() const { return free_; }

 private:
  bool in_bounds(int row, int col) const;

  // World rectangle (x_lo, y_lo, x_hi, y_hi) of one cell — same float expressions
  // as the Python map layer.
  std::array<double, 4> cell_rect(int row, int col) const;
  // Distance from a point to a closed cell rect (0 inside), and between a segment
  // and a closed cell rect (0 on any intersection): min over {endpoint-to-rect}
  // and corner-to-segment distances — for two disjoint segments the closest pair
  // always includes an endpoint of one.
  double point_rect_distance(const Point& p, const std::array<double, 4>& rect) const;
  double segment_rect_distance(const Point& a, const Point& b,
                               const std::array<double, 4>& rect) const;

  int rows_;
  int cols_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<bool> free_;  // rows_*cols_, row-major, row 0 = top
};

}  // namespace mrmp::maps
