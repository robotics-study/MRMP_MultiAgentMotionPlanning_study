#pragma once

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

// 2D occupancy grid providing the DiscreteSpace capability. World<->cell
// conversion lives only here. Geometry follows ROS map_server: origin is the
// world pose of the bottom-left pixel and row 0 is the top image row. The move
// set is the MAPF action model: 4-connected moves plus a wait self-loop, every
// action costing one time step (see core/capabilities.hpp).
class OccupancyGrid2D final : public core::MapBase, public core::DiscreteSpace {
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

  int rows() const { return rows_; }
  int cols() const { return cols_; }
  double resolution() const { return resolution_; }
  bool is_free(int row, int col) const;
  // Row-major free mask (row 0 = top), width-wide — the viz-side mirror of
  // Python's free_mask().
  const std::vector<bool>& free_cells() const { return free_; }

 private:
  bool in_bounds(int row, int col) const;

  int rows_;
  int cols_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<bool> free_;  // rows_*cols_, row-major, row 0 = top
};

}  // namespace mrmp::maps
