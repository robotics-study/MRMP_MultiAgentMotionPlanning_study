#include "mrmp/maps/occupancy_grid.hpp"

#include <cmath>

namespace mrmp::maps {

OccupancyGrid2D::OccupancyGrid2D(int rows, int cols, double resolution, double origin_x,
                                 double origin_y, std::vector<bool> free_cells)
    : rows_(rows),
      cols_(cols),
      resolution_(resolution),
      origin_x_(origin_x),
      origin_y_(origin_y),
      free_(std::move(free_cells)) {}

OccupancyGrid2D OccupancyGrid2D::from_image(const PgmImage& img, double resolution,
                                            double origin_x, double origin_y,
                                            double occupied_thresh, double free_thresh) {
  // occupied_thresh is accepted for ROS yaml signature compatibility but not used:
  // traversability is decided by free_thresh alone (occupied/unknown are blocked),
  // so a separate occupied cutoff would be redundant. occ = 1 - p/255; free iff
  // occ <= free_thresh; anything not clearly free is non-traversable.
  (void)occupied_thresh;
  std::vector<bool> free_cells(static_cast<size_t>(img.width) * img.height);
  for (size_t i = 0; i < free_cells.size(); ++i) {
    double occ = 1.0 - static_cast<double>(img.pixels[i]) / 255.0;
    free_cells[i] = occ <= free_thresh;
  }
  return OccupancyGrid2D(img.height, img.width, resolution, origin_x, origin_y,
                         std::move(free_cells));
}

bool OccupancyGrid2D::in_bounds(int row, int col) const {
  return 0 <= row && row < rows_ && 0 <= col && col < cols_;
}

bool OccupancyGrid2D::is_free(int row, int col) const {
  return in_bounds(row, col) && free_[static_cast<size_t>(row) * cols_ + static_cast<size_t>(col)];
}

Point OccupancyGrid2D::cell_to_world(const Cell& c) const {
  return Point{origin_x_ + (c.col + 0.5) * resolution_,
               origin_y_ + ((rows_ - 1 - c.row) + 0.5) * resolution_};
}

Cell OccupancyGrid2D::world_to_cell(double x, double y) const {
  int col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  int row = (rows_ - 1) - static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return Cell{row, col};
}

std::vector<std::pair<Cell, double>> OccupancyGrid2D::neighbors(const Cell& s) const {
  // Fixed order (up, down, left, right), then the wait self-loop — a fixed
  // cross-language emission order so deterministic searches with a stable
  // tie-break settle on the same path in both C++ and Python. Corner-cutting is
  // not a concern on a 4-connected grid: reaching a diagonal never crosses one.
  static const int kR[] = {-1, 1, 0, 0};
  static const int kC[] = {0, 0, -1, 1};
  std::vector<std::pair<Cell, double>> out;
  for (int i = 0; i < 4; ++i) {
    Cell n{s.row + kR[i], s.col + kC[i]};
    if (is_free(n.row, n.col)) out.push_back({n, 1.0});
  }
  out.push_back({s, 1.0});  // wait action — always available
  return out;
}

double OccupancyGrid2D::heuristic(const Cell& a, const Cell& b) const {
  return static_cast<double>(std::abs(a.row - b.row) + std::abs(a.col - b.col));
}

std::set<Capability> OccupancyGrid2D::capabilities() const { return {Capability::DISCRETE_SPACE}; }

}  // namespace mrmp::maps
