#include "mrmp/maps/occupancy_grid.hpp"

#include <cmath>

#include "mrmp/core/geometry.hpp"

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

std::vector<Cell> OccupancyGrid2D::cells() const {
  // Canonical row-major scan (row ascending, then column) — identical enumeration
  // order in every language, so a uniform draw over the vertex set is identical.
  std::vector<Cell> out;
  for (int r = 0; r < rows_; ++r)
    for (int c = 0; c < cols_; ++c) {
      if (is_free(r, c)) out.push_back(Cell{r, c});
    }
  return out;
}

std::array<double, 4> OccupancyGrid2D::extent() const {
  // World footprint [x_min, y_min, x_max, y_max] — same float expressions as Python.
  return {origin_x_, origin_y_, origin_x_ + cols_ * resolution_, origin_y_ + rows_ * resolution_};
}

std::array<double, 4> OccupancyGrid2D::cell_rect(int row, int col) const {
  // World rectangle (x_lo, y_lo, x_hi, y_hi) of one cell — the same float
  // expressions in every language, so boundary cases land on identical bits.
  return {origin_x_ + static_cast<double>(col) * resolution_,
          origin_y_ + static_cast<double>(rows_ - 1 - row) * resolution_,
          origin_x_ + static_cast<double>(col + 1) * resolution_,
          origin_y_ + static_cast<double>(rows_ - row) * resolution_};
}

double OccupancyGrid2D::point_rect_distance(const Point& p,
                                            const std::array<double, 4>& rect) const {
  const double dx = std::max({rect[0] - p.x, p.x - rect[2], 0.0});
  const double dy = std::max({rect[1] - p.y, p.y - rect[3], 0.0});
  return std::sqrt(dx * dx + dy * dy);
}

double OccupancyGrid2D::segment_rect_distance(const Point& a, const Point& b,
                                              const std::array<double, 4>& rect) const {
  // Distance between segment a->b and the closed cell rectangle: 0 on any
  // intersection (endpoint inside counts), else min over {endpoint-to-rect} of
  // the endpoints and corner-to-segment distances of all four corners — for two
  // disjoint segments the closest pair always includes an endpoint of one.
  const Point c0{rect[0], rect[1]};
  const Point c1{rect[2], rect[1]};
  const Point c2{rect[2], rect[3]};
  const Point c3{rect[0], rect[3]};
  const std::pair<Point, Point> edges[4] = {{c0, c1}, {c1, c2}, {c2, c3}, {c3, c0}};
  for (const auto& e : edges) {
    if (core::segments_intersect(a, b, e.first, e.second)) return 0.0;
  }
  double best = std::min(point_rect_distance(a, rect), point_rect_distance(b, rect));
  const Point corners[4] = {c0, c1, c2, c3};
  for (const Point& corner : corners) {
    const double d = core::point_segment_distance(corner, a, b);
    if (d < best) best = d;
  }
  return best;
}

bool OccupancyGrid2D::free_point(const Point& q, double radius) const {
  // Row-major scan of every blocked cell — fixed order so the first blocking
  // cell is identical in every language (the verdict is order-free anyway, but
  // debugging traces should match too).
  for (int row = 0; row < rows_; ++row) {
    for (int col = 0; col < cols_; ++col) {
      if (is_free(row, col)) continue;
      const std::array<double, 4> rect = cell_rect(row, col);
      const double dx = std::max({rect[0] - q.x, q.x - rect[2], 0.0});
      // dx > radius guarantees dist >= dx > radius: not blocked by this cell.
      if (dx > radius) continue;
      const double dy = std::max({rect[1] - q.y, q.y - rect[3], 0.0});
      const double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < radius || dist == 0.0) return false;
    }
  }
  return true;
}

bool OccupancyGrid2D::segment_free(const Point& a, const Point& b, double radius) const {
  const double min_x = std::min(a.x, b.x);
  const double max_x = std::max(a.x, b.x);
  const double min_y = std::min(a.y, b.y);
  const double max_y = std::max(a.y, b.y);
  for (int row = 0; row < rows_; ++row) {
    for (int col = 0; col < cols_; ++col) {
      if (is_free(row, col)) continue;
      const std::array<double, 4> rect = cell_rect(row, col);
      // Lower bound on dist(segment, rect): |p - q| >= |dx| and >= |dy|
      // componentwise, so either axis alone already clears the cell.
      if (std::max({rect[0] - max_x, min_x - rect[2], 0.0}) > radius) continue;
      if (std::max({rect[1] - max_y, min_y - rect[3], 0.0}) > radius) continue;
      const double dist = segment_rect_distance(a, b, rect);
      if (dist < radius || dist == 0.0) return false;
    }
  }
  return true;
}

std::set<Capability> OccupancyGrid2D::capabilities() const {
  return {Capability::DISCRETE_SPACE, Capability::CONTINUOUS_SPACE};
}

}  // namespace mrmp::maps
