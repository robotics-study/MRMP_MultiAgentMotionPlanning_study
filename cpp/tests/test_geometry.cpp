// Exact float geometry (core/geometry) + the ContinuousSpace capability — the
// exact mirrors of python/tests/test_geometry.py. Every predicate here is a fixed
// IEEE-754 expression; these tests pin exact bit values, not approximations,
// because cross-language parity of every continuous planner rests on identical
// decisions at boundary cases. Collision semantics: obstacle cells are CLOSED
// squares; blocked iff distance < radius (strict), and a point robot (radius 0)
// is blocked exactly when it sits on/inside a cell.

#include "mrmp/core/geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "mrmp/maps/occupancy_grid.hpp"

namespace mrmp {

using core::Point;

// Mirror of the Python test grid: ASCII rows ('.' free, '#' occupied), resolution
// 1.0, origin (0,0) — so cell rects land on exact integers.
namespace {
maps::OccupancyGrid2D grid_1(const std::vector<std::string>& rows) {
  int h = static_cast<int>(rows.size());
  int w = static_cast<int>(rows.empty() ? 0 : rows[0].size());
  std::vector<bool> free_cells(static_cast<size_t>(h) * w);
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) free_cells[static_cast<size_t>(r) * w + c] = rows[r][c] == '.';
  }
  return maps::OccupancyGrid2D(h, w, 1.0, 0.0, 0.0, std::move(free_cells));
}

Point pt(double x, double y) { return Point{x, y}; }
}  // namespace

// --- core/geometry: exact values -----------------------------------------------

TEST(Geometry, PointSegmentExact) {
  // Foot inside the segment: exact perpendicular distance.
  EXPECT_EQ(core::point_segment_distance(pt(1.0, 1.0), pt(0.0, 0.0), pt(2.0, 0.0)), 1.0);
  // Foot beyond b clamps to b: sqrt(1+1) exactly as computed by the fixed order.
  EXPECT_EQ(core::point_segment_distance(pt(3.0, 1.0), pt(0.0, 0.0), pt(2.0, 0.0)),
            std::sqrt(2.0));
  // Degenerate segment a == b: distance to the point itself.
  EXPECT_EQ(core::point_segment_distance(pt(3.0, 4.0), pt(1.0, 1.0), pt(1.0, 1.0)),
            std::sqrt(13.0));
}

TEST(Geometry, SegmentsIntersectCases) {
  // Proper crossing.
  EXPECT_TRUE(core::segments_intersect(pt(0, 0), pt(1, 1), pt(0, 1), pt(1, 0)));
  // Collinear overlap.
  EXPECT_TRUE(core::segments_intersect(pt(0, 0), pt(2, 0), pt(1, 0), pt(3, 0)));
  // Touching at a shared endpoint counts (closed segments).
  EXPECT_TRUE(core::segments_intersect(pt(0, 0), pt(1, 0), pt(1, 0), pt(1, 2)));
  // Parallel and apart; collinear and apart.
  EXPECT_FALSE(core::segments_intersect(pt(0, 0), pt(1, 0), pt(0, 1), pt(1, 1)));
  EXPECT_FALSE(core::segments_intersect(pt(0, 0), pt(1, 0), pt(2, 0), pt(3, 0)));
}

TEST(Geometry, MovingPairDistance) {
  // Head-on swap along one line: relative motion passes through the origin.
  EXPECT_EQ(core::moving_pair_distance(pt(0, 0), pt(2, 0), pt(2, 0), pt(0, 0)), 0.0);
  // Parallel equal-velocity translation: relative position is constant; the
  // distance is |w| exactly (degenerate relative segment).
  EXPECT_EQ(core::moving_pair_distance(pt(0, 0), pt(2, 0), pt(2, 3), pt(4, 3)), std::sqrt(13.0));
  // Anti-parallel translation on parallel lines: the relative segment is the
  // horizontal line y = -2 from x=-4 to x=4; closest approach is exactly 2.
  EXPECT_EQ(core::moving_pair_distance(pt(0, 0), pt(4, 0), pt(4, 2), pt(0, 2)), 2.0);
}

// --- ContinuousSpace on the grid ------------------------------------------------

TEST(GridContinuous, FreeStrictOverlapSemantics) {
  // One obstacle cell covering x,y in [0,1].
  auto grid = grid_1({"#."});
  EXPECT_TRUE(grid.free_point(pt(2.0, 2.0), 0.0));   // far away, point robot
  EXPECT_FALSE(grid.free_point(pt(0.5, 0.5), 0.0));  // inside the cell: distance 0 blocks r=0
  EXPECT_FALSE(grid.free_point(pt(1.0, 0.5), 0.0));  // on the boundary counts as on the cell
  EXPECT_FALSE(grid.free_point(pt(1.0, 0.5), 0.2));  // disc overlaps the interior
  EXPECT_TRUE(grid.free_point(pt(2.0, 0.5), 1.0));   // grazing at exactly r stays free
  EXPECT_FALSE(grid.free_point(pt(2.0, 0.5), 1.5));  // overlap: distance 1 < 1.5
}

TEST(GridContinuous, SegmentFreeStrictOverlapSemantics) {
  // One full-height wall column: obstacle cells cover x in [0,1], y in [0,4].
  auto grid = grid_1({"#", "#", "#", "#"});
  // Segment grazing the wall face at exactly radius distance stays free...
  EXPECT_TRUE(grid.segment_free(pt(2.0, 2.0), pt(2.0, 3.0), 1.0));
  // ...while a wider disc overlaps it.
  EXPECT_FALSE(grid.segment_free(pt(2.0, 2.0), pt(2.0, 3.0), 1.5));
  // A segment entering the wall blocks even a point robot (distance 0).
  EXPECT_FALSE(grid.segment_free(pt(0.5, 2.0), pt(3.0, 2.0), 0.0));
  // Touching the wall corner exactly still counts as touching (distance 0).
  EXPECT_FALSE(grid.segment_free(pt(1.0, 2.0), pt(3.0, 2.0), 0.5));
}

}  // namespace mrmp
