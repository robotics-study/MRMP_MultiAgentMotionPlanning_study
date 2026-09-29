"""Exact float geometry (core/geometry) + the ContinuousSpace capability.

Every predicate here is a fixed IEEE-754 expression — these tests pin exact bit
values, not approximations, because cross-language parity of every continuous
planner rests on identical decisions at boundary cases. Collision semantics:
obstacle cells are CLOSED squares; blocked iff distance < radius (strict), and a
point robot (radius 0) is blocked exactly when it sits on/inside a cell."""

from __future__ import annotations

import math

from conftest import grid_from

from mrmp.core.geometry import moving_pair_distance, point_segment_distance, segments_intersect
from mrmp.maps.occupancy_grid import OccupancyGrid2D


# --- core/geometry: exact values ------------------------------------------------
def test_point_segment_exact() -> None:
    # Foot inside the segment: exact perpendicular distance.
    assert point_segment_distance((1.0, 1.0), (0.0, 0.0), (2.0, 0.0)) == 1.0
    # Foot beyond b clamps to b: sqrt(1+1) exactly as computed by the fixed order.
    assert point_segment_distance((3.0, 1.0), (0.0, 0.0), (2.0, 0.0)) == math.sqrt(2.0)
    # Degenerate segment a == b: distance to the point itself.
    assert point_segment_distance((3.0, 4.0), (1.0, 1.0), (1.0, 1.0)) == math.sqrt(13.0)


def test_segments_intersect_cases() -> None:
    # Proper crossing.
    assert segments_intersect((0.0, 0.0), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0)) is True
    # Collinear overlap.
    assert segments_intersect((0.0, 0.0), (2.0, 0.0), (1.0, 0.0), (3.0, 0.0)) is True
    # Touching at a shared endpoint counts (closed segments).
    assert segments_intersect((0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (1.0, 2.0)) is True
    # Parallel and apart; collinear and apart.
    assert segments_intersect((0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)) is False
    assert segments_intersect((0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)) is False


def test_moving_pair_distance() -> None:
    # Head-on swap along one line: relative motion passes through the origin.
    assert moving_pair_distance((0.0, 0.0), (2.0, 0.0), (2.0, 0.0), (0.0, 0.0)) == 0.0
    # Parallel equal-velocity translation: relative position is constant; the
    # distance is |w| exactly (degenerate relative segment).
    assert moving_pair_distance((0.0, 0.0), (2.0, 0.0), (2.0, 3.0), (4.0, 3.0)) == math.sqrt(13.0)
    # Anti-parallel translation on parallel lines: the relative segment is the
    # horizontal line y = -2 from x=-4 to x=4; closest approach is exactly 2.
    assert moving_pair_distance((0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)) == 2.0


# --- ContinuousSpace on the grid -------------------------------------------------
def _wall() -> OccupancyGrid2D:
    """One full-height wall column: obstacle cells cover x in [0,1], y in [0,4]."""
    return grid_from(["#", "#", "#", "#"])


def test_free_strict_overlap_semantics() -> None:
    grid = grid_from(["#."])  # one obstacle cell covering x,y in [0,1]
    assert grid.free_point((2.0, 2.0), 0.0) is True   # far away, point robot
    assert grid.free_point((0.5, 0.5), 0.0) is False  # inside the cell: distance 0 blocks r=0
    assert grid.free_point((1.0, 0.5), 0.0) is False  # on the boundary counts as on the cell
    assert grid.free_point((1.0, 0.5), 0.2) is False  # disc overlaps the interior
    assert grid.free_point((2.0, 0.5), 1.0) is True   # grazing at exactly r stays free
    assert grid.free_point((2.0, 0.5), 1.5) is False  # overlap: distance 1 < 1.5


def test_segment_free_strict_overlap_semantics() -> None:
    grid = _wall()
    # Segment grazing the wall face at exactly radius distance stays free...
    assert grid.segment_free((2.0, 2.0), (2.0, 3.0), 1.0) is True
    # ...while a wider disc overlaps it.
    assert grid.segment_free((2.0, 2.0), (2.0, 3.0), 1.5) is False
    # A segment entering the wall blocks even a point robot (distance 0).
    assert grid.segment_free((0.5, 2.0), (3.0, 2.0), 0.0) is False
    # Touching the wall corner exactly still counts as touching (distance 0).
    assert grid.segment_free((1.0, 2.0), (3.0, 2.0), 0.5) is False
