"""OccupancyGrid2D — ROS-style grid map providing BOTH capabilities.

world<->grid conversion lives ONLY here (map layer owns the coordinate frames).
Discrete state is a Cell (row, col); world points stay floats only in this layer.
The move set is the MAPF action model: 4-connected moves plus a wait self-loop,
every action costing one time step — so g-values count elapsed steps and every
agent's cost is directly comparable (see core/capabilities.py).

The same raster also answers the ContinuousSpace queries for disc robots: a
configuration q is free iff the disc of radius r around q overlaps no obstacle
cell, and a segment is free iff its swept disc stays clear. Every predicate is an
exact float expression (core/geometry primitives, fixed operation order) so all
engines decide every boundary case on identical bits; collision means STRICT
overlap — touching counts as free.
"""

from __future__ import annotations

import math

import numpy as np

from mrmp.core.capabilities import Capability, MapBase
from mrmp.core.geometry import point_segment_distance, segments_intersect
from mrmp.core.types import Cell, Point

# 4-connected moves in fixed order (up, down, left, right); the wait action is
# appended after them. Fixed order = deterministic tie-breaking across languages.
_MOVES_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]


class OccupancyGrid2D(MapBase):
    def __init__(
        self,
        pixels: np.ndarray,
        resolution: float,
        origin: tuple[float, float, float],
        occupied_thresh: float = 0.65,
        free_thresh: float = 0.196,
    ) -> None:
        self._height, self._width = int(pixels.shape[0]), int(pixels.shape[1])
        self._resolution = resolution
        self._origin_x, self._origin_y = origin[0], origin[1]
        # occupied_thresh is accepted for ROS yaml signature compatibility but not
        # used: traversability is decided by free_thresh alone (occupied/unknown are
        # blocked), so a separate occupied cutoff would be redundant.
        # occ = 1 - p/255; free iff occ <= free_thresh; anything not clearly free
        # (occupied or unknown) is non-traversable.
        occ = 1.0 - pixels.astype(np.float64) / 255.0
        self._free = occ <= free_thresh

    # --- dimensions -------------------------------------------------------
    @property
    def height(self) -> int:
        return self._height

    @property
    def width(self) -> int:
        return self._width

    @property
    def resolution(self) -> float:
        return self._resolution

    def free_mask(self) -> np.ndarray:
        """Boolean [H, W] mask of traversable cells (read-only view for viz)."""
        return self._free

    # --- coordinate frames (owned here only) ------------------------------
    def cell_to_world(self, row: int, col: int) -> Point:
        x = self._origin_x + (col + 0.5) * self._resolution
        y = self._origin_y + ((self._height - 1 - row) + 0.5) * self._resolution
        return (x, y)

    def world_to_cell(self, x: float, y: float) -> Cell:
        col = int(math.floor((x - self._origin_x) / self._resolution))
        row = (self._height - 1) - int(math.floor((y - self._origin_y) / self._resolution))
        return (row, col)

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self._height and 0 <= col < self._width

    def is_free_cell(self, row: int, col: int) -> bool:
        return self.in_bounds(row, col) and bool(self._free[row][col])

    # --- DiscreteSpace ----------------------------------------------------
    def neighbors(self, s: Cell) -> list[tuple[Cell, float]]:
        """Passable 4-connected moves (fixed order up/down/left/right) then the
        wait self-loop; every action costs one time step. Corner-cutting is not a
        concern on a 4-connected grid — reaching a diagonal never crosses a corner."""
        out: list[tuple[Cell, float]] = []
        for dr, dc in _MOVES_4:
            n = (s[0] + dr, s[1] + dc)
            if self.is_free_cell(*n):
                out.append((n, 1.0))
        out.append((s, 1.0))
        return out

    def heuristic(self, a: Cell, b: Cell) -> float:
        """Manhattan distance — admissible + consistent for the unit-cost
        4-connected move set (diagonals are not moves here)."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def cells(self) -> list[Cell]:
        """Every passable cell in canonical row-major order (row ascending, then
        column) — the motion graph's vertex set for uniform waypoint sampling."""
        return [
            (r, c)
            for r in range(self._height)
            for c in range(self._width)
            if self._free[r][c]
        ]

    # --- ContinuousSpace ---------------------------------------------------
    def extent(self) -> tuple[float, float, float, float]:
        """World footprint [x_min, y_min, x_max, y_max] — the rectangle continuous
        planners sample configurations uniformly from."""
        return (
            self._origin_x,
            self._origin_y,
            self._origin_x + self._width * self._resolution,
            self._origin_y + self._height * self._resolution,
        )

    def _cell_rect(self, row: int, col: int) -> tuple[float, float, float, float]:
        """World rectangle of one cell (x_lo, y_lo, x_hi, y_hi)."""
        return (
            self._origin_x + col * self._resolution,
            self._origin_y + (self._height - 1 - row) * self._resolution,
            self._origin_x + (col + 1) * self._resolution,
            self._origin_y + (self._height - row) * self._resolution,
        )

    def free_point(self, q: Point, radius: float) -> bool:
        """Free iff the disc of `radius` around q overlaps no obstacle cell in more
        than a boundary point — blocked iff dist(q, cell) < radius (and for a point
        robot, radius 0, blocked exactly when q lies on/inside a cell: distance 0).
        Exact per-cell point-to-rect distance; cells provably closer-cleared are
        skipped on the exact lower bound dx > radius."""
        qx, qy = q[0], q[1]
        for row in range(self._height):
            for col in range(self._width):
                if self._free[row][col]:
                    continue
                x_lo, y_lo, x_hi, y_hi = self._cell_rect(row, col)
                dx = max(x_lo - qx, qx - x_hi, 0.0)
                # dx > radius guarantees dist >= dx > radius: not blocked by this cell.
                if dx > radius:
                    continue
                dy = max(y_lo - qy, qy - y_hi, 0.0)
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < radius or dist == 0.0:
                    return False
        return True

    def segment_free(self, a: Point, b: Point, radius: float) -> bool:
        """Free iff every point of segment a->b is free for a disc of `radius` —
        blocked iff the segment's distance to some obstacle cell is strictly below
        the radius, or zero (touching/entering the closed cell). Exact segment-vs-
        rect distance per cell; cells whose axis-aligned separation already exceeds
        the radius are skipped on that exact lower bound."""
        min_x = min(a[0], b[0])
        max_x = max(a[0], b[0])
        min_y = min(a[1], b[1])
        max_y = max(a[1], b[1])
        for row in range(self._height):
            for col in range(self._width):
                if self._free[row][col]:
                    continue
                x_lo, y_lo, x_hi, y_hi = self._cell_rect(row, col)
                # Lower bound on dist(segment, rect): |p - q| >= |dx| and >= |dy|
                # componentwise, so either axis alone already clears the cell.
                if max(x_lo - max_x, min_x - x_hi, 0.0) > radius:
                    continue
                if max(y_lo - max_y, min_y - y_hi, 0.0) > radius:
                    continue
                dist = self._segment_rect_distance(a, b, (x_lo, y_lo, x_hi, y_hi))
                if dist < radius or dist == 0.0:
                    return False
        return True

    def _point_rect_distance(self, p: Point, rect: tuple[float, float, float, float]) -> float:
        x_lo, y_lo, x_hi, y_hi = rect
        dx = max(x_lo - p[0], p[0] - x_hi, 0.0)
        dy = max(y_lo - p[1], p[1] - y_hi, 0.0)
        return math.sqrt(dx * dx + dy * dy)

    def _segment_rect_distance(
        self, a: Point, b: Point, rect: tuple[float, float, float, float]
    ) -> float:
        """Distance between segment a->b and the closed cell rectangle: 0 on any
        intersection (endpoint inside counts), else min over {endpoint-to-rect} of
        the endpoints and corner-to-segment distances of all four corners — for two
        disjoint segments the closest pair always includes an endpoint of one."""
        x_lo, y_lo, x_hi, y_hi = rect
        c0: Point = (x_lo, y_lo)
        c1: Point = (x_hi, y_lo)
        c2: Point = (x_hi, y_hi)
        c3: Point = (x_lo, y_hi)
        edges = ((c0, c1), (c1, c2), (c2, c3), (c3, c0))
        for e in edges:
            if segments_intersect(a, b, e[0], e[1]):
                return 0.0
        best = min(self._point_rect_distance(a, rect), self._point_rect_distance(b, rect))
        for corner in (c0, c1, c2, c3):
            d = point_segment_distance(corner, a, b)
            if d < best:
                best = d
        return best

    def capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE, Capability.CONTINUOUS_SPACE}
