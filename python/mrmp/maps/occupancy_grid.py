"""OccupancyGrid2D — ROS-style grid map providing the DiscreteSpace capability.

world<->grid conversion lives ONLY here (map layer owns the coordinate frames).
Discrete state is a Cell (row, col); world points stay floats only in this layer.
The move set is the MAPF action model: 4-connected moves plus a wait self-loop,
every action costing one time step — so g-values count elapsed steps and every
agent's cost is directly comparable (see core/capabilities.py).
"""

from __future__ import annotations

import math

import numpy as np

from mrmp.core.capabilities import Capability, MapBase
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
        return self.in_bounds(row, col) and bool(self._free[row, col])

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

    def capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE}
