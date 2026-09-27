"""OccupancyGrid2D geometry, occupancy, MAPF move set (4-connected + wait)."""

from __future__ import annotations

import pytest
from conftest import grid_from, open_grid

from mrmp.core.capabilities import Capability
from mrmp.maps.occupancy_grid import OccupancyGrid2D


def test_world_cell_round_trip() -> None:
    grid = open_grid(4, 4)
    for row in range(4):
        for col in range(4):
            x, y = grid.cell_to_world(row, col)
            assert grid.world_to_cell(x, y) == (row, col)


def test_known_geometry() -> None:
    # origin bottom-left, row 0 = top: top-left cell center sits high in world y.
    grid = open_grid(4, 4)
    assert grid.cell_to_world(0, 0) == pytest.approx((0.5, 3.5))
    assert grid.world_to_cell(0.5, 3.5) == (0, 0)


def test_occupancy_thresholding() -> None:
    # 255 -> free, 0 -> occupied, mid-gray -> unknown -> blocked.
    import numpy as np

    pixels = np.array([[255, 0, 128]], dtype=np.uint16)
    grid = OccupancyGrid2D(pixels, resolution=1.0, origin=(0.0, 0.0, 0.0))
    assert grid.is_free_cell(0, 0) is True
    assert grid.is_free_cell(0, 1) is False
    assert grid.is_free_cell(0, 2) is False


def test_neighbors_fixed_order_with_wait() -> None:
    # Interior cell: up/down/left/right in fixed order, then the wait self-loop —
    # all cost 1.0 (one time step per action). Order is a cross-language contract.
    grid = open_grid(3, 3)
    assert grid.neighbors((1, 1)) == [((0, 1), 1.0), ((2, 1), 1.0),
                                      ((1, 0), 1.0), ((1, 2), 1.0), ((1, 1), 1.0)]


def test_neighbors_blocked_and_out_of_bounds_excluded() -> None:
    grid = grid_from([".#.", "###", "..."])
    # (0,0): up/right blocked or out of bounds; down is occupied; left out of bounds.
    assert grid.neighbors((0, 0)) == [((0, 0), 1.0)]  # wait only


def test_wait_always_available() -> None:
    # A fully boxed-in cell still has the wait action — an agent can always stand
    # still; infeasibility comes from no passable moves, never from no actions.
    grid = grid_from([".#.", "#.#", ".#."])
    assert grid.neighbors((1, 1)) == [((1, 1), 1.0)]


def test_heuristic_is_manhattan() -> None:
    grid = open_grid(4, 4)
    assert grid.heuristic((0, 0), (2, 3)) == pytest.approx(5.0)
    # Diagonal distance is Manhattan, not Euclidean (no diagonal moves).
    assert grid.heuristic((0, 0), (1, 1)) == pytest.approx(2.0)


def test_capabilities() -> None:
    assert open_grid(2, 2).capabilities() == {Capability.DISCRETE_SPACE}
