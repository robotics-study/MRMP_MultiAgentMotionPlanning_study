"""Capability model: algorithms require capabilities, map types provide them.

Mirrors the C++ `core/capabilities.hpp`. Capabilities are structural Protocols
so one concrete map can satisfy several of them without a class hierarchy, and a
planner depends only on the capability it needs (never on a concrete map class).
MRMP has exactly two capabilities: every search-branch planner searches the same
discrete grid; every sampling-branch planner whose robots are geometric discs
plans on that same map's continuous free space instead.
"""

from __future__ import annotations

import enum
from abc import ABC, abstractmethod
from typing import Protocol

from .types import Cell, Point


class Capability(enum.Enum):
    DISCRETE_SPACE = "discrete_space"
    CONTINUOUS_SPACE = "continuous_space"


class DiscreteSpace(Protocol):
    """Graph-search view: enumerable successors + admissible heuristic.

    The move set IS the MAPF action model (Silver 2005): 4-connected moves plus a
    wait action — a self-loop on the current cell — and every action costs exactly
    one time step, so g-values count elapsed steps and sum-of-costs / makespan are
    directly comparable across agents. Successor order is fixed (up, down, left,
    right, then wait) because tie-breaking must be identical across languages.

    `cells()` exposes every passable cell in the same canonical row-major order on
    every platform: sampling planners (MA-RRT*) draw waypoints uniformly from the
    motion graph's vertex set, and a uniform draw needs that enumeration to be
    part of the contract, not an implementation detail.
    """

    def neighbors(self, s: Cell) -> list[tuple[Cell, float]]:
        """Return (successor, edge_cost) pairs reachable from ``s`` — passable
        4-connected moves in fixed order, then the wait self-loop; all cost 1.0."""
        ...

    def heuristic(self, a: Cell, b: Cell) -> float:
        """Manhattan distance: admissible and consistent for the unit-cost
        4-connected move set (diagonals are not moves here)."""
        ...

    def cells(self) -> list[Cell]:
        """Every passable cell in canonical row-major order — the motion graph's
        vertex set for uniform waypoint sampling."""
        ...


class ContinuousSpace(Protocol):
    """Continuous free-space view of a map for disc robots.

    A configuration q is free iff the disc of radius r around q overlaps no
    obstacle cell (touching counts as free — collision means strict overlap).
    A straight segment is free iff the swept disc stays clear, i.e. the segment's
    distance to every obstacle cell is >= r. Every predicate is an exact float
    expression evaluated in an identical operation order in Python and C++, so all
    engines decide every boundary case on identical bits (floats are compared with
    ``<`` / ``>=``, never re-serialized, inside the algorithms).

    `extent()` returns the world rectangle [x_min, x_max] x [y_min, y_max] the
    planner samples its configurations uniformly from. `area()` is mu(C_f), the
    free-space MEASURE the asymptotic-optimality radius bound reads: free cell
    count x resolution^2 — the raster's own measure (the disc-inflated region is
    deliberately NOT modeled; the algorithm's eta constant absorbs that slack).
    """

    def extent(self) -> tuple[float, float, float, float]:
        """(x_min, y_min, x_max, y_max) of the map's world footprint."""
        ...

    def area(self) -> float:
        """mu(C_f): free cell count x resolution^2 (the raster's own measure)."""
        ...

    def free_point(self, q: Point, radius: float) -> bool:
        """True iff the disc of `radius` around q overlaps no obstacle cell."""
        ...

    def segment_free(self, a: Point, b: Point, radius: float) -> bool:
        """True iff every point of segment a->b is free for a disc of `radius`
        (the swept-disc check between two roadmap vertices)."""
        ...


class MapBase(ABC):
    """Base for concrete maps. Owns the single `supports` implementation."""

    @abstractmethod
    def capabilities(self) -> set[Capability]: ...

    def supports(self, c: Capability) -> bool:
        return c in self.capabilities()
