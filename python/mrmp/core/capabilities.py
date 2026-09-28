"""Capability model: algorithms require capabilities, map types provide them.

Mirrors the C++ `core/capabilities.hpp`. Capabilities are structural Protocols
so one concrete map can satisfy a planner without a class hierarchy, and a planner
depends only on the capability it needs (never on a concrete map class). MRMP has
exactly one capability: every planner searches the same discrete grid.
"""

from __future__ import annotations

import enum
from abc import ABC, abstractmethod
from typing import Protocol

from .types import Cell


class Capability(enum.Enum):
    DISCRETE_SPACE = "discrete_space"


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
        vertex set, ordered identically across languages."""
        ...


class MapBase(ABC):
    """Base for concrete maps. Owns the single `supports` implementation."""

    @abstractmethod
    def capabilities(self) -> set[Capability]: ...

    def supports(self, c: Capability) -> bool:
        return c in self.capabilities()
