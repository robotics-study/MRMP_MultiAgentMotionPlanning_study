"""Map / scenario loading: type dispatch, path resolution, agents parsing."""

from __future__ import annotations

from pathlib import Path

import pytest
from conftest import REPO_ROOT

from mrmp.core.capabilities import Capability
from mrmp.maps.loader import AgentSpec, load_map, load_scenario
from mrmp.maps.occupancy_grid import OccupancyGrid2D

_MAZE = REPO_ROOT / "maps" / "grid" / "maze01.yaml"
_SCENARIO = REPO_ROOT / "maps" / "scenarios" / "maze01_two.yaml"


def test_load_occupancy_grid() -> None:
    grid = load_map(_MAZE)
    assert isinstance(grid, OccupancyGrid2D)
    assert grid.supports(Capability.DISCRETE_SPACE)
    assert (grid.height, grid.width) == (20, 20)
    # Manhattan heuristic on the unit-cost move set.
    assert grid.heuristic((0, 0), (2, 3)) == pytest.approx(5.0)


def test_load_scenario_resolves_map_path_and_agents() -> None:
    scenario = load_scenario(_SCENARIO)
    assert Path(scenario.map_path) == _MAZE.resolve()
    # World coords arrive untouched — the demo driver converts to cells.
    assert scenario.agents == (
        AgentSpec(start=(0.75, 1.25), goal=(8.25, 7.25)),
        AgentSpec(start=(8.25, 1.25), goal=(0.75, 7.25)),
    )


def test_unsupported_map_type_raises(tmp_path: Path) -> None:
    p = tmp_path / "g.yaml"
    p.write_text("type: graph\nnodes: []\nedges: []\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_map(p)


def test_single_robot_scenario_rejected(tmp_path: Path) -> None:
    # This repo is multi-agent only: a start/goal scenario without `agents:` is the
    # sibling single-robot format, not something load_scenario may silently accept.
    p = tmp_path / "s.yaml"
    p.write_text(f"map: {_MAZE}\nstart: [0.75, 1.25]\ngoal: [8.25, 7.25]\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_scenario(p)
