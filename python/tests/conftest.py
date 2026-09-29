"""Shared test helpers: repo paths, small in-memory grids, temp configs.

Configs live under configs/<section>/ — the section is part of a config's
identity (site sections mirror code directories), so lookups search both
sections by slug; slugs are globally unique across them."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from mrmp.core.geometry import moving_pair_distance
from mrmp.core.params import ParamSet
from mrmp.core.types import Point
from mrmp.maps.occupancy_grid import OccupancyGrid2D

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO_ROOT / "configs"


def config(algo: str) -> ParamSet:
    found = sorted(CONFIG_DIR.rglob(f"{algo}.yaml"))
    assert len(found) == 1, f"expected exactly one configs/<section>/{algo}.yaml"
    return ParamSet.from_yaml(found[0])


def open_grid(rows: int, cols: int) -> OccupancyGrid2D:
    pixels = np.full((rows, cols), 255, dtype=np.uint16)
    return OccupancyGrid2D(pixels=pixels, resolution=1.0, origin=(0.0, 0.0, 0.0))


def grid_from(free_rows: list[str]) -> OccupancyGrid2D:
    """Build a grid from ascii rows: '.' = free (255), '#' = occupied (0)."""
    pixels = np.array(
        [[255 if ch == "." else 0 for ch in row] for row in free_rows], dtype=np.uint16
    )
    return OccupancyGrid2D(pixels=pixels, resolution=1.0, origin=(0.0, 0.0, 0.0))


def assert_joint_valid(
    paths: list[list[Point]], radii: list[float], grid: OccupancyGrid2D
) -> None:
    """Every waypoint free for its own disc, and every simultaneous motion segment
    pair apart by at least the radius sum (strict overlap is collision). Past an
    agent's trimmed path end its final point clamps — a wait in space-time."""
    horizon = max(len(p) - 1 for p in paths)
    for i, path in enumerate(paths):
        for point in path:
            assert grid.free_point(point, radii[i]), f"agent {i} waypoint on an obstacle"
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            for t in range(1, horizon + 1):
                a_prev = paths[i][t - 1] if t - 1 < len(paths[i]) else paths[i][-1]
                a_now = paths[i][t] if t < len(paths[i]) else paths[i][-1]
                b_prev = paths[j][t - 1] if t - 1 < len(paths[j]) else paths[j][-1]
                b_now = paths[j][t] if t < len(paths[j]) else paths[j][-1]
                d = moving_pair_distance(a_prev, a_now, b_prev, b_now)
                assert d >= radii[i] + radii[j], f"discs overlap during step {t}"


def write_config(
    path: Path,
    algorithm: str,
    params: list[dict[str, object]],
    section: str = "search",
    scenarios: list[str] | None = None,
) -> Path:
    # scenarios is a required config key (routing for bench/export); tests that do
    # not route anywhere just declare the empty list.
    doc = {
        "algorithm": algorithm,
        "section": section,
        "scenarios": list(scenarios or []),
        "params": params,
    }
    path.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return path
