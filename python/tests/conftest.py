"""Shared test helpers: repo paths, small in-memory grids, temp configs.

Configs live under configs/<section>/ — the section is part of a config's
identity (site sections mirror code directories), so lookups search both
sections by slug; slugs are globally unique across them."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from mrmp.core.params import ParamSet
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
