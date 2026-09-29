"""Map / scenario loaders. Dispatch on the yaml `type` field, not the extension."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from mrmp.core.capabilities import MapBase
from mrmp.core.types import Point

from .occupancy_grid import OccupancyGrid2D
from .pgm import read_pgm


def load_map(path: str | Path) -> MapBase:
    path = Path(path)
    with open(path, encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    map_type = raw.get("type")
    if map_type != "occupancy_grid":
        raise ValueError(f"unsupported map type {map_type!r} (only occupancy_grid)")
    image_path = (path.parent / raw["image"]).resolve()
    _, _, pixels = read_pgm(str(image_path))
    origin = raw["origin"]
    return OccupancyGrid2D(
        pixels=pixels,
        resolution=float(raw["resolution"]),
        origin=(float(origin[0]), float(origin[1]), float(origin[2])),
        occupied_thresh=float(raw.get("occupied_thresh", 0.65)),
        free_thresh=float(raw.get("free_thresh", 0.196)),
    )


@dataclass(frozen=True)
class AgentSpec:
    """One agent's scenario spec in world coords. Discrete planners get Cells via
    the demo driver's grid.world_to_cell conversion (coordinate frames stay owned
    by the map layer, per the repo rule); continuous planners use the raw Points
    and the disc `radius` (0.0 = point robot when the scenario omits it)."""

    start: Point
    goal: Point
    radius: float = 0.0


@dataclass(frozen=True)
class Scenario:
    map_path: str  # resolved absolute path to the map yaml
    # One entry per agent, in agent-index order (agent 0 is the first entry).
    agents: tuple[AgentSpec, ...]


def load_scenario(path: str | Path) -> Scenario:
    path = Path(path)
    with open(path, encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    # This repo is multi-agent planning only: a scenario without an `agents:` list
    # (a single start/goal pair) belongs to the sibling navigation repo.
    if "agents" not in raw:
        raise ValueError(f"{path} declares no 'agents:' list — MRMP scenarios require one")
    agents = tuple(
        AgentSpec(
            start=(float(a["start"][0]), float(a["start"][1])),
            goal=(float(a["goal"][0]), float(a["goal"][1])),
            radius=float(a.get("radius", 0.0)),
        )
        for a in raw["agents"]
    )
    return Scenario(map_path=str((path.parent / raw["map"]).resolve()), agents=agents)
