#!/usr/bin/env python3
"""Replay an MRMP trace jsonl over its map (matplotlib).

Depends only on the trace/map spec + mrmp core/maps — never on algorithm modules.
All planner state needed for visualization arrives via trace events: per-agent
expansions, per-agent space-time paths, conflicts and constraints. There is no
wall-clock time in a trace: the search phase accumulates by event `seq`, and the
execution phase replays each agent's space-time path over discrete steps.

Output modes (combinable; all but interactive are headless via the Agg backend):
  (default)          interactive window with the full accumulated frame + final
                     execution pose of every agent
  --save out.png     same accumulated frame to a PNG
  --gif out.gif      animated replay: search accumulation, then execution replay
                     (each agent walks its space-time path step by step)
  --snapshots dir/   evenly-spaced mid-search PNG snapshots + a final frame
"""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from matplotlib.axes import Axes
    from mrmp.maps.occupancy_grid import OccupancyGrid2D

Cell = tuple[int, int]

# Frame budget so a trace with thousands of events yields a watchable GIF, split
# between the two phases (search accumulation / execution replay).
_DEFAULT_TARGET_FRAMES = 150
_SEARCH_SHARE = 0.6
_DEFAULT_SNAPSHOTS = 8

# One CVD-safe hue per agent index (cycled) — identical to the docs site's agent
# palette, so a GIF and the browser replay read as the same run.
_AGENT_PALETTE = ("#0d9488", "#c2179b", "#2563eb", "#ca8a04", "#7c3aed", "#e5484d")
# Conflict marker: a thick red X on the contested cell(s) — distinct from every
# agent hue.
_CONFLICT_COLOR = "#dc2626"


# Snap normalized time to 16 shades per agent ramp: the gradient still reads
# smooth, but total distinct mark colors stay well under Pillow's 256-color GIF
# palette, so the GIF re-compresses instead of ballooning.
_COLOR_LEVELS = 15


def _quantize(t: float) -> float:
    return round(t * _COLOR_LEVELS) / _COLOR_LEVELS


@dataclass
class AgentPath:
    """One path_found event: the agent index, its normalized reveal order, and the
    space-time cells (path[t] = cell occupied at step t)."""

    agent: int
    order: float
    cells: list[Cell]


@dataclass
class Scene:
    """Draw-ready geometry extracted from a trace, ordered by event sequence.

    Each drawable element carries the normalized seq at which it appeared, so a
    frame showing "the first N events" is a prefix cut across the per-type lists.
    The execution phase replays each agent's space-time path over discrete steps.
    """

    grid: OccupancyGrid2D
    # expanded cells per agent: (cell, agent index, normalized order in [0, 1]).
    expanded: list[tuple[Cell, int, float]] = field(default_factory=list)
    # per-agent space-time paths, in reveal order.
    paths: list[AgentPath] = field(default_factory=list)
    # conflicts/constraints: cells to mark + normalized reveal order.
    conflicts: list[tuple[list[Cell], float]] = field(default_factory=list)
    constraints: list[tuple[Cell, int, float]] = field(default_factory=list)
    total_events: int = 0
    makespan: int = 0
    algorithm: str = ""


def _read_events(trace_path: str) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    with open(trace_path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                events.append(json.loads(line))
    return events


def _resolve_map(trace_path: str, events: list[dict[str, Any]], override: str | None) -> str:
    if override:
        return override
    for ev in events:
        if ev.get("event") == "planning_started" and ev.get("map"):
            candidate = Path(ev["map"])
            if candidate.exists():
                return str(candidate)
    raise SystemExit(
        f"{trace_path}: planning_started carries no loadable map path; pass --map explicitly"
    )


def build_scene(trace_path: str, map_override: str | None = None) -> Scene:
    events = _read_events(trace_path)
    from mrmp.maps.loader import load_map
    from mrmp.maps.occupancy_grid import OccupancyGrid2D

    grid = load_map(_resolve_map(trace_path, events, map_override))
    # MRMP maps are occupancy grids only (the sole DiscreteSpace provider).
    assert isinstance(grid, OccupancyGrid2D)
    scene = Scene(grid=grid)
    total = len(events) or 1
    for ev in events:
        order = ev["seq"] / (total - 1) if total > 1 else 1.0
        kind = ev.get("event")
        if kind == "planning_started":
            scene.algorithm = str(ev.get("algorithm", ""))
        elif kind == "node_expanded":
            state = [int(v) for v in ev["state"]]
            agent = ev.get("agent")
            if agent is not None:
                # Per-agent search (prioritized / CBS low level): one cell, own hue.
                scene.expanded.append(((state[0], state[1]), int(agent), order))
            else:
                # Joint-state expansion: the flattened state carries every agent's
                # cell at this instant — paint each pair in its own agent's hue.
                for k in range(len(state) // 2):
                    scene.expanded.append(((state[2 * k], state[2 * k + 1]), k, order))
        elif kind == "path_found":
            cells = [(int(p[0]), int(p[1])) for p in ev["path"]]
            scene.paths.append(AgentPath(int(ev["agent"]), order, cells))
            scene.makespan = max(scene.makespan, len(cells) - 1)
        elif kind == "conflict_found":
            cells = [(int(ev["cell"][0]), int(ev["cell"][1]))]
            if ev.get("to") is not None:
                cells.append((int(ev["to"][0]), int(ev["to"][1])))
            scene.conflicts.append((cells, order))
        elif kind == "constraint_added":
            scene.constraints.append(
                ((int(ev["cell"][0]), int(ev["cell"][1])), int(ev["agent"]), order)
            )
    scene.total_events = len(events)
    return scene


def draw(ax: Axes, scene: Scene, cutoff: float, exec_step: int | None) -> None:
    """Render the accumulated state at normalized event cutoff in [0, 1].

    Cell-unit coordinates (row 0 = top). When exec_step is not None, each agent's
    disc also sits at step exec_step of its space-time path (frozen at the path
    end after arrival), numbered so agents stay identifiable.
    """
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.colors import LinearSegmentedColormap

    ax.clear()
    grid = scene.grid
    h, w = grid.height, grid.width
    # Background: free cells light, occupied dark (row 0 on top => flipud + lower).
    grid_cmap = LinearSegmentedColormap.from_list("mrmp_grid", ["#0f172a", "#e2e8f0"])
    ax.imshow(
        np.flipud(grid.free_mask().astype(float)), cmap=grid_cmap, origin="lower",
        extent=(0, w, 0, h), vmin=0.0, vmax=1.0, interpolation="nearest", zorder=1,
    )

    # Expanded cells per agent: one raster per agent (NaN = untouched). The cell's
    # shade encodes the normalized order of that agent's expansion, so search
    # progress reads as a wave in each agent's hue.
    ramps: list[np.ndarray] = []
    for k in range(len(_AGENT_PALETTE)):
        ramp = np.full((h, w), np.nan)
        touched = False
        for (r, c), agent, order in scene.expanded:
            if agent % len(_AGENT_PALETTE) == k and order <= cutoff:
                ramp[r, c] = _quantize(order)
                touched = True
        if touched:
            ramps.append(ramp)
    for k, ramp in enumerate(ramps):
        cmap = LinearSegmentedColormap.from_list(
            f"mrmp_ramp_{k}", ["#f8fafc", _AGENT_PALETTE[k % len(_AGENT_PALETTE)]]
        )
        ax.imshow(
            np.flipud(ramp), cmap=cmap, origin="lower", extent=(0, w, 0, h),
            vmin=0.0, vmax=1.0, interpolation="nearest", alpha=0.5, zorder=2,
        )

    # Per-agent space-time paths: the whole polyline appears at its path_found seq;
    # start = filled dot, goal = hollow ring (both revealed with the path).
    for ap in scene.paths:
        if ap.order > cutoff:
            continue
        base = _AGENT_PALETTE[ap.agent % len(_AGENT_PALETTE)]
        pts = [(c + 0.5, h - 1 - r + 0.5) for (r, c) in ap.cells]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=base, lw=1.6, alpha=0.9,
                solid_capstyle="round", zorder=4)
        sx, sy = pts[0]
        gx, gy = pts[-1]
        ax.scatter([sx], [sy], s=28, color=base, edgecolors="white", linewidths=1.0, zorder=6)
        ax.scatter([gx], [gy], s=55, facecolors="none", edgecolors=base, linewidths=1.6, zorder=6)

    # Execution phase: each agent's disc at its cell at step exec_step.
    if exec_step is not None:
        for ap in scene.paths:
            base = _AGENT_PALETTE[ap.agent % len(_AGENT_PALETTE)]
            r, c = ap.cells[min(exec_step, len(ap.cells) - 1)]
            ax.scatter([c + 0.5], [h - 1 - r + 0.5], s=90, color=base, edgecolors="white",
                       linewidths=1.2, zorder=7)
            ax.text(c + 0.5, h - 1 - r + 0.5, str(ap.agent), color="white", fontsize=7,
                    ha="center", va="center", zorder=8)

    # Constraints (dashed outline in the constrained agent's hue) and conflicts
    # (thick red X on every contested cell).
    for (r, c), agent, order in scene.constraints:
        if order > cutoff:
            continue
        base = _AGENT_PALETTE[agent % len(_AGENT_PALETTE)]
        ax.add_patch(plt.Rectangle((c, h - 1 - r), 1, 1, fill=False,
                                   ec=base, lw=1.4, ls="--", zorder=3))
    for cells, order in scene.conflicts:
        if order > cutoff:
            continue
        for (r, c) in cells:
            ax.plot([c + 0.2, c + 0.8], [h - r - 0.8, h - r - 0.2], color=_CONFLICT_COLOR,
                    lw=2.4, solid_capstyle="round", zorder=6)
            ax.plot([c + 0.2, c + 0.8], [h - r - 0.2, h - r - 0.8], color=_CONFLICT_COLOR,
                    lw=2.4, solid_capstyle="round", zorder=6)

    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.set_aspect("equal")
    ax.axis("off")
    label = scene.algorithm or "trace"
    ax.set_title(label if exec_step is None else f"{label}  execution t={exec_step}")


def main() -> None:
    parser = argparse.ArgumentParser(description="replay an MRMP trace over its map")
    parser.add_argument("trace", help="trace .jsonl path")
    parser.add_argument("--map", default=None, help="override the map yaml from planning_started")
    parser.add_argument("--save", default=None, help="write the final accumulated frame to a PNG")
    parser.add_argument("--gif", default=None, help="animated replay (search + execution)")
    parser.add_argument("--snapshots", default=None, help="directory for evenly-spaced PNGs")
    args = parser.parse_args()

    scene = build_scene(args.trace, args.map)
    os.environ.setdefault("MPLBACKEND", "Agg" if (args.save or args.gif or args.snapshots) else "")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    size = (8.0, 8.0)

    def new_fig() -> tuple[Any, Axes]:
        fig, ax = plt.subplots(figsize=size)
        return fig, ax

    if args.gif:
        search_frames = int(_DEFAULT_TARGET_FRAMES * _SEARCH_SHARE)
        exec_frames = max(1, _DEFAULT_TARGET_FRAMES - search_frames)
        fig, ax = new_fig()

        def update(i: int) -> list[Axes]:
            if i < search_frames:
                draw(ax, scene, (i + 1) / search_frames, None)
            else:
                step = round((i - search_frames + 1) / max(1, exec_frames - 1) * scene.makespan)
                draw(ax, scene, 1.0, step)
            return [ax]

        anim = FuncAnimation(fig, update, frames=search_frames + exec_frames, blit=False)
        Path(args.gif).parent.mkdir(parents=True, exist_ok=True)
        anim.save(args.gif, writer=PillowWriter(fps=30))
        plt.close(fig)
        print(f"gif: {args.gif}")

    if args.snapshots:
        out = Path(args.snapshots)
        out.mkdir(parents=True, exist_ok=True)
        for i in range(_DEFAULT_SNAPSHOTS):
            cutoff = (i + 1) / _DEFAULT_SNAPSHOTS
            last = i == _DEFAULT_SNAPSHOTS - 1
            fig, ax = new_fig()
            draw(ax, scene, 1.0 if last else cutoff, scene.makespan if last else None)
            fig.savefig(out / f"frame_{i:02d}.png", dpi=110, bbox_inches="tight")
            plt.close(fig)
        print(f"snapshots: {out}")

    if args.save:
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        fig, ax = new_fig()
        draw(ax, scene, 1.0, scene.makespan)
        fig.savefig(args.save, dpi=140, bbox_inches="tight")
        plt.close(fig)
        print(f"saved: {args.save}")

    if not (args.save or args.gif or args.snapshots):
        fig, ax = new_fig()
        draw(ax, scene, 1.0, scene.makespan)
        plt.show()


if __name__ == "__main__":
    main()
