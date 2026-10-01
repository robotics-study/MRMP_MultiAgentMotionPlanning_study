#!/usr/bin/env python3
"""Replay an MRMP trace jsonl over its map (matplotlib).

Depends only on the trace/map spec + mrmp core/maps — never on algorithm modules.
All planner state needed for visualization arrives via trace events: per-agent
expansions, per-agent space-time paths, roadmaps, conflicts and constraints. There
is no wall-clock time in a trace: the search phase accumulates by event `seq`, and
the execution phase replays each agent's space-time path over discrete steps (cell
traces) or with linear interpolation between waypoints (world traces).

A trace declares its state interpretation via planning_started's `coords` field:
"cell" (default) draws expanded states as filled cells and paths as cell-center
polylines; "world" (continuous planners, dRRT family) draws expanded joint states
as dots at their world points, each agent's individual roadmap (roadmap_built
events) as faint edges + vertex dots, and execution discs at the TRUE radius from
the trace's `radius` field. A timed (kinodynamic) trace keeps cell pairs but adds
per-agent velocity limits (`vmax`) and schedule_found events instead of path_found:
execution replays the uniform velocity model — dwell on a cell until departure
(next arrival minus 1/vmax), traverse at exactly vmax — over fractional time.

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

Point = tuple[float, float]

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
    space-time points (points[t] = state occupied at step t; cell pairs in "cell"
    mode, world points in "world" mode)."""

    agent: int
    order: float
    points: list[Point]


@dataclass
class Roadmap:
    """One roadmap_built event: the agent's individual PRM (insertion-order vertices
    + index-pair edges) — continuous traces only."""

    agent: int
    order: float
    vertices: list[Point]
    edges: list[tuple[int, int]]


@dataclass
class Schedule:
    """One schedule_found event (timed/kinodynamic traces): the wait-free route and
    each retained location's earliest arrival time. Execution replays the uniform
    velocity model: dwell on cells[i] until departure D_i = times[i+1] − 1/vmax,
    traverse at exactly vmax, arrive exactly at times[i+1]."""

    agent: int
    order: float
    cells: list[Point]
    times: list[float]


def timed_position(sch: Schedule, vmax: float, tau: float) -> Point:
    """Position (row, col in cell units) at time tau under the uniform velocity
    model: dwell on c_i until D_i = T_{i+1} − 1/vmax, then linear at exactly
    vmax so arrival lands exactly on the scheduled time. Before the departure the
    position IS the cell — inside a dwell there is no traversal to extrapolate."""
    times = sch.times
    if tau <= times[0]:
        return sch.cells[0]
    for i in range(len(times) - 1):
        arrive = times[i + 1]
        depart = arrive - 1.0 / vmax
        if tau < depart:
            return sch.cells[i]  # dwell — still on the cell, never extrapolated past it
        if tau < arrive:
            fraction = (tau - depart) * vmax
            a, b = sch.cells[i], sch.cells[i + 1]
            return (a[0] + (b[0] - a[0]) * fraction, a[1] + (b[1] - a[1]) * fraction)
    return sch.cells[-1]


@dataclass
class Scene:
    """Draw-ready geometry extracted from a trace, ordered by event sequence.

    Each drawable element carries the normalized seq at which it appeared, so a
    frame showing "the first N events" is a prefix cut across the per-type lists.
    The execution phase replays each agent's space-time path over discrete steps
    (cell mode) or fractional τ with linear interpolation (world mode)."""

    grid: OccupancyGrid2D
    # State interpretation declared by planning_started ("cell" default).
    coords: str = "cell"
    # Timed (kinodynamic) trace: schedule_found events replace path_found and the
    # execution phase replays the uniform velocity model over fractional time.
    timed: bool = False
    # Per-agent disc radius in meters (continuous traces only) — the execution
    # discs draw at this TRUE radius, not one cell.
    radius: list[float] = field(default_factory=list)
    # Per-agent velocity limit in cells per time unit (timed traces only).
    vmax: list[float] = field(default_factory=list)
    # expanded states per agent: (point, agent index, normalized order in [0, 1]).
    # In "cell" mode a point is still a (row, col) pair — the renderer knows which.
    expanded: list[tuple[Point, int, float]] = field(default_factory=list)
    # per-agent space-time paths, in reveal order.
    paths: list[AgentPath] = field(default_factory=list)
    # per-agent timed routes (timed traces), in reveal order.
    schedules: list[Schedule] = field(default_factory=list)
    # individual roadmaps (continuous traces): vertices + edges per agent.
    roadmaps: list[Roadmap] = field(default_factory=list)
    # conflicts/constraints: cells to mark + normalized reveal order.
    conflicts: list[tuple[list[Point], float]] = field(default_factory=list)
    constraints: list[tuple[Point, int, float]] = field(default_factory=list)
    total_events: int = 0
    makespan: float = 0.0
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
    # MRMP maps are occupancy grids only (the sole ContinuousSpace provider).
    assert isinstance(grid, OccupancyGrid2D)
    scene = Scene(grid=grid)
    total = len(events) or 1
    for ev in events:
        order = ev["seq"] / (total - 1) if total > 1 else 1.0
        kind = ev.get("event")
        if kind == "planning_started":
            scene.algorithm = str(ev.get("algorithm", ""))
            # The coords declaration arrives with event 0, before anything to render.
            scene.coords = str(ev.get("coords", "cell"))
            if ev.get("radius") is not None:
                scene.radius = [float(r) for r in ev["radius"]]
            if ev.get("vmax") is not None:
                # A vmax field IS the timed declaration: routes arrive via
                # schedule_found, execution replays the uniform velocity model.
                scene.timed = True
                scene.vmax = [float(v) for v in ev["vmax"]]
        elif kind == "schedule_found":
            route = [(float(c[0]), float(c[1])) for c in ev["cells"]]
            times = [float(t) for t in ev["times"]]
            scene.schedules.append(Schedule(int(ev["agent"]), order, route, times))
            scene.makespan = max(scene.makespan, max(times))
        elif kind == "roadmap_built":
            vertices = [(float(v[0]), float(v[1])) for v in ev.get("vertices", [])]
            edges = [(int(e[0]), int(e[1])) for e in ev.get("edges", [])]
            scene.roadmaps.append(Roadmap(int(ev["agent"]), order, vertices, edges))
        elif kind == "node_expanded":
            # World states are float pairs — never int-cast here; cell traces carry
            # exact integer values through as floats and the renderer indexes with them.
            state = [float(v) for v in ev["state"]]
            agent = ev.get("agent")
            if agent is not None:
                # Per-agent search (prioritized / CBS low level): one pair, own hue.
                scene.expanded.append(((state[0], state[1]), int(agent), order))
            else:
                # Joint-state expansion: the flattened state carries every agent's
                # position at this instant — paint each pair in its own agent's hue.
                for k in range(len(state) // 2):
                    scene.expanded.append(((state[2 * k], state[2 * k + 1]), k, order))
        elif kind == "path_found":
            points = [(float(p[0]), float(p[1])) for p in ev["path"]]
            scene.paths.append(AgentPath(int(ev["agent"]), order, points))
            scene.makespan = max(scene.makespan, len(points) - 1)
        elif kind == "conflict_found":
            cells: list[Point] = [(float(ev["cell"][0]), float(ev["cell"][1]))]
            if ev.get("to") is not None:
                cells.append((float(ev["to"][0]), float(ev["to"][1])))
            scene.conflicts.append((cells, order))
        elif kind == "constraint_added":
            scene.constraints.append(
                ((float(ev["cell"][0]), float(ev["cell"][1])), int(ev["agent"]), order)
            )
    scene.total_events = len(events)
    return scene


def draw(ax: Axes, scene: Scene, cutoff: float, exec_step: float | None) -> None:
    """Render the accumulated state at normalized event cutoff in [0, 1].

    Cell-unit display coordinates (row 0 = top): cell-mode states are (row, col)
    pairs drawn as cells; world-mode states convert via the map's own frame —
    u = (x - origin_x)/res, v = h - (y - origin_y)/res — so both modes land on
    identical pixels for a cell center. When exec_step is not None, each agent's
    disc also sits at step exec_step of its space-time path (linear interpolation
    between waypoints in world mode; frozen at the path end after arrival). Timed
    traces instead replay every schedule under the uniform velocity model at time
    τ = exec_step (dwell, then traverse at exactly vmax — timed_position).
    """
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.patches import Circle

    ax.clear()
    grid = scene.grid
    h, w = grid.height, grid.width
    ox, oy = grid.origin
    res = grid.resolution
    world = scene.coords == "world"

    def disp(p: Point) -> tuple[float, float]:
        """State pair → cell-unit display coords (cell mode: its center)."""
        if not world:
            return (p[1] + 0.5, h - 1 - p[0] + 0.5)
        return ((p[0] - ox) / res, h - (p[1] - oy) / res)

    # Background: free cells light, occupied dark (row 0 on top => flipud + lower).
    grid_cmap = LinearSegmentedColormap.from_list("mrmp_grid", ["#0f172a", "#e2e8f0"])
    ax.imshow(
        np.flipud(grid.free_mask().astype(float)), cmap=grid_cmap, origin="lower",
        extent=(0, w, 0, h), vmin=0.0, vmax=1.0, interpolation="nearest", zorder=1,
    )

    if not world:
        # Expanded cells per agent: one raster per agent (NaN = untouched). The cell's
        # shade encodes the normalized order of that agent's expansion, so search
        # progress reads as a wave in each agent's hue.
        ramps: list[np.ndarray] = []
        for k in range(len(_AGENT_PALETTE)):
            ramp = np.full((h, w), np.nan)
            touched = False
            for (r, c), agent, order in scene.expanded:
                if agent % len(_AGENT_PALETTE) == k and order <= cutoff:
                    ramp[int(r), int(c)] = _quantize(order)
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
    else:
        # World mode: each agent's individual roadmap (the implicit composite graph's
        # visible half) — faint edges + vertex dots — then the tree's expanded joint
        # states as brighter dots in each agent's hue.
        for rm in scene.roadmaps:
            if rm.order > cutoff:
                continue
            base = _AGENT_PALETTE[rm.agent % len(_AGENT_PALETTE)]
            pts = [disp(p) for p in rm.vertices]
            for (a, b) in rm.edges:
                pa, pb = pts[a], pts[b]
                ax.plot([pa[0], pb[0]], [pa[1], pb[1]], color=base, lw=0.6, alpha=0.28,
                        solid_capstyle="round", zorder=3)
            ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=7, color=base,
                       alpha=0.55, edgecolors="none", zorder=4)

    # Expanded states (world mode: dots; cell mode was the raster ramp above).
    if world:
        for k in range(len(_AGENT_PALETTE)):
            pts = [disp(p) for (p, agent, order) in scene.expanded
                   if agent % len(_AGENT_PALETTE) == k and order <= cutoff]
            if pts:
                ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=26, color=_AGENT_PALETTE[k],
                           alpha=0.95, edgecolors="white", linewidths=0.4, zorder=5)

    # Per-agent space-time paths: the whole polyline appears at its path_found seq;
    # start = filled dot, goal = hollow ring (both revealed with the path). Timed
    # traces draw the same geometry from each schedule's wait-free route instead.
    for ap in scene.paths:
        if ap.order > cutoff:
            continue
        base = _AGENT_PALETTE[ap.agent % len(_AGENT_PALETTE)]
        pts = [disp(p) for p in ap.points]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=base, lw=1.6, alpha=0.9,
                solid_capstyle="round", zorder=5)
        sx, sy = pts[0]
        gx, gy = pts[-1]
        ax.scatter([sx], [sy], s=28, color=base, edgecolors="white", linewidths=1.0, zorder=7)
        ax.scatter([gx], [gy], s=55, facecolors="none", edgecolors=base, linewidths=1.6, zorder=7)
    for sch in scene.schedules:
        if sch.order > cutoff:
            continue
        base = _AGENT_PALETTE[sch.agent % len(_AGENT_PALETTE)]
        pts = [disp(p) for p in sch.cells]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=base, lw=1.6, alpha=0.9,
                solid_capstyle="round", zorder=5)
        sx, sy = pts[0]
        gx, gy = pts[-1]
        ax.scatter([sx], [sy], s=28, color=base, edgecolors="white", linewidths=1.0, zorder=7)
        ax.scatter([gx], [gy], s=55, facecolors="none", edgecolors=base, linewidths=1.6, zorder=7)

    # Execution phase: each agent's disc at step exec_step of its path. World mode
    # interpolates linearly between waypoints (motion between waypoints IS linear)
    # and draws the disc at its true radius (meters → cell units via resolution).
    # Timed traces replay every schedule at τ = exec_step under the uniform
    # velocity model instead (point robots — no radius field on timed traces).
    if exec_step is not None and scene.timed:
        for sch in scene.schedules:
            base = _AGENT_PALETTE[sch.agent % len(_AGENT_PALETTE)]
            v = scene.vmax[sch.agent % len(scene.vmax)]
            pos = timed_position(sch, v, exec_step)
            u, vxy = disp(pos)
            ax.scatter([u], [vxy], s=90, color=base, edgecolors="white",
                       linewidths=1.2, zorder=8)
            ax.text(u, vxy, str(sch.agent), color="white", fontsize=7,
                    ha="center", va="center", zorder=9)
    elif exec_step is not None:
        for ap in scene.paths:
            base = _AGENT_PALETTE[ap.agent % len(_AGENT_PALETTE)]
            n = len(ap.points)
            if world:
                tau = min(exec_step, n - 1)
                lo = int(tau)
                f = tau - lo
                a_pt, b_pt = ap.points[lo], ap.points[min(lo + 1, n - 1)]
                pos = (a_pt[0] + (b_pt[0] - a_pt[0]) * f, a_pt[1] + (b_pt[1] - a_pt[1]) * f)
                u, v = disp(pos)
                r_disp = scene.radius[ap.agent % len(scene.radius)] / res if scene.radius else 0.5
                ax.add_patch(Circle((u, v), r_disp, facecolor=base, edgecolor="white",
                                   linewidth=1.2, zorder=8))
                ax.text(u, v, str(ap.agent), color="white", fontsize=7,
                        ha="center", va="center", zorder=9)
            else:
                r, c = ap.points[min(int(exec_step), n - 1)]
                u, v = disp((r, c))
                ax.scatter([u], [v], s=90, color=base, edgecolors="white",
                           linewidths=1.2, zorder=8)
                ax.text(u, v, str(ap.agent), color="white", fontsize=7,
                        ha="center", va="center", zorder=9)

    # Constraints (dashed outline in the constrained agent's hue) and conflicts
    # (thick red X on every contested cell) — discrete traces only.
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
                    lw=2.4, solid_capstyle="round", zorder=7)
            ax.plot([c + 0.2, c + 0.8], [h - r - 0.2, h - r - 0.8], color=_CONFLICT_COLOR,
                    lw=2.4, solid_capstyle="round", zorder=7)

    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.set_aspect("equal")
    ax.axis("off")
    label = scene.algorithm or "trace"
    ax.set_title(label if exec_step is None else f"{label}  execution t={exec_step:g}")


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
                # Cell mode steps whole cells; world mode glides fractionally; a
                # timed trace replays its schedule at the raw fractional time τ.
                raw = (i - search_frames + 1) / max(1, exec_frames - 1) * scene.makespan
                discrete_step = scene.coords != "world" and not scene.timed
                draw(ax, scene, 1.0, round(raw) if discrete_step else raw)
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
