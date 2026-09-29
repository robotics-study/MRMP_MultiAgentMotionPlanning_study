"""Step-by-step trace recorder — the contract for visualization.

Mirrors the C++ `core/trace.hpp`. Emits JSON Lines per `spec/trace_schema.json`.
`seq` starts at 0 and increments per event; both languages serialize each event's
fields in exactly the order documented here, so parsed traces match across
languages field-for-field. Unlike single-robot nav traces, an MRMP trace carries
NO wall-clock time: replay is driven purely by `seq` order, and the only time a
space-time event ever carries is the discrete timestep (`t`) of space-time events.

A state on the wire is always a numeric pair: [row, col] ints for grid-cell
algorithms (the default, `coords` omitted) or world-point floats for continuous
planners (`planning_started` then carries `coords: "world"` plus each agent's disc
`radius`). Floats are compared parsed, not byte-wise (Python 5.0 vs C++ 5 is the
same value); ints stay ints on both sides.

A null recorder must cost nothing on the hot path: callers guard every emit with
``if recorder is not None`` so a None recorder never runs any of this code.
"""

from __future__ import annotations

import json
from collections.abc import Sequence
from types import TracebackType
from typing import Literal, TextIO

from .params import ParamValue
from .types import Cell

# A serialized state is a numeric pair -> JSON array of two numbers: an integer
# cell pair for grid algorithms, a float world-point pair for continuous ones
# (which joint state means is declared by planning_started's `coords` field). A
# joint state is the per-agent pairs flattened to [x0, y0, x1, y1, ...].
State = Sequence[float]
# Conflict / constraint kind: "vertex" = two agents on one cell at one step,
# "edge" = two agents swapping the ends of one edge (a swap is its own event —
# `cell` is one end and `to` the other; direction is not encoded).
ConflictKind = Literal["vertex", "edge"]
# Which interpretation a trace's numeric state pairs carry.
Coords = Literal["cell", "world"]


class TraceRecorder:
    def __init__(self, out: TextIO, owns: bool = False) -> None:
        self._out = out
        self._owns = owns
        self._seq = 0

    def __enter__(self) -> TraceRecorder:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    def close(self) -> None:
        # The C++ recorder writes through an ostream the caller owns; Python mirrors
        # that: open_trace() owns its file and closes it, a buffer-backed recorder
        # just flushes.
        self._out.flush()
        if self._owns:
            self._out.close()

    def _emit(self, event: str, fields: dict[str, object]) -> None:
        record: dict[str, object] = {"seq": self._seq}
        record["event"] = event
        record.update(fields)
        self._out.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._seq += 1

    # --- events -----------------------------------------------------------
    def planning_started(
        self,
        algorithm: str,
        map_path: str,
        params: dict[str, ParamValue],
        coords: Coords | None = None,
        radius: Sequence[float] | None = None,
    ) -> None:
        # params is serialized with keys sorted so the Python and C++ recorders
        # (std::map iterates sorted) emit identical field order.
        # `coords` declares how readers must read the numeric state pairs ("cell"
        # when omitted); `radius` carries each agent's disc radius in meters —
        # continuous traces only, so discrete traces keep their exact old bytes.
        fields: dict[str, object] = {
            "algorithm": algorithm,
            "map": map_path,
            "params": dict(sorted(params.items())),
        }
        if coords is not None:
            fields["coords"] = coords
        if radius is not None:
            fields["radius"] = list(radius)
        self._emit("planning_started", fields)

    def roadmap_built(
        self, agent: int, vertices: Sequence[Sequence[float]], edges: Sequence[Sequence[int]]
    ) -> None:
        # Continuous planners (dRRT family) prebuild one individual roadmap per
        # agent: `vertices` are world points in insertion order, `edges` are
        # vertex-index pairs [i, j], i < j, ordered by (min index, max index).
        self._emit(
            "roadmap_built",
            {
                "agent": agent,
                "vertices": [list(v) for v in vertices],
                "edges": [[int(e[0]), int(e[1])] for e in edges],
            },
        )

    def node_expanded(
        self,
        state: State,
        cost: float | None = None,
        agent: int | None = None,
        t: int | None = None,
    ) -> None:
        # One search-node expansion. `state` is one pair for per-agent searches
        # (prioritized planning, CBS low level) or the flattened joint state
        # (joint-space A*, dRRT). `agent` names the owning agent's sub-search when
        # the search is per-agent; `t` is the space-time step of the expanded node
        # (omitted for joint-state expansions, whose state already carries every
        # position at that instant); `cost` is the g-value.
        fields: dict[str, object] = {"state": list(state)}
        if cost is not None:
            fields["cost"] = cost
        if agent is not None:
            fields["agent"] = agent
        if t is not None:
            fields["t"] = t
        self._emit("node_expanded", fields)

    def path_found(self, path: Sequence[Sequence[float]], agent: int) -> None:
        # One space-time path per agent: path[t] is the state occupied at step t.
        # `agent` is required — every MRMP result belongs to a named agent.
        self._emit("path_found", {"path": [list(s) for s in path], "agent": agent})

    def conflict_found(
        self, kind: ConflictKind, cell: Cell, t: int, agents: tuple[int, int],
        to: Cell | None = None,
    ) -> None:
        # CBS high level (Sharon et al. 2015): two agents' current paths collide.
        # "vertex": both occupy `cell` at step t. "edge": the pair swaps `cell`
        # and `to` across step t; direction is not encoded — the pair of cells is
        # what a constraint must forbid.
        fields: dict[str, object] = {
            "kind": kind,
            "cell": list(cell),
            "t": t,
            "agents": [agents[0], agents[1]],
        }
        if to is not None:
            fields["to"] = list(to)
        self._emit("conflict_found", fields)

    def constraint_added(
        self, agent: int, kind: ConflictKind, cell: Cell, t: int, to: Cell | None = None
    ) -> None:
        # A branch of the CBS constraint tree: `agent` may not occupy `cell` at
        # step t (vertex) or traverse between `cell` and `to` across step t (edge).
        fields: dict[str, object] = {
            "agent": agent,
            "kind": kind,
            "cell": list(cell),
            "t": t,
        }
        if to is not None:
            fields["to"] = list(to)
        self._emit("constraint_added", fields)

    def planning_finished(self, success: bool, metrics: dict[str, float]) -> None:
        # metrics keys are sorted for byte-identical cross-language output.
        self._emit(
            "planning_finished",
            {"success": success, "metrics": dict(sorted(metrics.items()))},
        )


def open_trace(path: str) -> TraceRecorder:
    """Open ``path`` for writing and return a recorder that owns the file."""
    return TraceRecorder(open(path, "w", encoding="utf-8"), owns=True)
