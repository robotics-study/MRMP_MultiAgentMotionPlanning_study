"""TraceRecorder JSON Lines output + null-recorder behavior."""

from __future__ import annotations

import io
import json
from pathlib import Path

from mrmp.core.trace import TraceRecorder, open_trace


def _emit_sample_trace() -> list[dict[str, object]]:
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    # params keys arrive unsorted; the recorder must emit them sorted (the C++
    # std::map iterates sorted — parsed traces must match across languages).
    rec.planning_started("prioritized_astar", "maps/grid/maze01.yaml", {"zeta": 2.0, "alpha": 1.0})
    rec.node_expanded((3, 4), cost=5.0, agent=0, t=5)
    rec.path_found([[3, 4], [3, 4]], agent=0)
    rec.conflict_found("vertex", (2, 2), 1, (0, 1))
    rec.constraint_added(1, "vertex", (2, 2), 1)
    rec.planning_finished(True, {"expanded_nodes": 3.0, "sum_of_costs": 4.0})
    return [json.loads(line) for line in buf.getvalue().splitlines()]


def test_seq_starts_zero_and_is_monotonic() -> None:
    events = _emit_sample_trace()
    seqs = [e["seq"] for e in events]
    assert seqs == list(range(len(events)))


def test_required_fields_per_event() -> None:
    events = {e["event"]: e for e in _emit_sample_trace()}
    started = events["planning_started"]
    assert started["algorithm"] == "prioritized_astar"
    assert started["map"] == "maps/grid/maze01.yaml"
    # params sorted by key, values keep their types.
    assert list(started["params"].keys()) == ["alpha", "zeta"]  # type: ignore[union-attr]
    expanded = events["node_expanded"]
    assert expanded["state"] == [3, 4]
    assert expanded["cost"] == 5.0
    assert expanded["agent"] == 0 and expanded["t"] == 5
    found = events["path_found"]
    assert found["path"] == [[3, 4], [3, 4]] and found["agent"] == 0
    conflict = events["conflict_found"]
    assert conflict["kind"] == "vertex" and conflict["cell"] == [2, 2]
    assert conflict["t"] == 1 and conflict["agents"] == [0, 1]
    constraint = events["constraint_added"]
    assert constraint["agent"] == 1 and constraint["kind"] == "vertex"
    finished = events["planning_finished"]
    assert finished["success"] is True
    assert finished["metrics"] == {"expanded_nodes": 3.0, "sum_of_costs": 4.0}


def test_edge_conflict_carries_both_cells_and_no_wall_clock() -> None:
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    rec.conflict_found("edge", (2, 2), 3, (1, 0), to=(2, 3))
    event = json.loads(buf.getvalue())
    assert event["kind"] == "edge"
    assert event["cell"] == [2, 2] and event["to"] == [2, 3]
    # An MRMP trace carries no wall-clock time — only seq + discrete step t.
    assert set(event) == {"seq", "event", "kind", "cell", "t", "agents", "to"}


def test_optional_fields_omitted_when_absent() -> None:
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    rec.node_expanded((1, 1))  # no cost / agent / t
    event = json.loads(buf.getvalue())
    assert set(event) == {"seq", "event", "state"}


def test_joint_state_is_flattened_int_list() -> None:
    # Joint-space search expands one node per joint state: every agent's cell
    # flattened to [r0, c0, r1, c1, ...], no agent field (the state carries all).
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    rec.node_expanded([3, 4, 2, 2])
    event = json.loads(buf.getvalue())
    assert event["state"] == [3, 4, 2, 2]


def test_open_trace_writes_and_closes(tmp_path: Path) -> None:
    path = str(tmp_path / "t.jsonl")
    with open_trace(path) as rec:
        rec.planning_finished(False, {"expanded_nodes": 0.0})
    with open(path) as fh:
        line = json.loads(fh.readline())
    assert line == {
        "seq": 0,
        "event": "planning_finished",
        "success": False,
        "metrics": {"expanded_nodes": 0.0},
    }
