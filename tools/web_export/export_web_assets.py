#!/usr/bin/env python3
"""Export web-site data assets: grid maps as JSON + demo traces as gzip JSONL.

The docs SPA (document/) replays real demo traces. The C++ and Python demos emit
byte-identical event streams, so web assets are generated from the Python demo
alone and stored gzipped for static serving (gzip mtime is pinned to 0 so the
same input always produces the same bytes — stable git diffs).

Usage:
    python tools/web_export/export_web_assets.py --algos prioritized_astar \
        --maps maze01 --scenario maze01_two
"""

from __future__ import annotations

import argparse
import gzip
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
DATA_DIR = REPO / "document" / "public" / "data"
# Web replay cap — traces larger than this are never committed (unplayable size).
MAX_EVENTS = 200_000


def read_pgm(path: Path) -> tuple[int, int, list[int]]:
    """Minimal PGM reader (P2/P5)."""
    data = path.read_bytes()
    if data[:2] == b"P2":
        tokens = []
        for line in data.decode("ascii").splitlines():
            line = line.split("#", 1)[0]
            tokens.extend(line.split())
        width, height, _maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
        pixels = [int(t) for t in tokens[4:4 + width * height]]
        return width, height, pixels
    if data[:2] == b"P5":
        # Read the three header integers (width height maxval) whitespace-separated;
        # everything after the single terminating whitespace byte is raster.
        idx = 2
        values: list[int] = []
        while len(values) < 3:
            while idx < len(data) and data[idx:idx + 1].isspace():
                idx += 1
            if data[idx:idx + 1] == b"#":
                while data[idx:idx + 1] != b"\n":
                    idx += 1
                continue
            start = idx
            while idx < len(data) and not data[idx:idx + 1].isspace():
                idx += 1
            values.append(int(data[start:idx]))
        idx += 1  # single whitespace byte terminating the header
        width, height, _maxval = values
        return width, height, list(data[idx:idx + width * height])
    raise ValueError(f"unsupported PGM format: {path}")


def export_map(name: str) -> None:
    map_yaml = REPO / "maps" / "grid" / f"{name}.yaml"
    meta = yaml.safe_load(map_yaml.read_text())
    image = map_yaml.parent / meta["image"]
    width, height, pixels = read_pgm(image)
    # ROS style: 0 (black) = occupied, 255 (white) = free. Split at the midpoint.
    rows = [
        "".join("#" if pixels[r * width + c] < 128 else "." for c in range(width))
        for r in range(height)
    ]
    out = DATA_DIR / "maps" / f"{name}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    out.write_text(
        '{\n'
        f'  "name": "{name}",\n'
        f'  "width": {width},\n'
        f'  "height": {height},\n'
        f'  "resolution": {meta["resolution"]},\n'
        f'  "origin": [{origin[0]}, {origin[1]}],\n'
        '  "rows": [\n    '
        + ",\n    ".join(f'"{row}"' for row in rows)
        + "\n  ]\n}\n"
    )
    print(f"map: {out.relative_to(REPO)} ({width}x{height})")


def params_file(algo: str, overrides: dict[str, str], tmp: Path) -> Path:
    """Config yaml path — with overrides, a copy whose defaults are replaced.

    Demo default budgets can be too large for web replay files, so web assets are
    re-run with smaller budgets; parameters are recorded in the trace's
    planning_started event, so replay and parity stay exact either way.
    """
    src = REPO / "configs" / "mapf" / f"{algo}.yaml"
    if not overrides:
        return src
    doc = yaml.safe_load(src.read_text())
    for p in doc["params"]:
        if p["name"] in overrides:
            raw = overrides[p["name"]]
            p["default"] = int(raw) if p["type"] == "int" else float(raw)
    out = tmp / f"{algo}.yaml"
    out.write_text(yaml.safe_dump(doc, sort_keys=False, allow_unicode=True))
    return out


def run_demo(algo: str, map_name: str, scenario: Path, trace_path: Path,
             params_path: Path) -> None:
    cmd = [
        sys.executable, str(REPO / "python" / "demos" / f"demo_{algo}.py"),
        "--map", str(REPO / "maps" / "grid" / f"{map_name}.yaml"),
        "--scenario", str(scenario),
        "--params", str(params_path),
        "--trace", str(trace_path),
    ]
    subprocess.run(cmd, check=True, cwd=REPO, env={**os.environ, "PYTHONPATH": str(REPO / "python")},
                   stdout=subprocess.DEVNULL)


def export_traces(algo: str, map_name: str, scenario: Path,
                  overrides: dict[str, str]) -> None:
    with tempfile.TemporaryDirectory() as tmp:
        trace = Path(tmp) / "trace.jsonl"
        params_path = params_file(algo, overrides, Path(tmp))
        run_demo(algo, map_name, scenario, trace, params_path)
        events = sum(1 for _ in trace.open())
        if events > MAX_EVENTS:
            raise SystemExit(
                f"{algo}/{map_name}/py: {events} events > {MAX_EVENTS} — "
                "reduce the demo budget before exporting for the web"
            )
        out = DATA_DIR / "traces" / algo / f"{map_name}.py.jsonl.gz"
        out.parent.mkdir(parents=True, exist_ok=True)
        # mtime=0 keeps identical inputs byte-identical (stable git diffs).
        with trace.open("rb") as src, gzip.GzipFile(out, "wb", mtime=0) as dst:
            shutil.copyfileobj(src, dst)
        print(f"trace: {out.relative_to(REPO)} ({events} events)")


def main() -> None:
    parser = argparse.ArgumentParser(description="export web data assets for document/")
    parser.add_argument(
        "--algos", default="", help="comma-separated algorithm slugs (empty: maps only)"
    )
    parser.add_argument("--maps", required=True, help="comma-separated grid map names")
    parser.add_argument("--scenario", required=True,
                        help="scenario name under maps/scenarios/ (no .yaml suffix)")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                        help="param default override for the demo run (repeatable)")
    args = parser.parse_args()
    algos = [a for a in args.algos.split(",") if a]
    overrides = dict(kv.split("=", 1) for kv in args.set)
    scenario = REPO / "maps" / "scenarios" / f"{args.scenario}.yaml"
    for map_name in args.maps.split(","):
        export_map(map_name)
        for algo in algos:
            export_traces(algo, map_name, scenario, overrides)


if __name__ == "__main__":
    main()
