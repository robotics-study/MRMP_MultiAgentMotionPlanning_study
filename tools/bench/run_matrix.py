#!/usr/bin/env python3
"""Benchmark matrix runner: (algorithm x its declared scenarios) -> metrics -> report.

Runs each Python demo as a subprocess and collects metrics from the trace's
`planning_finished` event plus makespan from its `path_found` events. Depends on
spec/core/maps only — it never imports an algorithm module; an algorithm is
runnable exactly when `configs/<section>/<algo>.yaml` and `python/demos/demo_<algo>.py`
both exist, which is how the matrix discovers its columns. Each config declares the
scenario slugs IT runs on (`scenarios:`), so a continuous-planner config never runs
on cell-only scenarios and vice versa — the matrix iterates algorithms and routes
each one onto its own scenario list.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from mrmp.maps.loader import load_scenario

_REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass
class Row:
    scenario: str
    algorithm: str
    status: str  # "ok" | "no_path" | "error"
    metrics: dict[str, float]


def _final_metrics(trace_path: Path) -> dict[str, float] | None:
    """Metrics from the last planning_finished event; makespan is derived from the
    per-agent path_found events (longest space-time path minus its start step)."""
    result: dict[str, float] | None = None
    success = False
    makespan = 0.0
    with open(trace_path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            ev: dict[str, Any] = json.loads(line)
            if ev.get("event") == "path_found":
                makespan = max(makespan, float(len(ev["path"]) - 1))
            elif ev.get("event") == "planning_finished":
                result = dict(ev.get("metrics", {}))
                success = bool(ev.get("success"))
    if result is not None:
        result["success"] = 1.0 if success else 0.0
        result.setdefault("makespan", makespan)
    return result


def _run_one(py: str, demos_dir: Path, config: Path, scenario_path: Path, algo: str) -> Row:
    # Loading the scenario through the real loader validates its shape before any
    # demo subprocess is spawned (the map path it resolves feeds --map).
    scenario = load_scenario(scenario_path)
    demo = demos_dir / f"demo_{algo}.py"
    if not demo.exists():
        return Row(scenario_path.stem, algo, "error", {})
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as tmp:
        trace_path = Path(tmp.name)
    cmd = [
        py, str(demo),
        "--map", scenario.map_path,
        "--scenario", str(scenario_path),
        "--params", str(config),
        "--trace", str(trace_path),
    ]
    try:
        # check=False on purpose: a failing demo is a matrix row ("error"), not
        # an exception in the runner.
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if proc.returncode != 0:
            return Row(scenario_path.stem, algo, "error", {})
        metrics = _final_metrics(trace_path)
    finally:
        trace_path.unlink(missing_ok=True)
    if metrics is None:
        return Row(scenario_path.stem, algo, "error", {})
    status = "ok" if metrics.get("success", 0.0) >= 1.0 else "no_path"
    return Row(scenario_path.stem, algo, status, metrics)


def _render(rows: list[Row]) -> str:
    header = (
        "| algorithm | scenario | status | sum_of_costs | makespan | expanded |\n"
        "|---|---|---|---|---|---|\n"
    )
    lines = [header]
    for r in rows:
        m = r.metrics
        if r.status == "error":
            lines.append(f"| {r.algorithm} | {r.scenario} | {r.status} | - | - | - |\n")
            continue
        lines.append(
            f"| {r.algorithm} | {r.scenario} | {r.status} | "
            f"{m.get('sum_of_costs', 0.0):.1f} | {int(m.get('makespan', 0))} | "
            f"{int(m.get('expanded_nodes', 0))} |\n"
        )
    return "".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="MRMP benchmark matrix runner")
    parser.add_argument("--configs", default=str(_REPO_ROOT / "configs"),
                        help="algorithm-config root (algo id -> <section-dir>/<algo>.yaml under it)")
    parser.add_argument("--scenarios", default=str(_REPO_ROOT / "maps" / "scenarios"),
                        help="scenario directory; each config's `scenarios:` list selects from it")
    parser.add_argument("--demos", default=str(_REPO_ROOT / "python" / "demos"))
    parser.add_argument("--out", default=str(_REPO_ROOT / "out" / "report.md"))
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument(
        "--algos", nargs="*", default=None,
        help="algorithm ids to run (default: every config under --configs, sorted)",
    )
    args = parser.parse_args()

    configs_dir = Path(args.configs)
    scenarios_dir = Path(args.scenarios)
    demos_dir = Path(args.demos)
    algos = args.algos or sorted(p.stem for p in configs_dir.rglob("*.yaml"))

    rows: list[Row] = []
    for algo in algos:
        # Slugs are globally unique across sections — two matches is a repo bug,
        # not a tie to break silently. A missing config still runs the demo
        # subprocess with a nonexistent --params path so an explicit --algos
        # request fails loudly ("error"), never silently skipped.
        found = sorted(configs_dir.rglob(f"{algo}.yaml"))
        if len(found) > 1:
            raise SystemExit(
                f"expected exactly one configs/<section>/{algo}.yaml, found {len(found)}"
            )
        config_path = found[0] if found else configs_dir / f"{algo}.yaml"
        # Route the algorithm onto ITS declared scenarios (raw read — the runner
        # stays free of mrmp.core; the demo subprocess validates the config itself).
        try:
            declared = yaml.safe_load(config_path.read_text(encoding="utf-8")).get("scenarios", [])
        except FileNotFoundError:
            declared = []
        for scenario_name in declared:
            rows.append(_run_one(args.python, demos_dir, config_path,
                                 scenarios_dir / f"{scenario_name}.yaml", algo))
            print(f"ran {algo} x {scenario_name} -> {rows[-1].status}", file=sys.stderr)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("# MRMP benchmark matrix\n\n" + _render(rows), encoding="utf-8")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
