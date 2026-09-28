#!/usr/bin/env python3
"""Prioritized A* demo — assembly only (see demos/demo_common.py)."""

from __future__ import annotations

from demo_common import run

from mrmp.search import PrioritizedAStar

if __name__ == "__main__":
    run("prioritized_astar", PrioritizedAStar)
