#!/usr/bin/env python3
"""MA-RRT* demo — assembly only (see demos/demo_common.py)."""

from __future__ import annotations

from demo_common import run

from mrmp.sampling import MaRrtStar

if __name__ == "__main__":
    run("ma_rrt_star", MaRrtStar)
