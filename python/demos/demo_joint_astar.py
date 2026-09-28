#!/usr/bin/env python3
"""Joint-space A* demo — assembly only (see demos/demo_common.py)."""

from __future__ import annotations

from demo_common import run

from mrmp.mapf import JointAStar

if __name__ == "__main__":
    run("joint_astar", JointAStar)
