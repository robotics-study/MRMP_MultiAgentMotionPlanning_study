#!/usr/bin/env python3
"""dRRT demo — assembly only (see demos/demo_common.py). Continuous planner: the
scenario's world-coord Points stay raw and each disc's radius rides along; the
trace declares coords="world" plus the radii."""

from __future__ import annotations

from demo_common import run_continuous

from mrmp.sampling import Drrt

if __name__ == "__main__":
    run_continuous("drrt", Drrt)
