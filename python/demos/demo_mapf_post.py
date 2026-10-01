#!/usr/bin/env python3
"""MAPF-POST demo — assembly only (see demos/demo_common.py, run_timed)."""

from __future__ import annotations

from demo_common import run_timed

from mrmp.kinodynamic import MapfPost

if __name__ == "__main__":
    run_timed("mapf_post", MapfPost)
