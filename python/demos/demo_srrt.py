#!/usr/bin/env python3
"""sRRT demo — assembly only (see demos/demo_common.py)."""

from __future__ import annotations

from demo_common import run

from mrmp.sampling import Srrt

if __name__ == "__main__":
    run("srrt", Srrt)
