#!/usr/bin/env python3
"""PIBT demo — assembly only (see demos/demo_common.py)."""

from __future__ import annotations

from demo_common import run

from mrmp.decentralized import Pibt

if __name__ == "__main__":
    run("pibt", Pibt)
