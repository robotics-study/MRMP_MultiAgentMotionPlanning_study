#!/usr/bin/env python3
"""RHCR demo — assembly only (see demos/demo_common.py)."""

from __future__ import annotations

from demo_common import run

from mrmp.search import Rhcr

if __name__ == "__main__":
    run("rhcr", Rhcr)
