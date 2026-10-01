#!/usr/bin/env python3
"""winPIBT demo — assembly only (see demos/demo_common.py)."""

from __future__ import annotations

from demo_common import run

from mrmp.decentralized import Winpibt

if __name__ == "__main__":
    run("winpibt", Winpibt)
