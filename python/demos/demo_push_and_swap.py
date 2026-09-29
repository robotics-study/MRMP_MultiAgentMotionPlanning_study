#!/usr/bin/env python3
"""Push and Swap demo — assembly only (see demos/demo_common.py)."""

from __future__ import annotations

from demo_common import run

from mrmp.search import PushAndSwap

if __name__ == "__main__":
    run("push_and_swap", PushAndSwap)
