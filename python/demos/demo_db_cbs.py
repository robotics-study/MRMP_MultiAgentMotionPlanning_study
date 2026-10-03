#!/usr/bin/env python3
"""db-CBS demo — assembly only (see demos/demo_common.py, run_velocity)."""

from __future__ import annotations

from demo_common import run_velocity

from mrmp.kinodynamic import DbCbs

if __name__ == "__main__":
    run_velocity("db_cbs", DbCbs)
