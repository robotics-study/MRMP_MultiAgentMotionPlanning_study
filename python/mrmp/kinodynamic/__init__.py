"""Kinodynamic-branch planners live here — one module per algorithm, slugs matching
configs/kinodynamic/<slug>.yaml. This branch is where motion stops being one cell per
step and becomes velocity with a limit. MAPF-POST plans NOTHING new: the search
branch's CBS solves the discrete problem first; it converts that plan into a Temporal
Plan Graph, solves the Simple Temporal Network it becomes and hands back the earliest
feasible plan-execution schedule — per-agent routes plus one arrival time per retained
location, so point robots executing at their own velocity limits provably keep a safety
distance. db-CBS plans the motion ITSELF: CBS whose low level searches states
(cell, velocity) on the discretized double integrator, chaining primitives under a
discontinuity bound that tightens rung by rung — momentum enters as state, not as a
clock laid over someone else's plan. The genealogy line: search gives the plan,
kinodynamics gives it a clock — and then hands the clock back to the search."""

from .db_cbs import DbCbs
from .mapf_post import MapfPost

__all__ = ["DbCbs", "MapfPost"]
