"""Kinodynamic-branch planners live here — one module per algorithm, slugs
matching configs/kinodynamic/<slug>.yaml. This branch plans NOTHING new: a
collision-free discrete plan already exists (the search branch's CBS solves the
discrete problem first); what these planners add is TIME. They convert the cell
sequence into a Temporal Plan Graph, solve the Simple Temporal Network it becomes
and hand back the earliest feasible plan-execution schedule — per-agent routes
plus one arrival time per retained location — so point robots executing at their
own velocity limits provably keep a safety distance. The genealogy line: search
gives the plan, kinodynamics gives it a clock."""

from .mapf_post import MapfPost

__all__ = ["MapfPost"]
