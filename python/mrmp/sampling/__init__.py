"""Sampling-based planners live here — one module per algorithm, slugs matching
configs/<slug>.yaml. Reading order follows the branch's genealogy: MA-RRT* (Čáp
et al. 2013) is the coupled pole of this branch — RRT* on the joint state space
of motion graphs — and comes first; subdimensional expansion and implicit
roadmaps follow."""

from .ma_rrt_star import MaRrtStar

__all__ = ["MaRrtStar"]
