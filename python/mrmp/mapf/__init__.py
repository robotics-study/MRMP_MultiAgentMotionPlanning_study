"""MRMP MAPF planners live here — one module per algorithm, slugs matching
configs/mapf/<slug>.yaml. Wave order follows the multi-agent genealogy:
prioritized planning (Erdmann & Lozano-Pérez 1987) -> joint-space search ->
CBS (Sharon et al. 2015)."""

from .prioritized_astar import PrioritizedAStar

__all__ = ["PrioritizedAStar"]
