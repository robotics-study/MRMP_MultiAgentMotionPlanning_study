"""Decentralized planners live here — one module per algorithm, slugs matching
configs/decentralized/<slug>.yaml. Reading order follows the branch's genealogy:
PIBT (Okumura et al., IJCAI 2019 / Artificial Intelligence 2022) is the priority
line stripped of offline planning entirely — every timestep each agent decides its
own next cell, blocked occupants inherit the claimant's priority and must vacate,
and a failed inheritance backtracks. No plan exists before it happens."""

from .pibt import Pibt

__all__ = ["Pibt"]
