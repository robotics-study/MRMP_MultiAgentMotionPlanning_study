"""Decentralized planners live here — one module per algorithm, slugs matching
configs/decentralized/<slug>.yaml. Reading order follows the branch's genealogy:
PIBT (Okumura et al., IJCAI 2019 / Artificial Intelligence 2022) is the priority
line stripped of offline planning entirely — every timestep each agent decides its
own next cell, blocked occupants inherit the claimant's priority and must vacate,
and a failed inheritance backtracks. No plan exists before it happens. winPIBT
(Okumura, Tamura & Défago, IJCAI 2019) generalizes that negotiation along the time
axis: each agent holds a provisional space-time path, secures its steps one by one
in priority order, and pulls parked occupants forward (retroactively, step by step)
before stepping onto their cell; window w = 1 collapses back to PIBT's per-cell game.

Both members are centralized simulations of decentralized behavior — the honest
label for this whole branch until a radio-range demo exists."""

from .pibt import Pibt
from .winpibt import Winpibt

__all__ = ["Pibt", "Winpibt"]
