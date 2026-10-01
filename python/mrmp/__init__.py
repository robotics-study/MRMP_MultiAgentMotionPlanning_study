"""mrmp — multi-agent motion planning algorithms (Python mirror of C++).

Algorithm packages are named after the site's sections — the branches of the MRMP
genealogy: ``search`` (enumeration over the joint state space), ``sampling``
(randomized growth over continuous configuration space), ``decentralized`` (no
offline plan at all — per-timestep online negotiation) and ``kinodynamic`` (plan
post-processing — the discrete plan already exists; this branch gives it a clock).
The kinodynamic package is the one cross-branch edge: its planner reuses the
search branch's CBS as the silent underlying solver.
"""

__version__ = "0.1.0"
