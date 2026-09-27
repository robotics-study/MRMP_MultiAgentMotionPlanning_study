"""Core abstractions: state types, capabilities, params, trace, planner base.

Depends only on stdlib + numpy. Knows nothing about concrete maps or algorithms.
"""

from .capabilities import Capability, DiscreteSpace, MapBase
from .params import ParamDecl, ParamError, ParamSet, ParamValue
from .planner import MultiAgentPlanner
from .trace import TraceRecorder, open_trace
from .types import AgentTask, Cell, MultiPlanResult, PlanStats, Point

__all__ = [
    "Capability",
    "DiscreteSpace",
    "MapBase",
    "ParamDecl",
    "ParamError",
    "ParamSet",
    "ParamValue",
    "MultiAgentPlanner",
    "TraceRecorder",
    "open_trace",
    "AgentTask",
    "Cell",
    "Point",
    "MultiPlanResult",
    "PlanStats",
]
