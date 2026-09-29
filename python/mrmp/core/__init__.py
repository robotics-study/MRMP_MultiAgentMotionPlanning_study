"""Core abstractions: state types, capabilities, geometry, params, trace, planner base.

Depends only on stdlib + numpy. Knows nothing about concrete maps or algorithms.
"""

from .capabilities import Capability, ContinuousSpace, DiscreteSpace, MapBase
from .geometry import moving_pair_distance, point_segment_distance, segments_intersect
from .params import ParamDecl, ParamError, ParamSet, ParamValue
from .planner import ContinuousMultiAgentPlanner, MultiAgentPlanner
from .trace import TraceRecorder, open_trace
from .types import (
    AgentTask,
    Cell,
    ContinuousPlanResult,
    ContinuousTask,
    MultiPlanResult,
    PlanStats,
    Point,
)

__all__ = [
    "Capability",
    "ContinuousSpace",
    "DiscreteSpace",
    "MapBase",
    "moving_pair_distance",
    "point_segment_distance",
    "segments_intersect",
    "ParamDecl",
    "ParamError",
    "ParamSet",
    "ParamValue",
    "ContinuousMultiAgentPlanner",
    "MultiAgentPlanner",
    "TraceRecorder",
    "open_trace",
    "AgentTask",
    "Cell",
    "ContinuousPlanResult",
    "ContinuousTask",
    "MultiPlanResult",
    "PlanStats",
    "Point",
]
