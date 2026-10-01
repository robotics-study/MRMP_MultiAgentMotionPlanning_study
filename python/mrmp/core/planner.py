"""Abstract planner bases — the common surface every algorithm implements.

Mirrors the C++ `core/planner.hpp` (`MultiAgentPlanner`, `ContinuousMultiAgent
Planner`, `KinodynamicPlanner`). No type parameters: the search branch searches the
discrete grid over Cell states, the sampling branch's continuous planners (dRRT
family) plan on the same map's continuous free space over Point states, and the
kinodynamic branch takes discrete tasks plus per-agent velocity limits and returns a
timed schedule — one base per state kind so no generic pair sits unused on every
implementation. One plan() call solves the whole task list: it returns one space-time
path (or timed route) per agent (or fails), and emits its whole reasoning as trace
events. Unlike a single-robot planner there is no start/goal pair — the tasks carry
every agent's endpoints, in agent-index order.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from .capabilities import Capability, ContinuousSpace, DiscreteSpace
from .params import ParamSet
from .trace import TraceRecorder
from .types import (
    AgentTask,
    ContinuousPlanResult,
    ContinuousTask,
    MultiPlanResult,
    TimedPlanResult,
)


class MultiAgentPlanner(ABC):
    """Base for the discrete (grid) planners."""

    def __init__(self, params: ParamSet) -> None:
        self.params = params

    @property
    @abstractmethod
    def name(self) -> str:
        """Algorithm id; matches the config filename and trace `algorithm` field."""
        ...

    @abstractmethod
    def required_capabilities(self) -> set[Capability]: ...

    @abstractmethod
    def plan(
        self,
        space: DiscreteSpace,
        tasks: list[AgentTask],
        recorder: TraceRecorder | None = None,
    ) -> MultiPlanResult: ...


class ContinuousMultiAgentPlanner(ABC):
    """Base for the continuous-space planners (the dRRT family). Same lifecycle as
    MultiAgentPlanner; states are world points and tasks carry each disc's radius."""

    def __init__(self, params: ParamSet) -> None:
        self.params = params

    @property
    @abstractmethod
    def name(self) -> str:
        """Algorithm id; matches the config filename and trace `algorithm` field."""
        ...

    @abstractmethod
    def required_capabilities(self) -> set[Capability]: ...

    @abstractmethod
    def plan(
        self,
        space: ContinuousSpace,
        tasks: list[ContinuousTask],
        recorder: TraceRecorder | None = None,
    ) -> ContinuousPlanResult: ...


class KinodynamicPlanner(ABC):
    """Base for the kinodynamic branch (MAPF-POST). The input stays discrete — grid
    cells plus each agent's velocity limit `vmax` (cells per time unit) — and the
    output is a timed schedule (TimedPlanResult): routes with wait actions removed
    plus every retained location's earliest arrival time. Like MultiAgentPlanner the
    space is DiscreteSpace; what differs is the result kind."""

    def __init__(self, params: ParamSet) -> None:
        self.params = params

    @property
    @abstractmethod
    def name(self) -> str:
        """Algorithm id; matches the config filename and trace `algorithm` field."""
        ...

    @abstractmethod
    def required_capabilities(self) -> set[Capability]: ...

    @abstractmethod
    def plan(
        self,
        space: DiscreteSpace,
        tasks: list[AgentTask],
        recorder: TraceRecorder | None = None,
    ) -> TimedPlanResult: ...
