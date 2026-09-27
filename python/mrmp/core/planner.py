"""Abstract planner base — the common surface every algorithm implements.

Mirrors the C++ `core/planner.hpp` (`MultiAgentPlanner`). No type parameters:
every MRMP planner searches the same discrete grid over Cell states, so a state/
space generic pair would sit unused on every implementation.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from .capabilities import Capability, DiscreteSpace
from .params import ParamSet
from .trace import TraceRecorder
from .types import AgentTask, MultiPlanResult


class MultiAgentPlanner(ABC):
    """One plan() call solves the whole task list: it returns one space-time path
    per agent (or fails), and emits its whole reasoning as trace events. Unlike a
    single-robot planner there is no start/goal pair — the tasks carry every
    agent's endpoints, in agent-index order."""

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
