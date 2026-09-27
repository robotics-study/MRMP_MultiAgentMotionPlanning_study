#pragma once

#include <set>
#include <string>
#include <vector>

#include "mrmp/core/capabilities.hpp"
#include "mrmp/core/params.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"

namespace mrmp::core {

// Base for all planners. No template parameters: every MRMP planner searches the
// same discrete grid over Cell states, so a state/space generic pair would sit
// unused on every implementation. One plan() call solves the whole task list: it
// returns one space-time path per agent (or fails), and emits its whole reasoning
// as trace events. Unlike a single-robot planner there is no start/goal pair —
// the tasks carry every agent's endpoints, in agent-index order.
class MultiAgentPlanner {
 public:
  explicit MultiAgentPlanner(ParamSet params) : params_(std::move(params)) {}
  virtual ~MultiAgentPlanner() = default;

  virtual std::string name() const = 0;
  virtual std::set<Capability> required_capabilities() const = 0;
  virtual MultiPlanResult plan(const DiscreteSpace& space, const std::vector<AgentTask>& tasks,
                               TraceRecorder* recorder) = 0;

 protected:
  ParamSet params_;
};

}  // namespace mrmp::core
