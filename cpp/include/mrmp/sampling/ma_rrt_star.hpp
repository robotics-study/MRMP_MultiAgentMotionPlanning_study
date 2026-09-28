#pragma once

#include <string>
#include <vector>

#include "mrmp/core/capabilities.hpp"
#include "mrmp/core/planner.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"

namespace mrmp::sampling {

// MA-RRT* — Čáp, Novák, Vokřínek & Pěchouček 2013 (AAMAS). The Python mirror
// (python/mrmp/sampling/ma_rrt_star.py) carries the full semantics commentary;
// in one line: RRT* run on the JOINT state space of per-agent motion graphs — a
// tree whose vertices are joint states, grown by GREEDY steering (every agent
// greedily steps toward the sampled tuple's own coordinate, aborting on any
// conflict), rewired by NEAR balls, asymptotically optimal in the limit. The
// paper's discretization (unit primitives over the 4-neighborhood plus wait) is
// exactly this repo's DiscreteSpace, so no new capability was needed.
//
// Cross-language determinism: the PRNG is MINSTD Lehmer (s ← 16807·s mod 2³¹−1,
// u = s/(2³¹−1)) — integer-exact in int64 and exact in IEEE doubles; distances
// are sums of correctly-rounded sqrt over integer deltas; the near-ball radius is
// quantized to a 1/64 lattice because log/pow are libm-dependent at ULP level.
class MaRrtStar final : public core::MultiAgentPlanner {
 public:
  explicit MaRrtStar(core::ParamSet params) : MultiAgentPlanner(std::move(params)) {}

  std::string name() const override { return "ma_rrt_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }

  core::MultiPlanResult plan(const core::DiscreteSpace& space,
                             const std::vector<core::AgentTask>& tasks,
                             core::TraceRecorder* recorder) override;
};

}  // namespace mrmp::sampling
