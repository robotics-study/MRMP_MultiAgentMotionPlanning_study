#pragma once

#include <string>
#include <vector>

#include "mrmp/core/capabilities.hpp"
#include "mrmp/core/planner.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"

namespace mrmp::sampling {

// sRRT — Wagner, Kang & Choset 2012 (ICRA); framework: Wagner & Choset (AIJ
// 2015). The Python mirror (python/mrmp/sampling/srrt.py) carries the full
// semantics commentary; in one line: subdimensional expansion over an RRT —
// each robot gets an individual policy (a BFS tree grown backward from its goal),
// the joint tree grows forward, robots outside a node's collision set obey their
// policy (one step per expansion), robots inside are steered by samples, and a
// robot-robot collision on a local path enlarges the expanding node's collision
// set and back-propagates that addition up the predecessor chain.
//
// Cross-language determinism: the PRNG is MINSTD Lehmer (s ← 16807·s mod 2³¹−1,
// u = s/(2³¹−1)) — integer-exact in int64 and exact in IEEE doubles; everything
// else here is integer/grid arithmetic, so parity is exact by construction (no
// float appears anywhere in this planner's decisions except the rng draw).
class Srrt final : public core::MultiAgentPlanner {
 public:
  explicit Srrt(core::ParamSet params) : MultiAgentPlanner(std::move(params)) {}

  std::string name() const override { return "srrt"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }

  core::MultiPlanResult plan(const core::DiscreteSpace& space,
                             const std::vector<core::AgentTask>& tasks,
                             core::TraceRecorder* recorder) override;
};

}  // namespace mrmp::sampling
