#pragma once

#include <string>
#include <vector>

#include "mrmp/core/capabilities.hpp"
#include "mrmp/core/planner.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"

namespace mrmp::sampling {

// dRRT — Solovey, Salzman & Halperin 2016 (IJRR 35(5):501–513; arXiv:1305.2889).
// The Python mirror (python/mrmp/sampling/drrt.py) carries the full semantics
// commentary; in one line: the composite roadmap is never built — each robot gets
// its OWN PRM over the continuous free space (start and goal always vertices, n
// rejection samples, k-nearest edges), the composite roadmap is their TENSOR
// product (every joint edge moves EVERY robot), a tree grows over that implicit
// graph via the direction oracle (per-robot argmax-cosine neighbor), and the goal
// tuple is connected by the paper's prioritized local connector. Probabilistically
// complete, no optimality guarantee — dRRT* is that later wave.
//
// Cross-language bit-identity: every float expression (cosine oracle, Euclidean
// distances with a fixed sum order, moving-pair distance, free/segment predicates)
// has one fixed operation order mirrored here line for line; the PRNG is the same
// int64-exact MINSTD Lehmer stream.
class Drrt final : public core::ContinuousMultiAgentPlanner {
 public:
  explicit Drrt(core::ParamSet params) : ContinuousMultiAgentPlanner(std::move(params)) {}

  std::string name() const override { return "drrt"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::CONTINUOUS_SPACE};
  }

  core::ContinuousPlanResult plan(const core::ContinuousSpace& space,
                                  const std::vector<core::ContinuousAgentTask>& tasks,
                                  core::TraceRecorder* recorder) override;
};

}  // namespace mrmp::sampling
