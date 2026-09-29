#pragma once

#include <string>
#include <set>
#include <vector>

#include "mrmp/core/capabilities.hpp"
#include "mrmp/core/planner.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"

namespace mrmp::sampling {

// dRRT* — Shome, Solovey, Dobson, Halperin & Bekris 2020 (Autonomous Robots
// 44(3-4):443–467; conference version Dobson, Solovey, Shome, Halperin & Bekris,
// IEEE Intl. Symposium on Multi-Robot and Multi-Agent Systems (MRS), 2017). The Python mirror
// (python/mrmp/sampling/drrt_star.py) carries the full semantics commentary; in
// one line: the same tensor-product roadmap as dRRT, but each G_i connects by
// the PRM* radius r(n) = (1+eta)^2 * sqrt(mu(C_f) log n / (2 pi n)) with
// self-loops (waiting is a graph edge), and the tree search becomes RRT*-style:
// cost-to-come rewiring, branch-and-bound against the incumbent, and an oracle
// I_d that switches to argmin-H (shortest-path length on G_i to the goal
// vertex) on goal-biased draws and picks a uniformly random neighbor otherwise.
// Anytime by construction — growth and rewiring keep improving the incumbent.
//
// Cross-language bit-identity: every float expression (Euclidean distance with
// a fixed sum order, edge weights, arc lengths, the radius bound) has one fixed
// operation order mirrored here line for line; the PRNG is the same int64-exact
// MINSTD Lehmer stream; Dijkstra's heap pops (distance, index) lexicographically
// exactly like Python's heapq tuples.
class DrrtStar final : public core::ContinuousMultiAgentPlanner {
 public:
  explicit DrrtStar(core::ParamSet params) : ContinuousMultiAgentPlanner(std::move(params)) {}

  std::string name() const override { return "drrt_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::CONTINUOUS_SPACE};
  }

  core::ContinuousPlanResult plan(const core::ContinuousSpace& space,
                                  const std::vector<core::ContinuousAgentTask>& tasks,
                                  core::TraceRecorder* recorder) override;
};

}  // namespace mrmp::sampling
