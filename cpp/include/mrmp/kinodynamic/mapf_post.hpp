#pragma once

#include <set>
#include <string>
#include <vector>

#include "mrmp/core/capabilities.hpp"
#include "mrmp/core/planner.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"

namespace mrmp::kinodynamic {

// MAPF-POST — plan post-processing, the kinodynamic branch's (only) algorithm.
//
// Hönig, Kumar, Cohen, Ma, Xu, Ayanian & Koenig (ICAPS 2016), "Multi-Agent Path
// Finding with Kinematic Constraints": MAPF solvers plan on a graph in discrete steps;
// real robots move continuously at finite velocity. MAPF-POST plans NOTHING new — it
// takes the search branch's collision-free discrete plan (CBS runs here, silently, as
// the underlying solver) and converts it into a plan-execution schedule: per-agent routes
// with wait actions removed plus one earliest arrival time per retained location, so each
// agent dwells on a cell until its departure and traverses at exactly its own limit.
//
// The conversion (the paper's Algorithm 1, implemented literally):
//
// * TPG. A vertex is an event — "agent j enters location s at discrete step t" (waits
//   removed: the first occurrence of each retained location keeps its step). Type-1 edges
//   chain each agent's own events in route order. Type-2 edges encode the plan's
//   collision-freeness as precedence: for every location two agents both enter, an edge
//   runs from the EARLIER visitor's event to the LATER one (the scan stops at the first
//   later visit — later ones are implied by transitivity). Both directions of a same-edge
//   swap can never appear in a collision-free plan (t > s and t <= s+1 force t = s+1,
//   exactly an edge conflict), so the earlier visitor always has an outgoing move edge and
//   the later one an incoming one.
// * Augmentation. Every Type-1 edge e (length l(e) = 1 here, always) splits into v -> m1
//   (length delta), m1 -> m2 (l - 2*delta), m2 -> v' (delta); each Type-2 edge is rewired
//   from the earlier visitor's safety marker m1 to the later visitor's marker m2. Delta
//   must keep every edge longer than 2*delta: 0 < delta <= 0.5 = l/2; with OPEN protected
//   clouds of radius delta around cells, 2*delta <= l keeps every pair disjoint.
// * STN. Every augmented edge becomes a simple temporal constraint [LB, inf] with
//   LB = its length / v_k and Type-2 edges get [0, inf]; each agent's first event is
//   pinned to t = 0 ([0, 0] from X_S). The graph stays acyclic (Type-2 edges always point
//   forward in discrete step), so a negative-cost cycle is impossible — the STN is always
//   consistent (paper Theorem 1) and the EARLIEST schedule exists. This repo computes it
//   as max-relaxation Bellman-Ford on the constraint edges directly, which yields exactly
//   the paper's t(v) = -dist(v, X_S); expanded_nodes counts every relaxation that strictly
//   improved a label (this algorithm's own work metric — never the base search's).
//
// Execution semantics (the uniform velocity model): an agent dwells at cell c_i until its
// departure D_i = T(next) - l/v_k and then traverses at exactly v_k, arriving exactly on
// the scheduled time. Safety (paper Theorem 2): point agents executing a consistent
// schedule keep graph distance >= 2*delta*v_min/v_max > 0.
//
// The paper's LP variants (minimize flow time / maximize v_min) are NOT implemented — this
// is the earliest schedule of Algorithm 1; sum_of_costs here is the resulting flow-time
// analogue, not an optimized objective. The non-holonomic extension (orientation vertices,
// rotate actions) is out of scope: this repo's robots stay point agents on a grid.
//
// Determinism contract: vertex numbering and edge-list order are fixed by construction
// order (agents in index order, segments ascending; Type-2 scans agent index then step
// then other-agent index), and delta plus every demo vmax value are dyadic rationals, so
// all times are exactly representable doubles and the strict-improvement counts and arrival
// times are bit-identical across languages. Mirrors python/mrmp/kinodynamic/mapf_post.py.
class MapfPost final : public core::KinodynamicPlanner {
 public:
  explicit MapfPost(core::ParamSet params) : KinodynamicPlanner(std::move(params)) {}

  std::string name() const override { return "mapf_post"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }

  core::TimedPlanResult plan(const core::DiscreteSpace& space,
                             const std::vector<core::AgentTask>& tasks,
                             core::TraceRecorder* recorder) override;

 private:
  // One STN constraint: the tail's time plus LB never exceeds the head's.
  struct Edge {
    int src = 0;
    int dst = 0;
    double lb = 0.0;
  };

  // The augmented TPG as a flat edge list plus the initial labels (every vertex -inf,
  // every agent's first event pinned to 0). Vertex ids ascend agent by agent (each
  // agent's events in step order), then markers take the remaining ids in chain order.
  static void build_stn(const std::vector<std::vector<core::Cell>>& paths,
                        const std::vector<std::vector<core::Cell>>& routes,
                        const std::vector<std::vector<int>>& steps,
                        const std::vector<core::AgentTask>& tasks, double delta,
                        std::vector<double>& labels, std::vector<Edge>& edges,
                        std::vector<std::vector<int>>& event_ids);

  // Max-relaxation Bellman-Ford on the constraint edges — the DAG's unique fixed point.
  // Every pass reads every edge in construction order; a pass that improves nothing stops
  // the loop. Returns the count of relaxations that strictly improved a label.
  static int relax(std::vector<double>& labels, const std::vector<Edge>& edges);
};

}  // namespace mrmp::kinodynamic
