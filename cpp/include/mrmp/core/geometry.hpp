#pragma once

#include "mrmp/core/types.hpp"

// Exact floating-point geometry shared by the continuous capability and planners.
// Every function here is a fixed IEEE-754 double expression: identical operation
// order in Python and C++ means bit-identical results, so every collision boundary
// case resolves on identical bits across engines (the repo's parse-equality
// contract covers the wire; this module makes the DECISIONS equal). -ffp-contract=
// off keeps FMA from fusing multiply-add into one rounding, matching Python.
//
// Collision semantics: obstacle cells are CLOSED squares and a robot's disc is
// blocked by one iff the distance from its center to the cell is strictly less
// than the radius — grazing at exactly the radius stays free, and a point robot
// (r = 0) is blocked exactly when it sits on/inside a cell (distance 0). Two
// robots collide iff their discs overlap: center distance strictly below the sum.
namespace mrmp::core {

// Distance from point p to segment a->b. Fixed expression order — the clamp of t
// and the reconstruction of the closest point are part of the contract.
double point_segment_distance(const Point& p, const Point& a, const Point& b);

// Cross product (b-a) x (c-a); its sign is the orientation of a->b->c.
double orient(const Point& a, const Point& b, const Point& c);

// True iff closed segment a->b intersects closed segment c->d (touching counts;
// collinear overlap falls through to the on-segment cases).
bool segments_intersect(const Point& a, const Point& b, const Point& c, const Point& d);

// Minimum center-to-center distance while robot 1 slides a1->b1 and robot 2 slides
// a2->b2 simultaneously over the same unit interval (both at equal speed along
// their segments). The relative motion w + t*d is itself a segment, so this is
// just its distance to the origin — exact, no time discretization.
double moving_pair_distance(const Point& a1, const Point& b1, const Point& a2, const Point& b2);

}  // namespace mrmp::core
