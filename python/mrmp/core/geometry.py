"""Exact floating-point geometry shared by the continuous capability and planners.

Every function here is a fixed IEEE-754 double expression: identical operation
order in Python and C++ means bit-identical results, so every boundary case of
every collision predicate resolves on identical bits across engines (the repo's
parse-equality contract covers the wire; this module makes the DECISIONS equal).

Collision semantics: obstacle cells are CLOSED squares and a robot's disc is
blocked by one iff the distance from its center to the cell is strictly less than
the radius — grazing at exactly the radius stays free, and a point robot (r = 0)
is blocked exactly when it sits on/inside a cell (distance 0). Two robots collide
iff their discs overlap: center distance strictly below the sum of radii.
"""

from __future__ import annotations

import math

from .types import Point


def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    """Distance from point p to segment a->b. Fixed expression order — the clamp
    of t and the reconstruction of the closest point are part of the contract."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    wx = p[0] - a[0]
    wy = p[1] - a[1]
    len2 = dx * dx + dy * dy
    if len2 == 0.0:
        t = 0.0
    else:
        t = (wx * dx + wy * dy) / len2
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
    cx = a[0] + t * dx
    cy = a[1] + t * dy
    ex = p[0] - cx
    ey = p[1] - cy
    return math.sqrt(ex * ex + ey * ey)


def _orient(a: Point, b: Point, c: Point) -> float:
    """Cross product (b-a) x (c-a); its sign is the orientation of a->b->c."""
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: Point, b: Point, p: Point) -> bool:
    """p is collinear with a->b (caller checked orient == 0): is it on the segment?"""
    return (min(a[0], b[0]) <= p[0] <= max(a[0], b[0])) and (
        min(a[1], b[1]) <= p[1] <= max(a[1], b[1])
    )


def segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    """Closed-segment intersection test (touching counts). Exact float sign tests;
    collinear overlap falls through to the on-segment cases."""
    o1 = _orient(a, b, c)
    o2 = _orient(a, b, d)
    o3 = _orient(c, d, a)
    o4 = _orient(c, d, b)
    if o1 == 0.0 and _on_segment(a, b, c):
        return True
    if o2 == 0.0 and _on_segment(a, b, d):
        return True
    if o3 == 0.0 and _on_segment(c, d, a):
        return True
    if o4 == 0.0 and _on_segment(c, d, b):
        return True
    return (o1 > 0.0) != (o2 > 0.0) and (o3 > 0.0) != (o4 > 0.0)


def moving_pair_distance(a1: Point, b1: Point, a2: Point, b2: Point) -> float:
    """Minimum center-to-center distance while robot 1 slides a1->b1 and robot 2
    slides a2->b2 simultaneously over the same unit interval (both at equal speed
    along their segments). The relative motion w + t*d is itself a segment, so
    this is just its distance to the origin — exact, no time discretization."""
    wx = a1[0] - a2[0]
    wy = a1[1] - a2[1]
    dx = (b1[0] - a1[0]) - (b2[0] - a2[0])
    dy = (b1[1] - a1[1]) - (b2[1] - a2[1])
    return point_segment_distance((0.0, 0.0), (wx, wy), (wx + dx, wy + dy))
