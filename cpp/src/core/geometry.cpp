#include "mrmp/core/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace mrmp::core {

double point_segment_distance(const Point& p, const Point& a, const Point& b) {
  // Fixed operation order — identical in Python. The clamp of t and the
  // reconstruction of the closest point are part of the cross-language contract.
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  double wx = p.x - a.x;
  double wy = p.y - a.y;
  double len2 = dx * dx + dy * dy;
  double t;
  if (len2 == 0.0) {
    t = 0.0;
  } else {
    t = (wx * dx + wy * dy) / len2;
    if (t < 0.0) {
      t = 0.0;
    } else if (t > 1.0) {
      t = 1.0;
    }
  }
  double cx = a.x + t * dx;
  double cy = a.y + t * dy;
  double ex = p.x - cx;
  double ey = p.y - cy;
  return std::sqrt(ex * ex + ey * ey);
}

double orient(const Point& a, const Point& b, const Point& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool segments_intersect(const Point& a, const Point& b, const Point& c, const Point& d) {
  auto on_segment = [](const Point& s, const Point& e, const Point& p) {
    return (std::min(s.x, e.x) <= p.x && p.x <= std::max(s.x, e.x)) &&
           (std::min(s.y, e.y) <= p.y && p.y <= std::max(s.y, e.y));
  };
  double o1 = orient(a, b, c);
  double o2 = orient(a, b, d);
  double o3 = orient(c, d, a);
  double o4 = orient(c, d, b);
  if (o1 == 0.0 && on_segment(a, b, c)) return true;
  if (o2 == 0.0 && on_segment(a, b, d)) return true;
  if (o3 == 0.0 && on_segment(c, d, a)) return true;
  if (o4 == 0.0 && on_segment(c, d, b)) return true;
  return (o1 > 0.0) != (o2 > 0.0) && (o3 > 0.0) != (o4 > 0.0);
}

double moving_pair_distance(const Point& a1, const Point& b1, const Point& a2, const Point& b2) {
  double wx = a1.x - a2.x;
  double wy = a1.y - a2.y;
  double dx = (b1.x - a1.x) - (b2.x - a2.x);
  double dy = (b1.y - a1.y) - (b2.y - a2.y);
  return point_segment_distance(Point{0.0, 0.0}, Point{wx, wy}, Point{wx + dx, wy + dy});
}

}  // namespace mrmp::core
