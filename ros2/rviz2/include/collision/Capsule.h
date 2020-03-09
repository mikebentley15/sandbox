#ifndef CAPSULE_H
#define CAPSULE_H

#include <collision/Point.h>
#include <collision/collision_primitives.h> // for interpolate() and closest_t()

#include <iostream>

namespace collision {

struct Capsule {
  Point a;
  Point b;
  double r;

  Point interpolate(double t) const { return collision::interpolate(a, b, t); }
  double closest_t(const Point &p) const { return collision::closest_t(a, b, p); }
};

std::ostream& operator<<(std::ostream &out, const Capsule &c) {
  return out << "Capsule(" << c.a << ", " << c.b << ", " << "r=" << c.r << ")";
}

} // end of namespace collision

#endif // CAPSULE_H
