#ifndef SPHERE_H
#define SPHERE_H

#include <collision/Point.h>

#include <iostream>

namespace collision {

struct Sphere {
  Point c;
  double r;
};

inline std::ostream& operator<<(std::ostream &out, const Sphere &s) {
  return out << "Sphere(" << s.c << ", r=" << s.r << ")";
}

} // end of namespace collision

#endif // SPHERE_H
