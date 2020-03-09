#ifndef COLLISION_H
#define COLLISION_H

#include <collision/collision_primitives.h>
#include <collision/Point.h>
#include <collision/Sphere.h>
#include <collision/Capsule.h>
#include <collision/CapsuleSequence.h>

namespace collision {


//
// Point
//

inline bool collides(const Point &a, const Point &b) {
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}


//
// Sphere
//

inline bool collides(const Sphere &s, const Point &p) {
  Point diff = s.c - p;
  return diff.dot(diff) <= (s.r * s.r);
}

inline bool collides(const Point &p, const Sphere &s) {
  return collides(s, p);
}

inline bool collides(const Sphere &a, const Sphere &b) {
  return collides(Sphere{a.c, a.r + b.r}, b.c);
}


//
// Capsule
//

inline bool collides(const Capsule &c, const Point &p) {
  auto t = closest_t_segment(c.a, c.b, p);
  auto closest = interpolate(c.a, c.b, t);
  return collides(Sphere{closest, c.r}, p);
}

inline bool collides(const Point &p, const Capsule &c) {
  return collides(c, p);
}

inline bool collides(const Capsule &c, const Sphere &s) {
  // dilated capsule against point
  return collides(Capsule{c.a, c.b, c.r + s.r}, s.c);
}

inline bool collides(const Sphere &s, const Capsule &c) {
  return collides(c, s);
}

inline bool collides(const Capsule &c1, const Capsule &c2) {
  auto [s, t] = closest_st_segment(c1.a, c1.b, c2.a, c2.b);
  auto closest_1 = interpolate(c1.a, c1.b, s);
  auto closest_2 = interpolate(c2.a, c2.b, t);
  const bool val = collides(Sphere{closest_1, c1.r + c2.r}, closest_2);
  return val;
}


//
// CapsuleSequence
//

// piggy back on the collides(Capsule, *) functions
template <typename T>
inline bool collides(const CapsuleSequence &seq, const T &val) {
  for (size_t i = 0; i < seq.size(); i++) {
    if (collides(seq[i], val)) {
      return true;
    }
  }
  return false;
}

inline bool collides(const Point &p, const CapsuleSequence &seq) {
  return collides(seq, p);
}

inline bool collides(const Sphere &s, const CapsuleSequence &seq) {
  return collides(seq, s);
}

inline bool collides(const Capsule &c, const CapsuleSequence &seq) {
  return collides(seq, c);
}

} // end of namespace collision

#endif // COLLISION_H
