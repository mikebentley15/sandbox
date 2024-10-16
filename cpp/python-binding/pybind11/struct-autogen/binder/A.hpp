#pragma once

#include <ostream>
#include <string>
#include <vector>

namespace A_ns {

/** Represents a 3D point
 *
 * @param x: x-coordinate
 * @param y: y-coordinate
 * @param z: z-coordinate
 */
struct Point {
    double x;
    double y;
    double z;
};

struct A {
    int a;
    std::string b;
    std::vector<Point> points;
};

} // namespace A_ns
