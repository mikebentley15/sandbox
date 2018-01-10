#include <algorithm>
#include <ios>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <unordered_map>

#include <cstring>  // for memcpy
#include <cmath>    // for sqrt and atan2

using stringlist  = std::vector<std::string>;
using intlist     = std::vector<int>;

//const bool DEBUG = true;
const bool DEBUG = false;
template <typename T>
void debug(const T &item) {
  if (DEBUG) {
    std::cout << item << std::endl;
  }
}

#define my_assert(x) \
  if (!x) { \
    printf("Assertion failed!, line %d, %s", __LINE__, #x); \
  }

stringlist read_input() {
  stringlist inlist;
  while (!std::cin.eof()) {
    std::string line;
    std::getline(std::cin, line);
    if (line.size() > 0) { // ignore empty lines
      inlist.emplace_back(line);
    }
  }
  return inlist;
}

template <typename T>
void print_lines(const std::vector<T> outlist, bool print = true) {
  if (print) {
    for (auto val : outlist) {
      std::cout << val << std::endl;
    }
  }
}

template <typename T>
std::vector<T> str_split(const std::string &instr) {
  if (instr == "") {
    return {};
  }
  std::vector<T> vals;
  std::istringstream streamer;
  streamer.str(instr);
  while(!streamer.eof()) {
    T val;
    streamer >> val;
    vals.emplace_back(val);
  }
  return vals;
}

struct Vertex {
  double x;
  double y;
  Vertex(double _x = 0.0, double _y = 0.0) : x(_x), y(_y) {}
};
using vertexlist  = std::vector<Vertex>;
using Polygon     = vertexlist;
using PolygonList = std::vector<Polygon>;

PolygonList read_polygons(const stringlist &inlist) {
  PolygonList polylist;
  int idx = 0;
  while (idx < inlist.size()) {
    auto size = std::stoi(inlist[idx++]);
    Polygon poly;
    for (int i = idx; idx < i + size; idx++) {
      auto xy = str_split<double>(inlist[idx]);
      poly.emplace_back(xy[0], xy[1]);
    }
    polylist.emplace_back(std::move(poly));
  }
  return polylist;
}

std::ostream& operator<<(std::ostream& out, const Vertex &v) {
  out << "(" << v.x << ", " << v.y << ")";
  return out;
}

std::ostream& operator<<(std::ostream& out, const Polygon &p) {
  out << "Polygon{";
  for (auto v : p) {
    out << v << ", ";
  }
  out << "}";
  return out;
}

Polygon& sort_vertices(Polygon &poly) {
  if (poly.size() <= 2) {
    return poly;
  }

  Vertex center {0, 0};
  for (auto &v : poly) {
    center.x += v.x;
    center.y += v.y;
  }
  center.x /= poly.size();
  center.y /= poly.size();

  auto comp_angle = [&center](const Vertex &v) {
    return std::atan2(v.y - center.y, v.x - center.x);
  };

  // Sort counter-clockwise
  std::sort(poly.begin(), poly.end(),
      [&comp_angle] (const Vertex &a, const Vertex &b) {
        return comp_angle(a) < comp_angle(b);
      });

  return poly;
}

bool in_sorted_polygon(const Polygon &poly, const Vertex &vertex) {
  auto x = vertex.x;
  auto y = vertex.y;
  for (auto i = 1; i < poly.size(); i++) {
    auto vp = poly[i-1];
    auto v = poly[i];
    auto val = (x - v.x) * (vp.y - v.y)
             - (y - v.y) * (vp.x - v.x);
    if (val < 0) {
      return false;
    }
  }
  return true;
}

Vertex intersect_lines(const Vertex v1, const Vertex v2,
                       const Vertex v3, const Vertex v4)
{
  double x_divisor =
    (
     (v1.x - v2.x) * (v3.y - v4.y)
     -
     (v1.y - v2.y) * (v3.x - v4.x)
    );
  double y_divisor = 
    (
     (v1.y - v2.y) * (v3.x - v4.x)
     -
     (v1.x - v2.x) * (v3.y - v4.y)
    );
  if (x_divisor == 0.0 || y_divisor == 0.0) {
    throw std::domain_error("parallel lines");
  }

  double x =
    (
     (v1.x * v2.y - v1.y * v2.x) * (v3.x - v4.x)
     -
     (v1.x - v2.x) * (v3.x * v4.y - v3.y * v4.x)
    )
    / x_divisor;
  double y =
    (
     (v1.y * v2.x - v1.x * v2.y) * (v3.y - v4.y)
     -
     (v1.y - v2.y) * (v3.y * v4.x - v3.x * v4.y)
    )
    / y_divisor;
  
  // Make sure (x, y) is between v1 and v2 and between v3 and v4.
  if (std::abs(x - v1.x) > std::abs(v1.x - v2.x) ||
      std::abs(x - v2.x) > std::abs(v1.x - v2.x) ||
      std::abs(y - v1.y) > std::abs(v1.y - v2.y) ||
      std::abs(y - v2.y) > std::abs(v1.y - v2.y) ||
      std::abs(x - v3.x) > std::abs(v3.x - v4.x) ||
      std::abs(x - v4.x) > std::abs(v3.x - v4.x) ||
      std::abs(y - v3.y) > std::abs(v3.y - v4.y) ||
      std::abs(y - v4.y) > std::abs(v3.y - v4.y))
  {
    throw std::domain_error("intersection not on the segment");
  }

  return {x, y};
}

Polygon intersect_polygons(Polygon a, Polygon b) {
  Polygon intersection;
  if (a.size() <= 2 || b.size() <= 2) {
    return intersection;
  }

  // Sort vertices in clockwise order
  sort_vertices(a);
  sort_vertices(b);
  debug("\nSorted:");
  debug(a);
  debug(b);

  // Duplicate the first node as the last node
  a.emplace_back(a[0]);
  b.emplace_back(b[0]);

  // See if the corners are in the intersection
  for (auto it = a.begin() + 1; it != a.end(); it++) {
    if (in_sorted_polygon(b, *it)) {
      intersection.emplace_back(*it);
    }
  }
  for (auto it = b.begin() + 1; it != b.end(); it++) {
    if (in_sorted_polygon(a, *it)) {
      intersection.emplace_back(*it);
    }
  }

  // Get intersections now of intersecting lines
  for (auto i = 1; i < a.size(); i++) {
    auto &av1 = a[i-1];
    auto &av2 = a[i];
    for (auto j = 1; j < b.size(); j++) {
      auto &bv1 = b[j-1];
      auto &bv2 = b[j];
      try {
        intersection.emplace_back(intersect_lines(av1, av2, bv1, bv2));
      } catch (std::domain_error &ex) {}  // ignore
    }
  }

  return intersection;
}

double calculate_area(Polygon poly) {
  // Polygon vertices should be ordered either in clockwise or counterclockwise
  // order before calling this function.
  double area = 0.0;
  if (poly.size() <= 2) {
    return area;
  }

  sort_vertices(poly);
  poly.emplace_back(poly[0]);

  for (auto i = 1; i < poly.size(); i++) {
    auto &v_prev = poly[i-1];
    double xp = v_prev.x;
    double yp = v_prev.y;
    auto &v_curr = poly[i];
    double xc = v_curr.x;
    double yc = v_curr.y;
    area += xp * yc - xc * yp;
  }
  area *= 0.5;
  return std::abs(area);
}

int main() {
  // Read input
  debug("Read Polygons:");
  auto inlist = read_input();
  auto polygons = read_polygons(inlist);
  my_assert(polygons.size() == 2);
  for (auto &poly : polygons) {
    my_assert(poly.size() > 2);
    debug(poly);
    debug("  Area: " + std::to_string(calculate_area(poly)));
  }

  // Get the intersection
  Polygon intersection = intersect_polygons(polygons[0], polygons[1]);
  debug("\nIntersection:");
  debug(intersection);

  // Calculate the area
  double area = calculate_area(intersection);
  debug("\nIntersection Area:");
  printf("%0.02f\n", area);
  debug("");
}

