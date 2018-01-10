#include <algorithm>
#include <ios>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include <cstring>  // for memcpy
#include <cmath>    // for sqrt and atan2

using stringlist  = std::vector<std::string>;
using intlist     = std::vector<int>;
using stringset   = std::unordered_set<std::string>;
using intset      = std::unordered_set<unsigned int>;

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
void print_lines(const T &outlist, bool print = true) {
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

template <typename A, typename B>
std::vector<A> map_keys(const std::map<A,B> &m) {
  std::vector<A> keys;
  for (auto &ab : m) {
    keys.emplace_back(ab.first);
  }
  return keys;
}

template <typename A, typename B>
std::vector<A> map_keys(const std::unordered_map<A,B> &m) {
  std::vector<A> keys;
  for (auto &ab : m) {
    keys.emplace_back(ab.first);
  }
  return keys;
}

template <typename A, typename B>
std::vector<B> map_values(const std::map<A,B> &m) {
  std::vector<B> values;
  for (auto &ab : m) {
    values.emplace_back(ab.second);
  }
  return values;
}

template <typename A, typename B>
std::vector<B> map_values(const std::unordered_map<A,B> &m) {
  std::vector<B> values;
  for (auto &ab : m) {
    values.emplace_back(ab.second);
  }
  return values;
}

struct Point {
  uint x;
  uint y;
  Point(uint _x, uint _y) : x(_x), y(_y) {}
};

struct BoundingBox {
  int x;
  int y;
  int w;
  int h;
  BoundingBox(int _x, int _y, int _w, int _h)
    : x(_x), y(_y), w(_w), h(_h)
  {
    standardize();
  }

  void standardize() {
    if (w < 0) {
      w = -w;
      x -= w;
    }
    if (h < 0) {
      h = -h;
      y -= h;
    }
  }

  void move_top_left(Point dest) {
    double ratio = static_cast<double>(w) / h;
    Point br(x+w, y+h);
    int w1 = br.x - dest.x;
    int h2 = br.y - dest.y;
    int h1 = std::abs(w1 / ratio) * (h2 > 0 ? 1 : -1);
    int w2 = std::abs(h2 * ratio) * (w1 > 0 ? 1 : -1);
    if (abs(w1) > abs(w2)) {
      x = br.x - w1;
      y = br.y - h1;
      w = w1;
      h = h1;
    } else {
      x = br.x - w2;
      y = br.y - h2;
      w = w2;
      h = h2;
    }
    standardize();
  }

  void move_bottom_left(Point dest) {
    double ratio = static_cast<double>(w) / h;
    Point tr(x+w, y);
    int w1 = tr.x - dest.x;
    int h2 = dest.y - tr.y;
    int h1 = std::abs(w1 / ratio) * (h2 > 0 ? 1 : -1);
    int w2 = std::abs(h2 * ratio) * (w1 > 0 ? 1 : -1);
    if (abs(w1) > abs(w2)) {
      x = tr.x - w1;
      y = tr.y;
      w = w1;
      h = h1;
    } else {
      x = tr.x - w2;
      y = tr.y;
      w = w2;
      h = h2;
    }
    standardize();
  }

  void move_bottom_right(Point dest) {
    double ratio = static_cast<double>(w) / h;
    Point tl(x, y);
    int w1 = dest.x - tl.x;
    int h2 = dest.y - tl.y;
    int h1 = std::abs(w1 / ratio) * (h2 > 0 ? 1 : -1);
    int w2 = std::abs(h2 * ratio) * (w1 > 0 ? 1 : -1);
    if (abs(w1) > abs(w2)) {
      x = tl.x;
      y = tl.y;
      w = w1;
      h = h1;
    } else {
      x = tl.x;
      y = tl.y;
      w = w2;
      h = h2;
    }
    standardize();
  }

  void move_top_right(Point dest) {
    double ratio = static_cast<double>(w) / h;
    Point bl(x, y+h);
    int w1 = dest.x - bl.x;
    int h2 = bl.y - dest.y;
    int h1 = std::abs(w1 / ratio) * (h2 > 0 ? 1 : -1);
    int w2 = std::abs(h2 * ratio) * (w1 > 0 ? 1 : -1);
    if (abs(w1) > abs(w2)) {
      x = bl.x;
      y = bl.y - h1;
      w = w1;
      h = h1;
    } else {
      x = bl.x;
      y = bl.y - h2;
      w = w2;
      h = h2;
    }
    standardize();
  }
};

struct Description {
  BoundingBox box;
  std::string corner;
  Point cursor;
  Description(const BoundingBox &_box, const std::string &_corner,
              const Point &_cursor)
    : box(_box), corner(_corner), cursor(_cursor) {}
};
using DescriptionList = std::vector<Description>;

std::ostream& operator<<(std::ostream &out, const BoundingBox &box) {
  out << box.x << " " << box.y << " " << box.w << " " << box.h;
  return out;
}

std::ostream& operator<<(std::ostream &out, const Point &p) {
  out << p.x << " " << p.y;
  return out;
}

std::ostream& operator<<(std::ostream &out, const Description &desc) {
  out << "Description{"
      << "(" << desc.box << "), "
      << desc.corner << ", "
      << "(" << desc.cursor << ")"
      << "}";
  return out;
}

DescriptionList parse_descriptions(const stringlist &inlist) {
  DescriptionList descriptions;
  for (auto it = inlist.begin() + 1; it != inlist.end();) {
    auto box_specs = str_split<uint>(*it++);
    auto &corner = *it++;
    auto cursor_specs = str_split<uint>(*it++);
    descriptions.emplace_back(
        BoundingBox(box_specs[0], box_specs[1], box_specs[2], box_specs[3]),
        corner, Point(cursor_specs[0], cursor_specs[1]));
  }
  return descriptions;
}

BoundingBox resize_box(const BoundingBox &box, const std::string &corner,
                       const Point &cursor)
{
  BoundingBox resized = box;
  if (corner == "TopLeft") {
    resized.move_top_left(cursor);
  } else if (corner == "BottomLeft") {
    resized.move_bottom_left(cursor);
  } else if (corner == "BottomRight") {
    resized.move_bottom_right(cursor);
  } else if (corner == "TopRight") {
    resized.move_top_right(cursor);
  } else {
    throw std::runtime_error("Unimplemented corner type");
  }
  return resized;
}

int main() {
  debug("Read Input");
  auto inlist = read_input();

  debug("\nParse Descriptions:");
  DescriptionList descriptions = parse_descriptions(inlist);
  print_lines(descriptions, DEBUG);

  debug("\nResize and print");
  for (auto &d : descriptions) {
    debug("  Resize " + d.corner);
    std::cout << resize_box(d.box, d.corner, d.cursor) << std::endl;
  }
}

