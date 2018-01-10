#include <vector>
#include <iostream>

namespace pt {
  struct Point { int a, b; };
}

bool operator==(const pt::Point& lhs, const pt::Point& rhs) {
  return lhs.a == rhs.a && lhs.b == rhs.b;
}

int main() {
  std::vector<pt::Point> x { {1, 2}, {1, 3}, {4, 2}};
  std::vector<pt::Point> y { {1, 2}, {1, 3}, {4, 2}};

  std::cout << "x == y ? " << (x == y ? "true" : "false") << std::endl;

  return 0;
}
