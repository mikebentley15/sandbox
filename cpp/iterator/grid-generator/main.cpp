#include "GridGenerator.h"

#include <iostream>

int main() {
  GridGenerator gen;
  gen.add_dim(1.0, 2.5, 6);
  gen.add_dim(0.4, 1.0, 4);
  gen.add_dim(0.0, 10.0, 11);
  for (auto val : gen) {
    std::cout << val[0] << ",\t" << val[1] << ",\t" << val[2] << std::endl;
  }
  return 0;
}
