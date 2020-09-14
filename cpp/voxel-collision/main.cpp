#include "VoxelObject.h"

#include <iostream>

int main() {
  VoxelObject<4, 24, 40> v;
  std::cout << "v is size (" << v.Nx << ", " << v.Ny << ", " << v.Nz << ")" << std::endl;
  std::cout << "v's type is size ("
    << decltype(v)::Nx << ", "
    << decltype(v)::Ny << ", "
    << decltype(v)::Nz << ")" << std::endl;
  std::cout << "sizeof(v): " << sizeof(v) << std::endl;

  for (size_t i = 0; i < v.Nx; i++) {
    std::cout << "i = " << i << std::endl;
    for (size_t j = 0; j < v.Ny; j++) {
      std::cout << "  ";
      for (size_t k = 0; k < v.Nz; k++) {
        std::cout << (v.cell(i, j, k) ? '1' : '0');
      }
      std::cout << std::endl;
    }
  }

  return 0;
}
