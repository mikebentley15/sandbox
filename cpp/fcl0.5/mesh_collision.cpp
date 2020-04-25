#include "Mesh.h"

#include <fcl/BVH/BVH_model.h>  // for meshes

#include <iomanip>
#include <iostream>

template <typename BV_A, typename BV_B>
bool collides(const fcl::BVHModel<BV_A> &a, const fcl::BVHModel<BV_B> &b) {
  return true;
}

int main(int argCount, char* argList[]) {
  if (argCount != 3) {
    std::cout << "Usage: " << argList[0] << " <stl-file> <stl-file>\n";
    return 1;
  }

  auto A = Mesh::from_stl(argList[1]).to_fcl<fcl::AABB>(); // first as axis-aligned
  auto B = Mesh::from_stl(argList[2]).to_fcl<fcl::OBB>();  // second as oriented

  std::cout << "collides: " << std::boolalpha << collides(A, B) << "\n";

  return 0;
}
