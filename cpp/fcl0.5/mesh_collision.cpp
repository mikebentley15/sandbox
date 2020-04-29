#include "Mesh.h"

#include <fcl/collision.h>      // for checkCollision
#include <fcl/BV/OBB.h>

#include <iomanip>
#include <iostream>

bool collides(const std::shared_ptr<fcl::CollisionObject> &a,
              const std::shared_ptr<fcl::CollisionObject> &b)
{
  fcl::CollisionRequest request; // default request
  fcl::CollisionResult result;
  fcl::collide(a.get(), b.get(), request, result);
  return result.isCollision();
}

int main(int argCount, char* argList[]) {
  if (argCount != 3) {
    std::cout << "Usage: " << argList[0] << " <stl-file> <stl-file>\n";
    return 1;
  }

  auto A = Mesh::from_stl(argList[1]).to_fcl_object<fcl::OBB>();
  auto B = Mesh::from_stl(argList[2]).to_fcl_object<fcl::OBB>();

  std::cout << "collides: " << std::boolalpha << collides(A, B) << "\n";

  return 0;
}
