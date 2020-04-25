#ifndef MESH_H
#define MESH_H

#include <fcl/BV/AABB.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/data_types.h>
#include <fcl/math/vec_3f.h>

#include <string>
#include <vector>


/// represents a triangle mesh
struct Mesh {
  using Triangle = fcl::Triangle;
  using Point = fcl::Vec3f;

  std::vector<Point> points;
  std::vector<Triangle> triangles;

  static Mesh from_stl(const std::string &fname);

  // TODO: test this somehow
  template <typename BV=fcl::AABB>
  fcl::BVHModel<BV> to_fcl() const {
    fcl::BVHModel<BV> model;
    model.beginModel(triangles.size(), points.size());
    model.addSubModel(points, triangles);
    model.endModel();
    return model;
  }
};

#endif // MESH_H
