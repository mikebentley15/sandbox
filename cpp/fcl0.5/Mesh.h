#ifndef MESH_H
#define MESH_H

#include <fcl/BV/OBBRSS.h>
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
  template <typename BV=fcl::OBBRSS>
  std::shared_ptr<fcl::BVHModel<BV>> to_fcl_model() const {
    auto model = std::make_shared<fcl::BVHModel<BV>>();
    model->beginModel(triangles.size(), points.size());
    model->addSubModel(points, triangles);
    model->endModel();
    return model;
  }

  template <typename BV=fcl::OBBRSS>
  std::shared_ptr<fcl::CollisionObject> to_fcl_object() const {
    auto model = to_fcl_model<BV>();
    auto obj = std::make_shared<fcl::CollisionObject>(model);
    return obj;
  }
};

#endif // MESH_H
