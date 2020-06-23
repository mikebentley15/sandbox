#ifndef MESH_H
#define MESH_H

#include <fcl/data_types.h>
#include <fcl/math/vec_3f.h>

#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <string>
#include <vector>

/// represents a triangle mesh
struct Mesh {
  using Triangle = fcl::Triangle;
  using Vertex = fcl::Vec3f;

  std::vector<Vertex> vertices;
  std::vector<Triangle> triangles;
  std::string filename;

  bool empty() const { return vertices.empty() || triangles.empty(); }
  void to_stl(const std::string &fname);
  static Mesh from_stl(const std::string &fname);

  visualization_msgs::msg::Marker to_vis_marker() const;
};

#endif // MESH_H
