#include "Mesh.h"

#include "stl_io.h"

#include <boost/filesystem.hpp>

#include <string>
#include <vector>

using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Point;
namespace fs = boost::filesystem;

namespace {

bool is_regular_file(const std::string &fname) {
  return fs::is_regular_file(fname);
}

std::string make_path_absolute(const std::string &fname) {
  try {
    return fs::canonical(fname).string();
  } catch (fs::filesystem_error &ex) {
    (void)ex; // unused
    // use absolute if file does not exist
    return fs::absolute(fname).string();
  }
}

} // end of unnamed namespace

void Mesh::to_stl(const std::string &fname) {
  write_stl_file_ascii(fname, vertices, triangles);
  if (this->filename.empty()) {
    this->filename = fname;
  }
}

Mesh Mesh::from_stl(const std::string &fname) {
  using vec = std::vector<float>;
  using ivec = std::vector<size_t>;
  vec points, normals;
  ivec triangles, solid_ranges;

  read_stl_file(fname.c_str(), points, normals, triangles, solid_ranges);

  Mesh mesh;
  mesh.filename = fname;
  mesh.triangles.reserve(triangles.size() / 3);
  mesh.vertices.reserve(points.size() / 3);

  for (size_t tri_idx = 0; tri_idx < triangles.size(); tri_idx += 3) {
    mesh.triangles.emplace_back(
        triangles[tri_idx],
        triangles[tri_idx + 1],
        triangles[tri_idx + 2]
        );
  }

  for (size_t pnt_idx = 0; pnt_idx < points.size(); pnt_idx += 3) {
    mesh.vertices.emplace_back(
        points[pnt_idx],
        points[pnt_idx + 1],
        points[pnt_idx + 2]
        );
  }

  return mesh;
}

Marker Mesh::to_vis_marker() const {
  Marker marker;
  marker.header.frame_id = "/map";
  marker.ns = "mesh_markers";
  marker.id = 4;
  marker.action = Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.4f;

  // convert the mesh
  if (is_regular_file(filename)) {
    marker.type = Marker::MESH_RESOURCE;
    marker.mesh_resource = "file://" + make_path_absolute(filename);
    marker.color.r = 1.0f; // make red if from filepath
    marker.color.g = 0.0f;
  } else {
    marker.type = Marker::TRIANGLE_LIST;
    for (auto &triangle : triangles) {
      for (int i = 0; i < 3; i++) {
        auto &vert = vertices[triangle[i]];
        Point p;
        p.x = vert[0];
        p.y = vert[1];
        p.z = vert[2];
        marker.points.push_back(p);
      }
    }
  }
  return marker;
}
