#include "Mesh.h"

#include "3rdparty/stl_reader.h"

#include <vector>

Mesh Mesh::from_stl(const std::string &fname) {
  using vec = std::vector<float>;
  using ivec = std::vector<size_t>;
  vec points, normals;
  ivec triangles, solid_ranges;

  stl_reader::ReadStlFile(fname.c_str(),
      points, normals, triangles, solid_ranges);

  Mesh mesh;
  mesh.triangles.reserve(triangles.size() / 3);
  mesh.points.reserve(points.size() / 3);

  for (size_t tri_idx = 0; tri_idx < triangles.size(); tri_idx += 3) {
    mesh.triangles.emplace_back(
        triangles[tri_idx],
        triangles[tri_idx + 1],
        triangles[tri_idx + 2]
        );
  }

  for (size_t pnt_idx = 0; pnt_idx < points.size(); pnt_idx += 3) {
    mesh.points.emplace_back(
        points[pnt_idx],
        points[pnt_idx + 1],
        points[pnt_idx + 2]
        );
  }

  return mesh;
}

