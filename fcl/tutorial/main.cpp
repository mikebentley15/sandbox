#include <fcl/common/types.h>
#include <fcl/math/vec_3f.h>
#include <fcl/math/triangle.h>

#include <memory>
#include <vector>

int main(void) {
  std::vector<Vector3f> vertices;
  std::vector<Triangle> triangles;
  
  using Model = BVHModel<OBBRSS>;
  auto model = std::make_unique<Model>(new Model());
  model->beginModel();
  model->addSubModel(vertices, triangles);
  model->endModel();

  return 0;
}
