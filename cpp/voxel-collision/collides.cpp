#include "collides.h"
#include "CTSparseVoxelObject.h"
#include "CTVoxelObject.h"
#include "SparseVoxelObject.h"
#include "VoxelObject.h"
#include "VoxelOctree.h"
#include "OctomapWrap.h"

namespace {

template <typename VType> bool collides_impl(const VType &v1, const VType &v2) {
  return v1.collides(v2);
}

} // end of unnamed namespace

#define COLLIDES_IMPL(VType) \
  bool collides (const VType &v1, const VType &v2) { return collides_impl(v1, v2); }

using ctv512 = CTVoxelObject<512, 512, 512>;
using ctv256 = CTVoxelObject<256, 256, 256>;
using ctv128 = CTVoxelObject<128, 128, 128>;
using ctsv512 = CTSparseVoxelObject<512, 512, 512>;
using ctsv256 = CTSparseVoxelObject<256, 256, 256>;
using ctsv128 = CTSparseVoxelObject<128, 128, 128>;

COLLIDES_IMPL(VoxelObject)
COLLIDES_IMPL(SparseVoxelObject)
COLLIDES_IMPL(ctv512)
COLLIDES_IMPL(ctv256)
COLLIDES_IMPL(ctv128)
COLLIDES_IMPL(ctsv512)
COLLIDES_IMPL(ctsv256)
COLLIDES_IMPL(ctsv128)
COLLIDES_IMPL(VoxelOctree<512>)
COLLIDES_IMPL(VoxelOctree<256>)
COLLIDES_IMPL(VoxelOctree<128>)
COLLIDES_IMPL(OctomapWrap)

#undef COLLIDES_IMPL
