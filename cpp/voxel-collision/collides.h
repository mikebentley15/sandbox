#ifndef COLLIDES_H
#define COLLIDES_H

#include <cstddef> // for size_t

class VoxelObject;
class SparseVoxelObject;
template <size_t Nx, size_t Ny, size_t Nz> class CTVoxelObject;
template <size_t Nx, size_t Ny, size_t Nz> class CTSparseVoxelObject;
template <size_t N> class CTVoxelOctree;
class OctomapWrap;

bool collides (const VoxelObject &v1, const VoxelObject &v2);
bool collides (const SparseVoxelObject &v1, const SparseVoxelObject &v2);
bool collides (const CTVoxelObject<512, 512, 512> &v1, const CTVoxelObject<512, 512, 512> &v2);
bool collides (const CTVoxelObject<256, 256, 256> &v1, const CTVoxelObject<256, 256, 256> &v2);
bool collides (const CTVoxelObject<128, 128, 128> &v1, const CTVoxelObject<128, 128, 128> &v2);
bool collides (const CTSparseVoxelObject<512, 512, 512> &v1, const CTSparseVoxelObject<512, 512, 512> &v2);
bool collides (const CTSparseVoxelObject<256, 256, 256> &v1, const CTSparseVoxelObject<256, 256, 256> &v2);
bool collides (const CTSparseVoxelObject<128, 128, 128> &v1, const CTSparseVoxelObject<128, 128, 128> &v2);
bool collides (const CTSparseVoxelObject<128, 128, 128> &v1, const CTSparseVoxelObject<128, 128, 128> &v2);
bool collides (const CTVoxelOctree<512> &v1, const CTVoxelOctree<512> &v2);
bool collides (const CTVoxelOctree<256> &v1, const CTVoxelOctree<256> &v2);
bool collides (const CTVoxelOctree<128> &v1, const CTVoxelOctree<128> &v2);
bool collides (const CTVoxelOctree< 64> &v1, const CTVoxelOctree< 64> &v2);
bool collides (const CTVoxelOctree< 32> &v1, const CTVoxelOctree< 32> &v2);
bool collides (const CTVoxelOctree< 16> &v1, const CTVoxelOctree< 16> &v2);
bool collides (const CTVoxelOctree<  8> &v1, const CTVoxelOctree<  8> &v2);
bool collides (const CTVoxelOctree<  4> &v1, const CTVoxelOctree<  4> &v2);
bool collides (const OctomapWrap &v1, const OctomapWrap &v2);

#endif // COLLIDES_H
