#ifndef CT_VOXEL_OCTREE_WRAP_H
#define CT_VOXEL_OCTREE_WRAP_H

#include "CTVoxelOctree.h"

#include <stdexcept>
#include <string>
#include <variant>

class CTVoxelOctreeWrap {
public:
  using VoxelType = std::variant<CTVoxelOctree<  4>,
                                 CTVoxelOctree<  8>,
                                 CTVoxelOctree< 16>,
                                 CTVoxelOctree< 32>,
                                 CTVoxelOctree< 64>,
                                 CTVoxelOctree<128>,
                                 CTVoxelOctree<256>,
                                 CTVoxelOctree<512>>;

  const size_t Nx;      // number of voxels in the x-direction
  const size_t Ny;      // number of voxels in the y-direction
  const size_t Nz;      // number of voxels in the z-direction
  const size_t N;       // number of voxels

  const size_t Nbx;     // number of blocks in the x-direction
  const size_t Nby;     // number of blocks in the y-direction
  const size_t Nbz;     // number of blocks in the z-direction
  const size_t Nb;      // number of blocks

  using BlockType = uint64_t;

public:
  CTVoxelOctreeWrap(size_t _N);
  CTVoxelOctreeWrap(const CTVoxelOctreeWrap &other) = default;  // copy
  CTVoxelOctreeWrap(CTVoxelOctreeWrap &&other)      = default;  // move

  void set_xlim(double xmin, double xmax);
  void set_ylim(double ymin, double ymax);
  void set_zlim(double zmin, double zmax);

  std::pair<double, double> xlim() const;
  std::pair<double, double> ylim() const;
  std::pair<double, double> zlim() const;

  // size of cells
  double dx() const;
  double dy() const;
  double dz() const;

  // size of blocks
  double dbx() const;
  double dby() const;
  double dbz() const;

  size_t nblocks() const;

  uint64_t block(size_t bx, size_t by, size_t bz) const;
  void set_block(size_t bx, size_t by, size_t bz, uint64_t value);

  // returns old block value before unioning or intersecting with value
  uint64_t union_block(size_t bx, size_t by, size_t bz, uint64_t value);
  uint64_t intersect_block(size_t bx, size_t by, size_t bz, uint64_t value);

  bool cell(size_t ix, size_t iy, size_t iz) const;

  // sets the cell's value
  // returns true if the cell's value changed
  bool set_cell(size_t ix, size_t iy, size_t iz, bool value = true);

  uint64_t find_block(double x, double y, double z) const;
  std::tuple<size_t, size_t, size_t> find_block_idx(double x, double y, double z) const;
  std::tuple<size_t, size_t, size_t> find_cell(double x, double y, double z) const;
  void add_point(double x, double y, double z);
  void add_sphere(double x, double y, double z, double r);
  void remove_interior_slow_1();
  void remove_interior();
  bool collides_check(const CTVoxelOctreeWrap &other) const;
  bool collides(const CTVoxelOctreeWrap &other) const;

private:
  VoxelType _data;
};

#include "CTVoxelOctreeWrap.hxx"

#endif // CT_VOXEL_OCTREE_WRAP_H
