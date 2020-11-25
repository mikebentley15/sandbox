#ifndef CT_VOXEL_OCTREE_H
#define CT_VOXEL_OCTREE_H

#include <iostream>
#include <limits>    // for std::numeric_limits
#include <map>       // for std::map
#include <stack>     // for std::stack
#include <stdexcept> // for std::length_error
#include <tuple>     // for std::tuple
#include <utility>   // for std::pair

#include <cstddef> // for size_t
#include <cstdint> // for uint64_t

namespace detail {
  template <size_t _Nt> class TreeNode;
} // end of namespace detail

// discretizations need to match for all dimensions
// and it needs to be a power of 2 bigger than 4
template <size_t _N>
class CTVoxelOctree {

public:
  using BlockType = uint64_t;

public:
  CTVoxelOctree();
  CTVoxelOctree(const CTVoxelOctree<_N> &other); // copy
  CTVoxelOctree(CTVoxelOctree<_N> &&other);      // move

  size_t Nx()  const { return _N; }
  size_t Ny()  const { return _N; }
  size_t Nz()  const { return _N; }
  size_t Nbx() const { return _N/4; }
  size_t Nby() const { return _N/4; }
  size_t Nbz() const { return _N/4; }

  void set_xlim(double xmin, double xmax);
  void set_ylim(double ymin, double ymax);
  void set_zlim(double zmin, double zmax);

  std::pair<double, double> xlim() const { return {_xmin, _xmax}; }
  std::pair<double, double> ylim() const { return {_ymin, _ymax}; }
  std::pair<double, double> zlim() const { return {_zmin, _zmax}; }

  // size of cells
  double dx() const { return _dx; }
  double dy() const { return _dy; }
  double dz() const { return _dz; }

  // size of blocks
  double dbx() const { return _dx * 4; }
  double dby() const { return _dy * 4; }
  double dbz() const { return _dz * 4; }

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
  bool collides_check(const CTVoxelOctree<_N> &other) const;
  bool collides(const CTVoxelOctree<_N> &other) const;

protected:
  // one in the given place, zeros everywhere else
  // for x, y, z within a block (each should be 0 <= x < 4)
  uint64_t bitmask(uint_fast8_t x, uint_fast8_t y, uint_fast8_t z) const;

  void domain_check(double x, double y, double z) const;
  void limit_check(const CTVoxelOctree &other) const;

private:
  std::unique_ptr<detail::TreeNode<_N>> _data; // sparse voxel tree data
  double _xmin;
  double _xmax;
  double _ymin;
  double _ymax;
  double _zmin;
  double _zmax;
  double _dx;
  double _dy;
  double _dz;
};

#include "CTVoxelOctree.hxx"

#endif // CT_VOXEL_OCTREE_H
