#ifndef CT_VOXEL_OCTREE_H
#define CT_VOXEL_OCTREE_H

#include <functional>
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

/** Occupancy octree for voxelization of a collision space.
 *
 * This is a sparse representation of a voxelization.  The space is separated
 * into individual voxels which can be either empty or occupied.  This is
 * condensed into blocks of 4x4x4.  Each block is represented by a single
 * 64-bit unsigned integer, one bit per voxel to indicate occupied or not.
 *
 * Only the blocks that have occupied voxel cells are stored.  Those blocks
 * that are occupied are stored into a tree structure.  At each level, the
 * space is subdivided in each of the three dimensions, making eight
 * subdivisions.  That is where the name Octree comes from.  For collision
 * checking against a particular point, we recurse down the octant child until
 * we either get to a null pointer (meaning that whole subtree is empty space)
 * or a leaf node containing a single 64-bit unsigned integer.
 *
 * You can specify the limits of the space.  The bottom-left of each voxel
 * (i.e., toward the origin) is the coordinate of that voxel.  This is
 * different than many other voxel implementations where the center of the
 * voxel is the voxel coordinate.  However, having the center be the voxel
 * coordinate causes geometric changes when scaling up or down to a different
 * voxelization resultion.  No such problem occurs when using the bottom-left
 * corner as the voxel coordinate.  Instead the center of a voxel at (ix, iy, iz) is
 *
 *   Point voxel_center(
 *       xlim().first + (ix + 0.5) * dx(),
 *       ylim().first + (iy + 0.5) * dy(),
 *       zlim().first + (iz + 0.5) * dz()
 *       );
 *
 * Note: discretizations need to match for all dimensions and it needs to be a
 * power of 2 bigger than 4.  We may be able to remove this limitation later,
 * but for now, it seems to work well.
 */
template <size_t _N>
class CTVoxelOctree {

public:
  using BlockType = uint64_t;

public:
  CTVoxelOctree();
  CTVoxelOctree(const CTVoxelOctree<_N> &other) = default; // copy
  CTVoxelOctree(CTVoxelOctree<_N> &&other) = default;      // move

  CTVoxelOctree& operator= (const CTVoxelOctree<_N> &other) = default; // copy
  CTVoxelOctree& operator= (CTVoxelOctree<_N> &&other) = default;      // move

  bool operator== (const CTVoxelOctree<_N> &other) const;

  // number of voxels in each dimension
  static constexpr size_t Nx()    { return _N; }
  static constexpr size_t Ny()    { return _N; }
  static constexpr size_t Nz()    { return _N; }
  static constexpr size_t Ntot()  { return Nx() * Ny() * Nz(); }

  // number of blocks in each dimension
  static constexpr size_t Nbx()   { return Nx()/4; }
  static constexpr size_t Nby()   { return Ny()/4; }
  static constexpr size_t Nbz()   { return Nz()/4; }
  static constexpr size_t Nbtot() { return Nbx() * Nby() * Nbz(); }

  void copy_limits(const CTVoxelOctree<_N> &other);
  CTVoxelOctree<_N> empty_copy() const; // covers same space, but empty

  void set_xlim(double xmin, double xmax);
  void set_ylim(double ymin, double ymax);
  void set_zlim(double zmin, double zmax);

  void set_xlim(const std::pair<double, double> &lim);
  void set_ylim(const std::pair<double, double> &lim);
  void set_zlim(const std::pair<double, double> &lim);

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

  bool is_in_domain(double x, double y, double z) const;

  // Note: bounds checks are NOT performed for performance reasons
  uint64_t block(size_t bx, size_t by, size_t bz) const;
  void set_block(size_t bx, size_t by, size_t bz, uint64_t value);

  // returns old block value before unioning or intersecting with value
  // Note: bounds checks are NOT performed for performance reasons
  uint64_t union_block(size_t bx, size_t by, size_t bz, uint64_t value);
  uint64_t intersect_block(size_t bx, size_t by, size_t bz, uint64_t value);
  uint64_t subtract_block(size_t bx, size_t by, size_t bz, uint64_t value);

  bool cell(size_t ix, size_t iy, size_t iz) const;

  // sets the cell's value
  // returns true if the cell's value changed
  bool set_cell(size_t ix, size_t iy, size_t iz, bool value = true);

  uint64_t find_block(double x, double y, double z) const;
  std::tuple<size_t, size_t, size_t> find_block_idx(double x, double y, double z) const;
  std::tuple<size_t, size_t, size_t> find_cell(double x, double y, double z) const;
  void add_point(double x, double y, double z);
  void add_sphere(double x, double y, double z, double r);
  void add_voxels(const CTVoxelOctree<_N> &other);
  void remove_interior_slow_1();
  void remove_interior();
  bool collides_check(const CTVoxelOctree<_N> &other) const; // checks limits too
  bool collides(const CTVoxelOctree<_N> &other) const;

  void remove_point(double x, double y, double z);
  void remove_voxels(const CTVoxelOctree<_N> &other);

  void intersect_voxels(const CTVoxelOctree<_N> &other);

  // Have a callback function at each occupied block
  void visit_leaves(
      const std::function<void(size_t, size_t, size_t, uint64_t)> &visitor) const;


protected:
  // one in the given place, zeros everywhere else
  // for x, y, z within a block (each should be 0 <= x < 4)
  uint64_t bitmask(uint_fast8_t x, uint_fast8_t y, uint_fast8_t z) const;

  void domain_check(double x, double y, double z) const;
  void limit_check(const CTVoxelOctree &other) const;

private:
  detail::TreeNode<_N> _data; // sparse voxel tree data
  double _xmin;
  double _xmax;
  double _ymin;
  double _ymax;
  double _zmin;
  double _zmax;
  double _dx;
  double _dy;
  double _dz;
}; // end of class CTVoxelOctree<_N>

#include "CTVoxelOctree.hxx"

#endif // CT_VOXEL_OCTREE_H
