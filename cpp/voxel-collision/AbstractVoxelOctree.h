#ifndef ABSTRACT_VOXEL_OCTREE_H
#define ABSTRACT_VOXEL_OCTREE_H

#include <iostream>
#include <limits>    // for std::numeric_limits
#include <map>       // for std::map
#include <stack>     // for std::stack
#include <stdexcept> // for std::length_error
#include <tuple>     // for std::tuple
#include <utility>   // for std::pair

#include <cstddef> // for size_t
#include <cstdint> // for uint64_t

// discretizations need to match for all dimensions
// and it needs to be a power of 2 bigger than 4
class AbstractVoxelOctree {
public:
  AbstractVoxelOctree();
  AbstractVoxelOctree(const AbstractVoxelOctree &other) = default; // copy
  AbstractVoxelOctree(AbstractVoxelOctree &&other)      = default; // move

  virtual ~AbstractVoxelOctree() = default;

  virtual size_t Nx() const = 0;
  virtual size_t Ny() const = 0;
  virtual size_t Nz() const = 0;
  virtual size_t Nbx() const = 0;
  virtual size_t Nby() const = 0;
  virtual size_t Nbz() const = 0;

  virtual void set_xlim(double xmin, double xmax);
  virtual void set_ylim(double ymin, double ymax);
  virtual void set_zlim(double zmin, double zmax);

  std::pair<double, double> xlim() const { return {_xmin, _xmax}; }
  std::pair<double, double> ylim() const { return {_ymin, _ymax}; }
  std::pair<double, double> zlim() const { return {_zmin, _zmax}; }

  // size of cells
  virtual double dx() const = 0;
  virtual double dy() const = 0;
  virtual double dz() const = 0;

  // size of blocks
  virtual double dbx() const = 0;
  virtual double dby() const = 0;
  virtual double dbz() const = 0;

  virtual size_t nblocks() const = 0;

  virtual uint64_t block(size_t bx, size_t by, size_t bz) const = 0;
  virtual void set_block(size_t bx, size_t by, size_t bz, uint64_t value) = 0;

  virtual uint64_t union_block(size_t bx, size_t by, size_t bz, uint64_t value) = 0;
  virtual uint64_t intersect_block(size_t bx, size_t by, size_t bz, uint64_t value) = 0;

  virtual bool cell(size_t ix, size_t iy, size_t iz) const = 0;

  virtual bool set_cell(size_t ix, size_t iy, size_t iz, bool value = true) = 0;

  virtual uint64_t find_block(double x, double y, double z) const;
  virtual std::tuple<size_t, size_t, size_t> find_block_idx(double x, double y, double z) const = 0;
  virtual std::tuple<size_t, size_t, size_t> find_cell(double x, double y, double z) const = 0;
  virtual void add_point(double x, double y, double z);
  virtual void add_sphere(double x, double y, double z, double r) = 0;
  virtual void remove_interior() = 0;
  virtual bool collides_check(const AbstractVoxelOctree &other) const = 0;
  virtual bool collides(const AbstractVoxelOctree &other) const = 0;

protected:
  double _xmin;
  double _xmax;
  double _ymin;
  double _ymax;
  double _zmin;
  double _zmax;
};

#endif // ABSTRACT_VOXEL_OCTREE_H
