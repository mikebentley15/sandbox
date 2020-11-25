#ifndef DERIVED_VOXEL_OCTREE_H
#define DERIVED_VOXEL_OCTREE_H

#include "AbstractVoxelOctree.h"
#include "detail/TreeNode.h"

#include <memory>

template <size_t _N>
class DerivedVoxelOctree : public AbstractVoxelOctree {
  
public:
  
  virtual size_t Nx()  const override { return _N; }
  virtual size_t Ny()  const override { return _N; }
  virtual size_t Nz()  const override { return _N; }
  virtual size_t Nbx() const override { return _N/4; }
  virtual size_t Nby() const override { return _N/4; }
  virtual size_t Nbz() const override { return _N/4; }

  size_t N()  const { return Nx() * Ny() * Nz(); }
  size_t Nb() const { return Nbx() * Nby() * Nbz(); }

  using BlockType = uint64_t;

public:
  DerivedVoxelOctree();
  DerivedVoxelOctree(const DerivedVoxelOctree<_N> &other); // copy
  DerivedVoxelOctree(DerivedVoxelOctree<_N> &&other);      // move

  virtual void set_xlim(double xmin, double xmax) override;
  virtual void set_ylim(double ymin, double ymax) override;
  virtual void set_zlim(double zmin, double zmax) override;

  // size of cells
  virtual double dx() const override { return _dx; }
  virtual double dy() const override { return _dy; }
  virtual double dz() const override { return _dz; }

  // size of blocks
  virtual double dbx() const override { return _dx * 4; }
  virtual double dby() const override { return _dy * 4; }
  virtual double dbz() const override { return _dz * 4; }

  virtual size_t nblocks() const override;

  virtual uint64_t block(size_t bx, size_t by, size_t bz) const override;
  virtual void set_block(size_t bx, size_t by, size_t bz, uint64_t value) override;

  // returns old block value before unioning or intersecting with value
  virtual uint64_t union_block(size_t bx, size_t by, size_t bz, uint64_t value) override;
  virtual uint64_t intersect_block(size_t bx, size_t by, size_t bz, uint64_t value) override;

  virtual bool cell(size_t ix, size_t iy, size_t iz) const override;
  virtual bool set_cell(size_t ix, size_t iy, size_t iz, bool value = true) override;

  virtual std::tuple<size_t, size_t, size_t> find_block_idx(double x, double y, double z) const override;
  virtual std::tuple<size_t, size_t, size_t> find_cell(double x, double y, double z) const override;
  virtual void add_sphere(double x, double y, double z, double r) override;
  virtual void remove_interior_slow_1();
  virtual void remove_interior() override;

  // assumed other is the same as this object, so dynamic cast
  virtual bool collides_check(const AbstractVoxelOctree &other) const override;
  virtual bool collides(const AbstractVoxelOctree &other) const override;

protected:
  // one in the given place, zeros everywhere else
  // for x, y, z within a block (each should be 0 <= x < 4)
  uint64_t bitmask(uint_fast8_t x, uint_fast8_t y, uint_fast8_t z) const;

  void domain_check(double x, double y, double z) const;
  void limit_check(const DerivedVoxelOctree &other) const;

protected:
  std::unique_ptr<detail::TreeNode<_N>> _data; // sparse voxel tree data
  double _dx;
  double _dy;
  double _dz;
};

#include "DerivedVoxelOctree.hxx"

#endif // DERIVED_VOXEL_OCTREE_H
