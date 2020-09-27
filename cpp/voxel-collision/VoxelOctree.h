#ifndef VOXEL_OCTREE_H
#define VOXEL_OCTREE_H

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

template <size_t _Nt> // Nt = voxels in each dimension
struct TreeNode {
  static_assert(_Nt % 2 == 0, "TreeNode: number of nodes must be even");

  static const size_t Nt = _Nt;
  static const size_t N = _Nt * _Nt * _Nt;

  using Child = TreeNode<Nt/2>;
  using ChildPtr = std::unique_ptr<Child>;

  TreeNode() : _children() {}
  TreeNode(const TreeNode<_Nt> &other) { // copy
    for (int i = 0; i < 8; i++) {
      auto &child = other._children[i];
      if (child) {
        _children[i] = std::make_unique<Child>(*child);
      }
    }
  }

  size_t nblocks() const {
    size_t child_nblocks = 0;
    for (auto &child : _children) {
      if (child) {
        child_nblocks += child->nblocks();
      }
    }
    return child_nblocks;
  }

  uint64_t block(size_t bx, size_t by, size_t bz) const {
    ChildPtr &child = _child(bx, by, bz);
    if (child) {
      return child->block(bx / 2, by / 2, bz / 2);
    }
    return 0;
  }

  void set_block(size_t bx, size_t by, size_t bz, uint64_t value) {
    ChildPtr &child = _child(bx, by, bz);
    if (!child) {
      child = std::make_unique<Child>();
    }
    child->set_block(bx / 2, by / 2, bz / 2, value);
  }

  uint64_t union_block(size_t bx, size_t by, size_t bz, uint64_t value) {
    ChildPtr &child = _child(bx, by, bz);
    if (!child) {
      child = std::make_unique<Child>();
    }
    return child->union_block(bx / 2, by / 2, bz / 2, value);
  }

  uint64_t intersect_block(size_t bx, size_t by, size_t bz, uint64_t value) {
    ChildPtr &child = _child(bx, by, bz);
    if (child) {
      return child->intersect_block(bx, by, bz, value);
    }
    return 0;
  }

  bool collides(const TreeNode<Nt> &other) const {
    for (int i = 0; i < 8; i++) {
      if (_children[i] && other._children[i]
          && _children[i]->collides(*other._children[i]))
      {
        return true;
      }
    }
    return false;
  }

protected:
  const ChildPtr& _child(size_t bx, size_t by, size_t bz) const {
    return _children[(bz%2) + 2 * ((by%2) + 2 * (bx%2))];
  }

  ChildPtr& _child(size_t bx, size_t by, size_t bz) {
    return _children[(bz%2) + 2 * ((by%2) + 2 * (bx%2))];
  }

protected:
  std::array<ChildPtr, 8> _children {};
};

// leaf node
template <> struct TreeNode<4> {
  static const size_t Nt = 4;
  static const size_t N = 64;
  size_t nblocks() const { return 1; }
  uint64_t block(size_t, size_t, size_t) const { return _data; }
  uint64_t union_block(size_t, size_t, size_t, uint64_t value) {
    auto prev = _data;
    _data |= value;
    return prev;
  }
  uint64_t intersect_block(size_t, size_t, size_t, uint64_t value) {
    auto prev = _data;
    _data &= value;
    return prev;
  }
  void set_block(size_t, size_t, size_t, uint64_t value) { _data = value; }
  bool collides(const TreeNode<4> &other) const { return _data & other._data; }
protected:
  uint64_t _data;
};

} // end of namespace detail

// discretizations need to match for all dimensions
// and it needs to be a power of 2 bigger than 4
template <size_t _N>
class VoxelOctree {

public:
  static const size_t Nx = _N;            // number of voxels in the x-direction
  static const size_t Ny = Nx;            // number of voxels in the y-direction
  static const size_t Nz = Nx;            // number of voxels in the z-direction
  static const size_t N  = Nx*Ny*Nz;      // number of voxels

  static const size_t Nbx = _N / 4;       // number of blocks in the x-direction
  static const size_t Nby = Nbx;          // number of blocks in the y-direction
  static const size_t Nbz = Nbx;          // number of blocks in the z-direction
  static const size_t Nb  = Nbx*Nby*Nbz;  // number of blocks

  using BlockType = uint64_t;

protected:
public:
  VoxelOctree() : _data(new detail::TreeNode<_N>()) {
    set_xlim(0.0, 1.0);
    set_ylim(0.0, 1.0);
    set_zlim(0.0, 1.0);
  }
  VoxelOctree(const VoxelOctree<_N> &other) // copy
    : _data(new detail::TreeNode<_N>(*(other._data)))
    , _xmin(other._xmin), _xmax(other._xmax)
    , _ymin(other._ymin), _ymax(other._ymax)
    , _zmin(other._zmin), _zmax(other._zmax)
    , _dx(other._dx), _dy(other._dy), _dz(other._dz)
  {}

  VoxelOctree(VoxelOctree<_N> &&other)  // move
    : _data(std::move(other._data))
    , _xmin(other._xmin), _xmax(other._xmax)
    , _ymin(other._ymin), _ymax(other._ymax)
    , _zmin(other._zmin), _zmax(other._zmax)
    , _dx(other._dx), _dy(other._dy), _dz(other._dz)
  {}

  void set_xlim(double xmin, double xmax) {
    if (xmin >= xmax) {
      throw std::length_error("xlimits must be positive in size");
    }
    _xmin = xmin;
    _xmax = xmax;
    _dx = (xmax - xmin) / Nx;
  }

  void set_ylim(double ymin, double ymax) {
    if (ymin >= ymax) {
      throw std::length_error("ylimits must be positive in size");
    }
    _ymin = ymin;
    _ymax = ymax;
    _dy = (ymax - ymin) / Ny;
  }

  void set_zlim(double zmin, double zmax) {
    if (zmin >= zmax) {
      throw std::length_error("zlimits must be positive in size");
    }
    _zmin = zmin;
    _zmax = zmax;
    _dz = (zmax - zmin) / Nz;
  }

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

  size_t nblocks() const { return _data->nblocks(); }

  uint64_t block(size_t bx, size_t by, size_t bz) const {
    return _data->block(bx, by, bz);
  }

  void set_block(size_t bx, size_t by, size_t bz, uint64_t value) {
    _data->set_block(bx, by, bz, value);
  }

  // returns old block type before unioning with value
  uint64_t union_block(size_t bx, size_t by, size_t bz, uint64_t value) {
    return _data->union_block(bx, by, bz, value);
  }

  uint64_t intersect_block(size_t bx, size_t by, size_t bz, uint64_t value) {
    return _data->intersect_block(bx, by, bz, value);
  }

  bool cell(size_t ix, size_t iy, size_t iz) const {
    auto b = block(ix / 4, iy / 4, iz / 4);
    return b && (b & bitmask(ix % 4, iy % 4, iz % 4));
  }

  // sets the cell's value
  // returns true if the cell's value changed
  bool set_cell(size_t ix, size_t iy, size_t iz, bool value = true) {
    auto mask = bitmask(ix % 4, iy % 4, iz % 4);
    uint64_t oldval;
    if (value) {
      oldval = _data->union_block(ix / 4, iy / 4, iz / 4, mask);
      return oldval & mask;
    } else {
      oldval = _data->intersect_block(ix / 4, iy / 4, iz / 4, ~mask);
      return oldval & ~mask;
    }
  }

  uint64_t find_block(double x, double y, double z) const {
    auto [bx, by, bz] = find_block_idx(x, y, z);
    return block(bx, by, bz);
  }

  std::tuple<size_t, size_t, size_t> find_block_idx(double x, double y, double z) const {
    domain_check(x, y, z);
    size_t ix = size_t((x - _xmin) / _dx);
    size_t iy = size_t((y - _ymin) / _dy);
    size_t iz = size_t((z - _zmin) / _dz);
    return {ix / 4, iy / 4, iz / 4};
  }

  std::tuple<size_t, size_t, size_t> find_cell(double x, double y, double z) const {
    domain_check(x, y, z);
    size_t ix = size_t((x - _xmin) / _dx);
    size_t iy = size_t((y - _ymin) / _dy);
    size_t iz = size_t((z - _zmin) / _dz);
    return {ix, iy, iz};
  }

  void add_point(double x, double y, double z) {
    auto [ix, iy, iz] = find_cell(x, y, z);
    set_cell(ix, iy, iz);
  }

  // TODO: easier algorithm is to go through each voxel, create an FCL AABB box
  // TODO- and collision check it against the object to be added (for sphere,
  // TODO- capsule, and mesh).  Slow, but effective

  // sets sphere as occupied in voxel space, with center and radius specified
  // adds any voxels that intersect or are inside of the sphere.
  void add_sphere(double x, double y, double z, double r) {
    // Algorithm:
    // 1. add the sphere center to the voxelization
    // 2. grow the sphere center, asking each voxel center to see if it is inside
    //    the sphere
    // 3. for each voxel adjacent to the end of the growing, check if that voxel
    //    box collides with the sphere at all.  If so, add it to the voxelization

    // 1. add center
    auto [ix, iy, iz] = find_cell(x, y, z);
    this->set_cell(ix, iy, iz);

    // 2. grow the sphere center, asking each voxel center to see if it is inside
    //    the sphere
    auto is_in_sphere = [x, y, z, r](double _x, double _y, double _z) {
      double dx = x - _x;
      double dy = y - _y;
      double dz = z - _z;
      bool is_in = dx*dx + dy*dy + dz*dz <= r*r;
      //std::cout << "  add_sphere(): checking is in sphere "
      //             "(" << _x << ", " << _y << ", " << _z << "): "
      //          << is_in << std::endl;
      return is_in;
    };

    // checks the center of the voxel against the sphere
    auto voxel_ctr_is_in_sphere = [&is_in_sphere, this]
      (size_t _ix, size_t _iy, size_t _iz) {
        double x = this->_dx * (_ix + 0.5);
        double y = this->_dy * (_iy + 0.5);
        double z = this->_dz * (_iz + 0.5);
        return is_in_sphere(x, y, z);
      };

    // just check all of them
    //for (size_t ix = 0; ix < Nx; ix++) {
    //  for (size_t iy = 0; iy < Ny; iy++) {
    //    for (size_t iz = 0; iz < Nz; iz++) {
    //      if (voxel_ctr_is_in_sphere(ix, iy, iz)) {
    //        this->set_cell(ix, iy, iz);
    //      }
    //    }
    //  }
    //}
    for (size_t bx = 0; bx < Nbx; bx++) {
      for (size_t by = 0; by < Nby; by++) {
        for (size_t bz = 0; bz < Nbz; bz++) {
          // check this block
          uint64_t b = 0;
          for (uint_fast8_t i = 0; i < 4; i++) {
            for (uint_fast8_t j = 0; j < 4; j++) {
              for (uint_fast8_t k = 0; k < 4; k++) {
                if (voxel_ctr_is_in_sphere((bx<<2) + i, (by<<2) + j, (bz<<2) + k)) {
                  b |= bitmask(i, j, k);
                }
              }
            }
          }
          if (b) { union_block(bx, by, bz, b); }
        }
      }
    }

    //// do a growing algorithm with a frontier and a visited
    //using IdxType = std::tuple<size_t, size_t, size_t>;
    //std::stack<IdxType> frontier;
    //auto visited = std::make_unique<VoxelObject<Nx, Ny, Nz>>();

    //auto check_push = [&frontier, &visited](size_t _ix, size_t _iy, size_t _iz) {
    //  if (!visited->cell(_ix, _iy, _iz) && _ix < Nx && _iy < Ny && _iz < Nz) {
    //    //std::cout << "  add_sphere(): pushing: "
    //    //          << _ix << ", " << _iy << ", " << _iz << std::endl;
    //    frontier.push(IdxType{_ix, _iy, _iz});
    //    visited->set_cell(_ix, _iy, _iz);
    //  }
    //};

    //check_push(ix, iy, iz);
    //while(!frontier.empty()) {
    //  auto [ix, iy, iz] = frontier.top();
    //  frontier.pop();
    //  if (voxel_ctr_is_in_sphere(ix, iy, iz)) {
    //    this->set_cell(ix, iy, iz);
    //    check_push(ix-1, iy, iz);
    //    check_push(ix+1, iy, iz);
    //    check_push(ix, iy-1, iz);
    //    check_push(ix, iy+1, iz);
    //    check_push(ix, iy, iz-1);
    //    check_push(ix, iy, iz+1);
    //  }
    //}

    // 3. for each voxel adjacent to the end of the growing, check if that voxel
    //    box collides with the sphere at all.  If so, add it to the voxelization
    // TODO: implement
  }

  bool collides_check(const VoxelOctree<_N> &other) const {
    limit_check(other); // significantly slows down collision checking
    return collides(other);
  }

  bool collides(const VoxelOctree<_N> &other) const {
    return _data->collides(*(other._data));
  }

protected:
  // one in the given place, zeros everywhere else
  // for x, y, z within a block (each should be 0 <= x < 4)
  uint64_t bitmask(uint_fast8_t x, uint_fast8_t y, uint_fast8_t z) const {
    return uint64_t(1) << (x*16 + y*4 + z);
  }

  void domain_check(double x, double y, double z) const {
    if (x < _xmin || _xmax < x) {
      throw std::domain_error("x is out of the voxel dimensions");
    }
    if (y < _ymin || _ymax < y) {
      throw std::domain_error("y is out of the voxel dimensions");
    }
    if (z < _zmin || _zmax < z) {
      throw std::domain_error("z is out of the voxel dimensions");
    }
  }

  void limit_check(const VoxelOctree &other) const {
    double eps = std::numeric_limits<double>::epsilon();
    auto dbl_eq_check = [eps](const std::string &name, double val1, double val2) {
      if (std::abs(val1 - val2) >= eps) {
        throw std::domain_error(name + " does not match");
      }
    };
    dbl_eq_check("xmin", this->_xmin, other._xmin);
    dbl_eq_check("ymin", this->_ymin, other._ymin);
    dbl_eq_check("zmin", this->_zmin, other._zmin);
    dbl_eq_check("xmax", this->_xmax, other._xmax);
    dbl_eq_check("ymax", this->_ymax, other._ymax);
    dbl_eq_check("zmax", this->_zmax, other._zmax);
  }

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


#endif // VOXEL_OCTREE_H

