#ifndef CT_VOXEL_OCTREE_HXX
#define CT_VOXEL_OCTREE_HXX

#include "detail/TreeNode.h"

#include <algorithm>
#include <functional>

template <size_t _N>
CTVoxelOctree<_N>::CTVoxelOctree() : _data() {
  set_xlim(0.0, 1.0);
  set_ylim(0.0, 1.0);
  set_zlim(0.0, 1.0);
}

template <size_t _N>
bool CTVoxelOctree<_N>::operator== (const CTVoxelOctree<_N> &other) const {
  return _xmin == other._xmin
      && _ymin == other._ymin
      && _zmin == other._zmin
      && _xmax == other._xmax
      && _ymax == other._ymax
      && _zmax == other._zmax
      && _dx   == other._dx
      && _dy   == other._dy
      && _dz   == other._dz
      && _data == other._data;
}

template <size_t _N>
void CTVoxelOctree<_N>::copy_limits(const CTVoxelOctree<_N> &other) {
  this->_xmin = other._xmin;
  this->_xmax = other._xmax;
  this->_ymin = other._ymin;
  this->_ymax = other._ymax;
  this->_zmin = other._zmin;
  this->_zmax = other._zmax;
  this->_dx   = other._dx;
  this->_dy   = other._dy;
  this->_dz   = other._dz;
}

template <size_t _N>
CTVoxelOctree<_N> CTVoxelOctree<_N>::empty_copy() const {
  CTVoxelOctree<_N> copy();
  copy.copy_limits(*this);
  return copy;
}

template <size_t _N>
void CTVoxelOctree<_N>::set_xlim(double xmin, double xmax) {
  if (xmin >= xmax) {
    throw std::length_error("xlimits must be positive in size");
  }
  _xmin = xmin;
  _xmax = xmax;
  _dx = (xmax - xmin) / Nx();
}

template <size_t _N>
void CTVoxelOctree<_N>::set_ylim(double ymin, double ymax) {
  if (ymin >= ymax) {
    throw std::length_error("ylimits must be positive in size");
  }
  _ymin = ymin;
  _ymax = ymax;
  _dy = (ymax - ymin) / Ny();
}

template <size_t _N>
void CTVoxelOctree<_N>::set_zlim(double zmin, double zmax) {
  if (zmin >= zmax) {
    throw std::length_error("zlimits must be positive in size");
  }
  _zmin = zmin;
  _zmax = zmax;
  _dz = (zmax - zmin) / Nz();
}

template <size_t _N>
void CTVoxelOctree<_N>::set_xlim(const std::pair<double, double> &lim) {
  set_xlim(lim.first, lim.second);
}

template <size_t _N>
void CTVoxelOctree<_N>::set_ylim(const std::pair<double, double> &lim) {
  set_ylim(lim.first, lim.second);
}

template <size_t _N>
void CTVoxelOctree<_N>::set_zlim(const std::pair<double, double> &lim) {
  set_zlim(lim.first, lim.second);
}

template <size_t _N>
size_t CTVoxelOctree<_N>::nblocks() const { return _data.nblocks(); }

template <size_t _N>
bool CTVoxelOctree<_N>::is_in_domain(double x, double y, double z) const {
  return (_xmin <= x && x <= _xmax)
      && (_ymin <= y && y <= _ymax)
      && (_zmin <= z && z <= _zmax);
}

template <size_t _N>
uint64_t CTVoxelOctree<_N>::block(size_t bx, size_t by, size_t bz) const {
  return _data.block(bx, by, bz);
}

template <size_t _N>
void CTVoxelOctree<_N>::set_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  _data.set_block(bx, by, bz, value);
}

// returns old block type before unioning with value
template <size_t _N>
uint64_t CTVoxelOctree<_N>::union_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  if (value) {
    return _data.union_block(bx, by, bz, value);
  } else {
    return _data.block(bx, by, bz);
  }
}

template <size_t _N>
uint64_t CTVoxelOctree<_N>::intersect_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  return _data.intersect_block(bx, by, bz, value);
}

template <size_t _N>
uint64_t CTVoxelOctree<_N>::subtract_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  return _data.subtract_block(bx, by, bz, value);
}

template <size_t _N>
bool CTVoxelOctree<_N>::cell(size_t ix, size_t iy, size_t iz) const {
  auto b = block(ix / 4, iy / 4, iz / 4);
  return b && (b & bitmask(ix % 4, iy % 4, iz % 4));
}

// sets the cell's value
// returns true if the cell's value changed
template <size_t _N>
bool CTVoxelOctree<_N>::set_cell(size_t ix, size_t iy, size_t iz, bool value) {
  auto mask = bitmask(ix % 4, iy % 4, iz % 4);
  if (value) {
    auto oldval = _data.union_block(ix / 4, iy / 4, iz / 4, mask);
    return oldval & mask;
  } else {
    auto oldval = _data.intersect_block(ix / 4, iy / 4, iz / 4, ~mask);
    return oldval & ~mask;
  }
}

template <size_t _N>
uint64_t CTVoxelOctree<_N>::find_block(double x, double y, double z) const {
  auto [bx, by, bz] = find_block_idx(x, y, z);
  return block(bx, by, bz);
}

template <size_t _N>
std::tuple<size_t, size_t, size_t> CTVoxelOctree<_N>::find_block_idx(double x, double y, double z) const {
  domain_check(x, y, z);
  size_t ix = size_t((x - _xmin) / _dx);
  size_t iy = size_t((y - _ymin) / _dy);
  size_t iz = size_t((z - _zmin) / _dz);
  return {ix / 4, iy / 4, iz / 4};
}

template <size_t _N>
std::tuple<size_t, size_t, size_t> CTVoxelOctree<_N>::find_cell(double x, double y, double z) const {
  domain_check(x, y, z);
  size_t ix = size_t((x - _xmin) / _dx);
  size_t iy = size_t((y - _ymin) / _dy);
  size_t iz = size_t((z - _zmin) / _dz);
  return {ix, iy, iz};
}

template <size_t _N>
void CTVoxelOctree<_N>::add_point(double x, double y, double z) {
  auto [ix, iy, iz] = find_cell(x, y, z);
  set_cell(ix, iy, iz);
}

// TODO: easier algorithm is to go through each voxel, create an FCL AABB box
// TODO- and collision check it against the object to be added (for sphere,
// TODO- capsule, and mesh).  Slow, but effective

// sets sphere as occupied in voxel space, with center and radius specified
// adds any voxels that intersect or are inside of the sphere.
template <size_t _N>
void CTVoxelOctree<_N>::add_sphere(double x, double y, double z, double r) {
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
  //for (size_t ix = 0; ix < Nx(); ix++) {
  //  for (size_t iy = 0; iy < Ny(); iy++) {
  //    for (size_t iz = 0; iz < Nz(); iz++) {
  //      if (voxel_ctr_is_in_sphere(ix, iy, iz)) {
  //        this->set_cell(ix, iy, iz);
  //      }
  //    }
  //  }
  //}
  for (size_t bx = 0; bx < Nbx(); bx++) {
    for (size_t by = 0; by < Nby(); by++) {
      for (size_t bz = 0; bz < Nbz(); bz++) {
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
  //auto visited = std::make_unique<VoxelObject<_N, _N, _N>>();

  //auto check_push = [&frontier, &visited](size_t _ix, size_t _iy, size_t _iz) {
  //  if (!visited->cell(_ix, _iy, _iz) && _ix < _N && _iy < _N && _iz < _N) {
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

template <size_t _N>
void CTVoxelOctree<_N>::add_voxels(const CTVoxelOctree<_N> &other) {
  _data.union_tree(other._data);
}

template <size_t _N>
void CTVoxelOctree<_N>::remove_interior_slow_1() {
  const CTVoxelOctree<_N> copy(*this);
  for (size_t ix = 0; ix < Nx(); ix++) {
    for (size_t iy = 0; iy < Ny(); iy++) {
      for (size_t iz = 0; iz < Nz(); iz++) {
        bool curr   = copy.cell(ix, iy, iz);
        bool left   = (ix == 0)      || copy.cell(ix-1, iy, iz);
        bool right  = (ix == Nx()-1) || copy.cell(ix+1, iy, iz);
        bool front  = (iy == 0)      || copy.cell(ix, iy-1, iz);
        bool behind = (iy == Ny()-1) || copy.cell(ix, iy+1, iz);
        bool below  = (iz == 0)      || copy.cell(ix, iy, iz-1);
        bool above  = (iz == Nz()-1) || copy.cell(ix, iy, iz+1);
        if (curr && left && right && front && behind && below && above) {
          set_cell(ix, iy, iz, false);
        }
      }
    }
  }
}

/** For solid objects, removes the interior.
 *
 * Basically, this simply removes any occupied cells that are completely
 * surrounded by other occupied cells.  So, only leaves the occupied cells
 * that have an empty neighbor.  Neighbors are only the six directly adjacent
 * cells.
 *
 * For sparse voxel objects, this should reduce the number of occupied cells
 * and hopefully blocks, thus reducing memory.
 */
template <size_t _N>
void CTVoxelOctree<_N>::remove_interior() {
  auto copy = std::make_unique<detail::TreeNode<_N> const>(_data);

  // iterate over copy and modify _data
  auto visitor = [this, &copy](size_t bx, size_t by, size_t bz, uint64_t old_b) {

#define my_assert(val) if (!val) { throw std::runtime_error(#val); }
    my_assert(old_b == copy->block(bx, by, bz));

    const uint64_t full = ~uint64_t(0);
    auto new_b = old_b;
    const uint64_t left   = (bx <= 0)       ? full : copy->block(bx-1, by, bz);
    const uint64_t right  = (bx >= Nbx()-1) ? full : copy->block(bx+1, by, bz);
    const uint64_t front  = (by <= 0)       ? full : copy->block(bx, by-1, bz);
    const uint64_t behind = (by >= Nby()-1) ? full : copy->block(bx, by+1, bz);
    const uint64_t below  = (bz <= 0)       ? full : copy->block(bx, by, bz-1);
    const uint64_t above  = (bz >= Nbz()-1) ? full : copy->block(bx, by, bz+1);

    auto is_interior =
      [left, right, front, behind, below, above, old_b, this]
      (unsigned ix, unsigned iy, unsigned iz) {
        bool is_in = true;
        uint64_t mask = this->bitmask(ix, iy, iz);

        if (ix > 0) { mask |= this->bitmask(ix-1, iy, iz); }
        else { is_in = (is_in && bool(left & this->bitmask(3, iy, iz))); }

        if (ix < 3) { mask |= this->bitmask(ix+1, iy, iz); }
        else { is_in = (is_in && bool(right & this->bitmask(0, iy, iz))); }

        if (iy > 0) { mask |= this->bitmask(ix, iy-1, iz); }
        else { is_in = (is_in && bool(front & this->bitmask(ix, 3, iz))); }

        if (iy < 3) { mask |= this->bitmask(ix, iy+1, iz); }
        else { is_in = (is_in && bool(behind & this->bitmask(ix, 0, iz))); }

        if (iz > 0) { mask |= this->bitmask(ix, iy, iz-1); }
        else { is_in = (is_in && bool(below & this->bitmask(ix, iy, 3))); }

        if (iz < 3) { mask |= this->bitmask(ix, iy, iz+1); }
        else { is_in = (is_in && bool(above & this->bitmask(ix, iy, 0))); }

        is_in = (is_in && (mask == (old_b & mask)));
        return is_in;
      };

    for (unsigned ix = 0; ix < 4; ix++) {
      for (unsigned iy = 0; iy < 4; iy++) {
        for (unsigned iz = 0; iz < 4; iz++) {
          if (is_interior(ix, iy, iz)) {
            new_b &= ~bitmask(ix, iy, iz);
          }
        }
      }
    }

    // store this new block
    this->_data.set_block(bx, by, bz, new_b);
  };
  copy->visit_leaves(visitor);
}

template <size_t _N>
void CTVoxelOctree<_N>::remove_point(double x, double y, double z) {
  if (!is_in_domain(x, y, z)) { return; }
  auto [ix, iy, iz] = find_cell(x, y, z);
  set_cell(ix, iy, iz, false);
}

template <size_t _N>
void CTVoxelOctree<_N>::remove_voxels(const CTVoxelOctree<_N> &other) {
  _data.remove_tree(other._data);
}

template <size_t _N>
void CTVoxelOctree<_N>::intersect_voxels(const CTVoxelOctree<_N> &other) {
  _data.intersect_tree(other._data);
}

template <size_t _N>
void CTVoxelOctree<_N>::visit_leaves(
    const std::function<void(size_t, size_t, size_t, uint64_t)> &visitor) const
{
  _data.visit_leaves(visitor);
}

template <size_t _N>
bool CTVoxelOctree<_N>::collides_check(const CTVoxelOctree<_N> &other) const {
  limit_check(other);
  return collides(other);
}

template <size_t _N>
bool CTVoxelOctree<_N>::collides(const CTVoxelOctree<_N> &other) const {
  return _data.collides(other._data);
}

// one in the given place, zeros everywhere else
// for x, y, z within a block (each should be 0 <= x < 4)
template <size_t _N>
uint64_t CTVoxelOctree<_N>::bitmask(uint_fast8_t x, uint_fast8_t y, uint_fast8_t z) const {
  return uint64_t(1) << (x*16 + y*4 + z);
}

template <size_t _N>
void CTVoxelOctree<_N>::domain_check(double x, double y, double z) const {
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

template <size_t _N>
void CTVoxelOctree<_N>::limit_check(const CTVoxelOctree &other) const {
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

#endif // CT_VOXEL_OCTREE_HXX
