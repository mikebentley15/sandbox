#ifndef DERIVED_VOXEL_OCTREE_HXX
#define DERIVED_VOXEL_OCTREE_HXX

#include "DerivedVoxelOctree.h"

template <size_t _N>
DerivedVoxelOctree<_N>::DerivedVoxelOctree()
  : AbstractVoxelOctree()
  , _data(new detail::TreeNode<_N>())
{}

template <size_t _N>
DerivedVoxelOctree<_N>::DerivedVoxelOctree(const DerivedVoxelOctree<_N> &other)
  : AbstractVoxelOctree(other)
  , _data(new detail::TreeNode<_N>(*(other._data)))
  , _dx(other._dx), _dy(other._dy), _dz(other._dz)
{}

template <size_t _N>
DerivedVoxelOctree<_N>::DerivedVoxelOctree(DerivedVoxelOctree<_N> &&other)
  : AbstractVoxelOctree(std::move(other))
  , _data(std::move(other._data))
  , _dx(other._dx), _dy(other._dy), _dz(other._dz)
{}


template <size_t _N>
void DerivedVoxelOctree<_N>::set_xlim(double xmin, double xmax) {
  AbstractVoxelOctree::set_xlim(xmin, xmax);
  _dx = (xmax - xmin) / Nx();
}

template <size_t _N>
void DerivedVoxelOctree<_N>::set_ylim(double ymin, double ymax) {
  AbstractVoxelOctree::set_ylim(ymin, ymax);
  _dy = (ymax - ymin) / Ny();
}

template <size_t _N>
void DerivedVoxelOctree<_N>::set_zlim(double zmin, double zmax) {
  AbstractVoxelOctree::set_zlim(zmin, zmax);
  _dz = (zmax - zmin) / Nz();
}

template <size_t _N>
size_t DerivedVoxelOctree<_N>::nblocks() const {
  return _data->nblocks();
}

template <size_t _N>
uint64_t DerivedVoxelOctree<_N>::block(size_t bx, size_t by, size_t bz) const {
  return _data->block(bx, by, bz);
}

template <size_t _N>
void DerivedVoxelOctree<_N>::set_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  _data->set_block(bx, by, bz, value);
}

template <size_t _N>
uint64_t DerivedVoxelOctree<_N>::union_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  if (value) {
    return _data->union_block(bx, by, bz, value);
  } else {
    return _data->block(bx, by, bz);
  }
}

template <size_t _N>
uint64_t DerivedVoxelOctree<_N>::intersect_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  return _data->intersect_block(bx, by, bz, value);
}

template <size_t _N>
bool DerivedVoxelOctree<_N>::cell(size_t ix, size_t iy, size_t iz) const {
  auto b = block(ix / 4, iy / 4, iz / 4);
  return b && (b & bitmask(ix % 4, iy % 4, iz % 4));
}

template <size_t _N>
bool DerivedVoxelOctree<_N>::set_cell(size_t ix, size_t iy, size_t iz, bool value) {
  auto mask = bitmask(ix % 4, iy % 4, iz % 4);
  if (value) {
    auto oldval = this->union_block(ix / 4, iy / 4, iz / 4, mask);
    return oldval & mask;
  } else {
    auto oldval = this->intersect_block(ix / 4, iy / 4, iz / 4, ~mask);
    return oldval & ~mask;
  }
}

template <size_t _N>
std::tuple<size_t, size_t, size_t> DerivedVoxelOctree<_N>::find_block_idx(double x, double y, double z) const {
  domain_check(x, y, z);
  size_t ix = size_t((x - _xmin) / _dx);
  size_t iy = size_t((y - _ymin) / _dy);
  size_t iz = size_t((z - _zmin) / _dz);
  return {ix / 4, iy / 4, iz / 4};
}

template <size_t _N>
std::tuple<size_t, size_t, size_t> DerivedVoxelOctree<_N>::find_cell(double x, double y, double z) const {
  domain_check(x, y, z);
  size_t ix = size_t((x - _xmin) / _dx);
  size_t iy = size_t((y - _ymin) / _dy);
  size_t iz = size_t((z - _zmin) / _dz);
  return {ix, iy, iz};
}

template <size_t _N>
void DerivedVoxelOctree<_N>::add_sphere(double x, double y, double z, double r) {
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
  //auto visited = std::make_unique<VoxelObject<Nx, Ny, Nz>>();

  //auto check_push = [&frontier, &visited](size_t _ix, size_t _iy, size_t _iz) {
  //  if (!visited->cell(_ix, _iy, _iz) && _ix < Nx() && _iy < Ny() && _iz < Nz()) {
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
void DerivedVoxelOctree<_N>::remove_interior_slow_1() {
  const DerivedVoxelOctree<_N> copy(*this);
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

template <size_t _N>
void DerivedVoxelOctree<_N>::remove_interior() {
  auto copy = std::make_unique<detail::TreeNode<_N> const>(*_data);

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
    this->_data->set_block(bx, by, bz, new_b);
  };
  //copy->visit_leaves(visitor);
  copy->visit_leaves_2(visitor);
}

template <size_t _N>
bool DerivedVoxelOctree<_N>::collides_check(const AbstractVoxelOctree &other) const {
  const auto other_derived = dynamic_cast<const DerivedVoxelOctree<_N>*>(&other);
  if (other_derived == nullptr) {
    throw std::invalid_argument("passed in not the same type of DerivedVoxelOctree");
  }
  limit_check(*other_derived); // significantly slows down collision checking
  return _data->collides(*(other_derived->_data));
}

template <size_t _N>
bool DerivedVoxelOctree<_N>::collides(const AbstractVoxelOctree &other) const {
  const auto other_derived = dynamic_cast<const DerivedVoxelOctree<_N>*>(&other);
  if (other_derived == nullptr) {
    throw std::invalid_argument("passed in not the same type of DerivedVoxelOctree");
  }
  return _data->collides(*(other_derived->_data));
}

template <size_t _N>
uint64_t DerivedVoxelOctree<_N>::bitmask(uint_fast8_t x, uint_fast8_t y, uint_fast8_t z) const {
  return uint64_t(1) << (x*16 + y*4 + z);
}

template <size_t _N>
void DerivedVoxelOctree<_N>::domain_check(double x, double y, double z) const {
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
void DerivedVoxelOctree<_N>::limit_check(const DerivedVoxelOctree<_N> &other) const {
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

#endif // DERIVED_VOXEL_OCTREE_HXX
