#ifndef SPARSE_VOXEL_OBJECT_H
#define SPARSE_VOXEL_OBJECT_H

#include <iostream>
#include <limits>    // for std::numeric_limits
#include <map>       // for std::map
#include <stack>     // for std::stack
#include <stdexcept> // for std::length_error
#include <tuple>     // for std::tuple
#include <utility>   // for std::pair

#include <cassert> // for assert()
#include <cstddef> // for size_t
#include <cstdint> // for uint64_t

class SparseVoxelObject {
protected:

  // one in the given place, zeros everywhere else
  // for x, y, z within a block (each should be 0 <= x < 4)
  constexpr static
  uint64_t bitmask(uint_fast8_t x, uint_fast8_t y, uint_fast8_t z) {
    return uint64_t(1) << (x*16 + y*4 + z);
  }

  static
  void dimension_size_check(const std::string &name, size_t val) {
    if (val % 4 != 0) {
      throw std::invalid_argument(name + " is not a multiple of 4");
    }
  }

public:
  const size_t Nx;  // number of voxels in the x-direction
  const size_t Ny;  // number of voxels in the y-direction
  const size_t Nz;  // number of voxels in the z-direction
  const size_t N;   // number of voxels

  const size_t Nbx; // number of blocks in the x-direction
  const size_t Nby; // number of blocks in the y-direction
  const size_t Nbz; // number of blocks in the z-direction
  const size_t Nb;  // number of blocks

public:
  SparseVoxelObject(size_t _Nx, size_t _Ny, size_t _Nz)
    : Nx(_Nx), Ny(_Ny), Nz(_Nz), N(_Nx * _Ny * _Nz)
    , Nbx(_Nx/4), Nby(_Ny/4), Nbz(_Nz/4), Nb(_Nx * _Ny * _Nz / 64)
    , _data()
  {
    dimension_size_check("Nx", Nx);
    dimension_size_check("Ny", Ny);
    dimension_size_check("Nz", Nz);
    set_xlim(0.0, 1.0);
    set_ylim(0.0, 1.0);
    set_zlim(0.0, 1.0);
  }

  SparseVoxelObject(const SparseVoxelObject &other)  // copy
    : Nx(other.Nx), Ny(other.Ny), Nz(other.Nz), N(other.N)
    , Nbx(other.Nbx), Nby(other.Nby), Nbz(other.Nbz), Nb(other.Nb)
    , _data(other._data)
    , _xmin(other._xmin), _xmax(other._xmax)
    , _ymin(other._ymin), _ymax(other._ymax)
    , _zmin(other._zmin), _zmax(other._zmax)
    , _dx(other._dx), _dy(other._dy), _dz(other._dz)
  {}

  SparseVoxelObject(SparseVoxelObject &&other)  // move
    : Nx(other.Nx), Ny(other.Ny), Nz(other.Nz), N(other.N)
    , Nbx(other.Nbx), Nby(other.Nby), Nbz(other.Nbz), Nb(other.Nb)
    , _data(std::move(other._data))
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

  size_t nblocks() const { return _data.size(); }

  uint64_t block(size_t idx) const {
    auto iter = _data.find(idx);
    if (iter != _data.end()) {
      return iter->second;
    }
    return 0;
  }

  size_t block_idx(size_t bx, size_t by, size_t bz) const {
    return bz + Nbz*(by + Nby*bx);
  }

  uint64_t block(size_t bx, size_t by, size_t bz) const {
    return block(block_idx(bx, by, bz));
  }

  void set_block(size_t idx, uint64_t value) {
    if (value == 0) { _data.erase(idx); }
    else { _data[idx] = value; }
  }

  void set_block(size_t bx, size_t by, size_t bz, uint64_t value) {
    set_block(block_idx(bx, by, bz), value);
  }

  bool cell(size_t ix, size_t iy, size_t iz) const {
    auto b = block(ix / 4, iy / 4, iz / 4);
    return b && (b & bitmask(ix % 4, iy % 4, iz % 4));
  }

  // sets the cell's value
  // returns true if the cell's value changed
  bool set_cell(size_t ix, size_t iy, size_t iz, bool value = true) {
    auto idx = block_idx(ix / 4, iy / 4, iz / 4);
    auto &b = _data[idx];
    auto mask = bitmask(ix % 4, iy % 4, iz % 4);
    bool is_set = b & mask;
    if (value) {
      b |= mask;
    } else {
      b &= ~mask;
      if (!b) {
        _data.erase(idx);
      }
    }
    return is_set != value;
  }

  uint64_t find_block(double x, double y, double z) const {
    auto idx = find_block_idx(x, y, z);
    return block(idx);
  }

  size_t find_block_idx(double x, double y, double z) const {
    domain_check(x, y, z);
    size_t ix = size_t((x - _xmin) / _dx);
    size_t iy = size_t((y - _ymin) / _dy);
    size_t iz = size_t((z - _zmin) / _dz);
    return block_idx(ix / 4, iy / 4, iz / 4);
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
          if (b) { _data[block_idx(bx, by, bz)] |= b; }
        }
      }
    }
  }

  bool collides_check(const SparseVoxelObject &other) const {
    dimension_check(other);
    limit_check(other); // significantly slows down collision checking
    return collides(other);
  }

  bool collides(const SparseVoxelObject &other) const {
    auto ita = _data.begin();
    auto itb = other._data.begin();
    while (ita != _data.end() && itb != other._data.end()) {
      if (ita->first == itb->first) {
        if (ita->second & itb->second) {
          return true;
        }
      } else if (ita->first < itb->first) {
        ita++;
      } else {
        itb++;
      }
    }
    return false;
  }

  void remove_interior_slow_1() {
    const SparseVoxelObject copy(*this);
    for (size_t ix = 0; ix < Nx; ix++) {
      for (size_t iy = 0; iy < Ny; iy++) {
        for (size_t iz = 0; iz < Nz; iz++) {
          bool curr   = copy.cell(ix, iy, iz);
          bool left   = (ix == 0)    || copy.cell(ix-1, iy, iz);
          bool right  = (ix == Nx-1) || copy.cell(ix+1, iy, iz);
          bool front  = (iy == 0)    || copy.cell(ix, iy-1, iz);
          bool behind = (iy == Ny-1) || copy.cell(ix, iy+1, iz);
          bool below  = (iz == 0)    || copy.cell(ix, iy, iz-1);
          bool above  = (iz == Nz-1) || copy.cell(ix, iy, iz+1);
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
  void remove_interior() {
    decltype(_data) new_data;
    const uint64_t full = ~uint64_t(0);

    for (const auto &[idx, old_b] : _data) {
      const size_t bx = idx / (Nby * Nbz);
      const size_t by = (idx / Nbz) % Nby;
      const size_t bz = idx % Nbz;

      //auto my_assert = [](bool val) {
      //  if (!val) {
      //    throw std::runtime_error("assertion failed");
      //  }
      //};
      uint64_t new_b = old_b;

      // get the neighbors
      const uint64_t left   = (bx <= 0)     ? full : block(idx - Nby*Nbz);
      const uint64_t right  = (bx >= Nbx-1) ? full : block(idx + Nby*Nbz);
      const uint64_t front  = (by <= 0)     ? full : block(idx - Nbz);
      const uint64_t behind = (by >= Nby-1) ? full : block(idx + Nbz);
      const uint64_t below  = (bz <= 0)     ? full : block(idx - 1);
      const uint64_t above  = (bz >= Nbz-1) ? full : block(idx + 1);

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

      auto update_cell =
        [&is_interior, &new_b] (unsigned ix, unsigned iy, unsigned iz) {
          if (is_interior(ix, iy, iz)) {
            new_b &= ~bitmask(ix, iy, iz);
          }
        };

      // want to check each of the 64 bits in their own optimized way

      update_cell(0, 0, 0);
      update_cell(0, 0, 1);
      update_cell(0, 0, 2);
      update_cell(0, 0, 3);
      update_cell(0, 1, 0);
      update_cell(0, 1, 1);
      update_cell(0, 1, 2);
      update_cell(0, 1, 3);
      update_cell(0, 2, 0);
      update_cell(0, 2, 1);
      update_cell(0, 2, 2);
      update_cell(0, 2, 3);
      update_cell(0, 3, 0);
      update_cell(0, 3, 1);
      update_cell(0, 3, 2);
      update_cell(0, 3, 3);
      update_cell(1, 0, 0);
      update_cell(1, 0, 1);
      update_cell(1, 0, 2);
      update_cell(1, 0, 3);
      update_cell(1, 1, 0);
      update_cell(1, 1, 1);
      update_cell(1, 1, 2);
      update_cell(1, 1, 3);
      update_cell(1, 2, 0);
      update_cell(1, 2, 1);
      update_cell(1, 2, 2);
      update_cell(1, 2, 3);
      update_cell(1, 3, 0);
      update_cell(1, 3, 1);
      update_cell(1, 3, 2);
      update_cell(1, 3, 3);
      update_cell(2, 0, 0);
      update_cell(2, 0, 1);
      update_cell(2, 0, 2);
      update_cell(2, 0, 3);
      update_cell(2, 1, 0);
      update_cell(2, 1, 1);
      update_cell(2, 1, 2);
      update_cell(2, 1, 3);
      update_cell(2, 2, 0);
      update_cell(2, 2, 1);
      update_cell(2, 2, 2);
      update_cell(2, 2, 3);
      update_cell(2, 3, 0);
      update_cell(2, 3, 1);
      update_cell(2, 3, 2);
      update_cell(2, 3, 3);
      update_cell(3, 0, 0);
      update_cell(3, 0, 1);
      update_cell(3, 0, 2);
      update_cell(3, 0, 3);
      update_cell(3, 1, 0);
      update_cell(3, 1, 1);
      update_cell(3, 1, 2);
      update_cell(3, 1, 3);
      update_cell(3, 2, 0);
      update_cell(3, 2, 1);
      update_cell(3, 2, 2);
      update_cell(3, 2, 3);
      update_cell(3, 3, 0);
      update_cell(3, 3, 1);
      update_cell(3, 3, 2);
      update_cell(3, 3, 3);

      ////
      //// for the eight interior cells in the middle (zero neighboring blocks)
      ////

      //constexpr uint64_t mask_check_111 = bitmask(1, 1, 1)
      //  | bitmask(0, 1, 1) | bitmask(1, 0, 1) | bitmask(1, 1, 0)
      //  | bitmask(2, 1, 1) | bitmask(1, 2, 1) | bitmask(1, 1, 2);
      //if ((old_b & mask_check_111) == mask_check_111) { new_b &= ~bitmask(1, 1, 1); }

      //constexpr uint64_t mask_check_112 = bitmask(1, 1, 2)
      //  | bitmask(0, 1, 1) | bitmask(1, 0, 1) | bitmask(1, 1, 1)
      //  | bitmask(2, 1, 1) | bitmask(1, 2, 1) | bitmask(1, 1, 3);
      //if ((old_b & mask_check_112) == mask_check_112) { new_b &= ~bitmask(1, 1, 2); }

      //constexpr uint64_t mask_check_121 = bitmask(1, 2, 1)
      //  | bitmask(0, 2, 1) | bitmask(1, 1, 1) | bitmask(1, 2, 0)
      //  | bitmask(2, 2, 1) | bitmask(1, 3, 1) | bitmask(1, 2, 2);
      //if ((old_b & mask_check_121) == mask_check_121) { new_b &= ~bitmask(1, 2, 1); }

      //constexpr uint64_t mask_check_122 = bitmask(1, 2, 2)
      //  | bitmask(0, 2, 2) | bitmask(1, 1, 2) | bitmask(1, 2, 1)
      //  | bitmask(2, 2, 2) | bitmask(1, 3, 2) | bitmask(1, 2, 3);
      //if ((old_b & mask_check_122) == mask_check_122) { new_b &= ~bitmask(1, 2, 2); }

      //constexpr uint64_t mask_check_211 = bitmask(2, 1, 1)
      //  | bitmask(1, 1, 1) | bitmask(2, 0, 1) | bitmask(2, 1, 0)
      //  | bitmask(3, 1, 1) | bitmask(2, 2, 1) | bitmask(2, 1, 2);
      //if ((old_b & mask_check_211) == mask_check_211) { new_b &= ~bitmask(2, 1, 1); }

      //constexpr uint64_t mask_check_212 = bitmask(2, 1, 2)
      //  | bitmask(1, 1, 2) | bitmask(2, 0, 2) | bitmask(2, 1, 1)
      //  | bitmask(3, 1, 2) | bitmask(2, 2, 2) | bitmask(2, 1, 3);
      //if ((old_b & mask_check_212) == mask_check_212) { new_b &= ~bitmask(2, 1, 2); }

      //constexpr uint64_t mask_check_221 = bitmask(2, 2, 1)
      //  | bitmask(1, 2, 1) | bitmask(2, 1, 1) | bitmask(2, 2, 0)
      //  | bitmask(3, 2, 1) | bitmask(2, 3, 1) | bitmask(2, 2, 2);
      //if ((old_b & mask_check_221) == mask_check_221) { new_b &= ~bitmask(2, 2, 1); }

      //constexpr uint64_t mask_check_222 = bitmask(2, 2, 2)
      //  | bitmask(1, 2, 2) | bitmask(2, 1, 2) | bitmask(2, 2, 1)
      //  | bitmask(3, 2, 2) | bitmask(2, 3, 2) | bitmask(2, 2, 3);
      //if ((old_b & mask_check_222 == mask_check_222)) { new_b &= ~bitmask(2, 2, 2); }

      ////
      //// for the eight corners (three neighboring blocks)
      ////

      //constexpr uint64_t mask_check_000 = bitmask(0, 0, 0)
      //  | bitmask(1, 0, 0) | bitmask(0, 1, 0) | bitmask(0, 0, 1);
      //if ((old_b & mask_check_000) == mask_check_000
      //    && (left   & bitmask(3, 0, 0))
      //    && (front  & bitmask(0, 3, 0))
      //    && (below  & bitmask(0, 0, 3)))
      //{ new_b &= ~bitmask(0, 0, 0); }

      //constexpr uint64_t mask_check_003 = bitmask(0, 0, 3)
      //  | bitmask(1, 0, 3) | bitmask(0, 1, 3) | bitmask(0, 0, 2);
      //if ((old_b & mask_check_003) == mask_check_003
      //    && (left   & bitmask(3, 0, 3))
      //    && (front  & bitmask(0, 3, 3))
      //    && (above  & bitmask(0, 0, 0)))
      //{ new_b &= ~bitmask(0, 0, 3); }

      //constexpr uint64_t mask_check_030 = bitmask(0, 3, 0)
      //  | bitmask(1, 3, 0) | bitmask(0, 2, 0) | bitmask(0, 3, 1);
      //if ((old_b & mask_check_030) == mask_check_030
      //    && (left   & bitmask(3, 3, 0))
      //    && (behind & bitmask(0, 0, 0))
      //    && (below  & bitmask(0, 3, 3)))
      //{ new_b &= ~bitmask(0, 3, 0); }

      //constexpr uint64_t mask_check_033 = bitmask(0, 3, 3)
      //  | bitmask(1, 3, 3) | bitmask(0, 2, 3) | bitmask(0, 3, 2);
      //if ((old_b & mask_check_033) == mask_check_033
      //    && (left   & bitmask(3, 3, 3))
      //    && (behind & bitmask(0, 0, 3))
      //    && (above  & bitmask(0, 3, 0)))
      //{ new_b &= ~bitmask(0, 3, 3); }

      //constexpr uint64_t mask_check_300 = bitmask(3, 0, 0)
      //  | bitmask(2, 0, 0) | bitmask(3, 1, 0) | bitmask(3, 0, 1);
      //if ((old_b & mask_check_300) == mask_check_300
      //    && (right  & bitmask(0, 0, 0))
      //    && (front  & bitmask(3, 3, 0))
      //    && (below  & bitmask(3, 0, 3)))
      //{ new_b &= ~bitmask(3, 0, 0); }

      //constexpr uint64_t mask_check_303 = bitmask(3, 0, 3)
      //  | bitmask(2, 0, 3) | bitmask(3, 1, 3) | bitmask(3, 0, 2);
      //if ((old_b & mask_check_303) == mask_check_303
      //    && (right  & bitmask(0, 0, 3))
      //    && (front  & bitmask(3, 3, 3))
      //    && (above  & bitmask(3, 0, 0)))
      //{ new_b &= ~bitmask(3, 0, 3); }

      //constexpr uint64_t mask_check_330 = bitmask(3, 3, 0)
      //  | bitmask(2, 3, 0) | bitmask(3, 2, 0) | bitmask(3, 3, 1);
      //if ((old_b & mask_check_330) == mask_check_330
      //    && (right  & bitmask(0, 3, 0))
      //    && (behind & bitmask(3, 0, 0))
      //    && (below  & bitmask(3, 3, 3)))
      //{ new_b &= ~bitmask(3, 3, 0); }

      //constexpr uint64_t mask_check_333 = bitmask(3, 3, 3)
      //  | bitmask(2, 3, 3) | bitmask(3, 2, 3) | bitmask(3, 3, 2);
      //if ((old_b & mask_check_333) == mask_check_333
      //    && (right  & bitmask(0, 3, 3))
      //    && (behind & bitmask(3, 0, 3))
      //    && (above  & bitmask(3, 3, 0)))
      //{ new_b &= ~bitmask(3, 3, 3); }

      ////
      //// for the 24 edges (two neighboring blocks)
      ////

      //constexpr uint64_t mask_check_001 = bitmask(0, 0, 1)
      //  | bitmask(0, 0, 2) | bitmask(0, 1, 1) | bitmask(1, 0, 1)
      //  | bitmask(0, 0, 0);
      //if ((old_b & mask_check_001) == mask_check_001
      //    && (left   & bitmask(3, 0, 1))
      //    && (front  & bitmask(0, 3, 1)))
      //{ new_b &= ~bitmask(0, 0, 1); }

      //constexpr uint64_t mask_check_002 = bitmask(0, 0, 2)
      //  | bitmask(0, 0, 3) | bitmask(0, 1, 2) | bitmask(1, 0, 2)
      //  | bitmask(0, 0, 1);
      //if ((old_b & mask_check_002) == mask_check_002
      //    && (left   & bitmask(3, 0, 2))
      //    && (front  & bitmask(0, 3, 2)))
      //{ new_b &= ~bitmask(0, 0, 2); }

      //constexpr uint64_t mask_check_031 = bitmask(0, 3, 1)
      //  | bitmask(0, 3, 2) | bitmask(0, 2, 1) | bitmask(1, 3, 1)
      //  | bitmask(0, 3, 0);
      //if ((old_b & mask_check_031) == mask_check_031
      //    && (left   & bitmask(3, 3, 1))
      //    && (behind & bitmask(0, 0, 1)))
      //{ new_b &= ~bitmask(0, 3, 1); }

      //constexpr uint64_t mask_check_032 = bitmask(0, 3, 2)
      //  | bitmask(0, 3, 3) | bitmask(0, 2, 2) | bitmask(1, 3, 2)
      //  | bitmask(0, 3, 1);
      //if ((old_b & mask_check_032) == mask_check_032
      //    && (left   & bitmask(3, 3, 2))
      //    && (behind & bitmask(0, 0, 2)))
      //{ new_b &= ~bitmask(0, 3, 2); }

      //constexpr uint64_t mask_check_301 = bitmask(3, 0, 1)
      //  | bitmask(3, 0, 2) | bitmask(3, 1, 1) | bitmask(2, 0, 1)
      //  | bitmask(3, 0, 0);
      //if ((old_b & mask_check_301) == mask_check_301
      //    && (right  & bitmask(0, 0, 1))
      //    && (front  & bitmask(3, 3, 1)))
      //{ new_b &= ~bitmask(3, 0, 1); }

      //constexpr uint64_t mask_check_302 = bitmask(3, 0, 2)
      //  | bitmask(3, 0, 3) | bitmask(3, 1, 2) | bitmask(2, 0, 2)
      //  | bitmask(3, 0, 1);
      //if ((old_b & mask_check_302) == mask_check_302
      //    && (right  & bitmask(0, 0, 2))
      //    && (front  & bitmask(3, 3, 2)))
      //{ new_b &= ~bitmask(3, 0, 2); }

      //constexpr uint64_t mask_check_331 = bitmask(3, 3, 1)
      //  | bitmask(3, 3, 2) | bitmask(3, 2, 1) | bitmask(2, 3, 1)
      //  | bitmask(3, 3, 0);
      //if ((old_b & mask_check_331) == mask_check_331
      //    && (right  & bitmask(0, 3, 1))
      //    && (behind & bitmask(3, 0, 1)))
      //{ new_b &= ~bitmask(3, 3, 1); }

      //constexpr uint64_t mask_check_332 = bitmask(3, 3, 2)
      //  | bitmask(3, 3, 3) | bitmask(3, 2, 2) | bitmask(2, 3, 2)
      //  | bitmask(3, 3, 1);
      //if ((old_b & mask_check_332) == mask_check_332
      //    && (right  & bitmask(0, 3, 2))
      //    && (behind & bitmask(3, 0, 2)))
      //{ new_b &= ~bitmask(3, 3, 2); }

      //constexpr uint64_t mask_check_010 = bitmask(0, 1, 0)
      //  | bitmask(0, 1, 1) | bitmask(0, 2, 0) | bitmask(1, 1, 0)
      //  | bitmask(0, 0, 0);
      //if ((old_b & mask_check_010) == mask_check_010
      //    && (left   & bitmask(3, 1, 0))
      //    && (below  & bitmask(0, 1, 3)))
      //{ new_b &= ~bitmask(0, 1, 0); }

      //constexpr uint64_t mask_check_020 = bitmask(0, 2, 0)
      //  | bitmask(0, 2, 1) | bitmask(0, 3, 0) | bitmask(1, 2, 0)
      //  | bitmask(0, 1, 0);
      //if ((old_b & mask_check_020) == mask_check_020
      //    && (left   & bitmask(3, 2, 0))
      //    && (below  & bitmask(0, 2, 3)))
      //{ new_b &= ~bitmask(0, 2, 0); }

      //constexpr uint64_t mask_check_013 = bitmask(0, 1, 3)
      //  | bitmask(0, 1, 2) | bitmask(0, 2, 3) | bitmask(1, 1, 3)
      //  | bitmask(0, 0, 3);
      //if ((old_b & mask_check_013) == mask_check_013
      //    && (left   & bitmask(3, 1, 3))
      //    && (above  & bitmask(0, 1, 0)))
      //{ new_b &= ~bitmask(0, 1, 3); }

      //constexpr uint64_t mask_check_023 = bitmask(0, 2, 3)
      //  | bitmask(0, 2, 2) | bitmask(0, 3, 3) | bitmask(1, 2, 3)
      //  | bitmask(0, 1, 3);
      //if ((old_b & mask_check_023) == mask_check_023
      //    && (left   & bitmask(3, 2, 3))
      //    && (above  & bitmask(0, 2, 0)))
      //{ new_b &= ~bitmask(0, 2, 3); }

      //constexpr uint64_t mask_check_310 = bitmask(3, 1, 0)
      //  | bitmask(3, 1, 1) | bitmask(3, 2, 0) | bitmask(2, 1, 0)
      //  | bitmask(3, 0, 0);
      //if ((old_b & mask_check_310) == mask_check_310
      //    && (right  & bitmask(0, 1, 0))
      //    && (below  & bitmask(3, 1, 3)))
      //{ new_b &= ~bitmask(3, 1, 0); }

      //constexpr uint64_t mask_check_320 = bitmask(3, 2, 0)
      //  | bitmask(3, 2, 1) | bitmask(3, 3, 0) | bitmask(2, 2, 0)
      //  | bitmask(3, 1, 0);
      //if ((old_b & mask_check_320) == mask_check_320
      //    && (right  & bitmask(0, 2, 0))
      //    && (below  & bitmask(3, 2, 3)))
      //{ new_b &= ~bitmask(3, 2, 0); }

      //constexpr uint64_t mask_check_313 = bitmask(3, 1, 3)
      //  | bitmask(3, 1, 2) | bitmask(3, 2, 3) | bitmask(2, 1, 3)
      //  | bitmask(3, 0, 3);
      //if ((old_b & mask_check_313) == mask_check_313
      //    && (right  & bitmask(0, 1, 3))
      //    && (above  & bitmask(3, 1, 0)))
      //{ new_b &= ~bitmask(3, 1, 3); }

      //constexpr uint64_t mask_check_323 = bitmask(3, 2, 3)
      //  | bitmask(3, 2, 2) | bitmask(3, 3, 3) | bitmask(2, 2, 3)
      //  | bitmask(3, 1, 3);
      //if ((old_b & mask_check_323) == mask_check_323
      //    && (right  & bitmask(0, 2, 3))
      //    && (above  & bitmask(3, 2, 0)))
      //{ new_b &= ~bitmask(3, 2, 3); }

      //constexpr uint64_t mask_check_100 = bitmask(1, 0, 0)
      //  | bitmask(1, 0, 1) | bitmask(1, 1, 0) | bitmask(2, 0, 0)
      //  | bitmask(0, 0, 0);
      //if ((old_b & mask_check_100) == mask_check_100
      //    && (front  & bitmask(1, 3, 0))
      //    && (below  & bitmask(1, 0, 3)))
      //{ new_b &= ~bitmask(1, 0, 0); }

      //constexpr uint64_t mask_check_200 = bitmask(2, 0, 0)
      //  | bitmask(2, 0, 1) | bitmask(2, 1, 0) | bitmask(3, 0, 0)
      //  | bitmask(1, 0, 0);
      //if ((old_b & mask_check_200) == mask_check_200
      //    && (front  & bitmask(2, 3, 0))
      //    && (below  & bitmask(2, 0, 3)))
      //{ new_b &= ~bitmask(2, 0, 0); }

      //constexpr uint64_t mask_check_103 = bitmask(1, 0, 3)
      //  | bitmask(1, 0, 2) | bitmask(1, 1, 3) | bitmask(2, 0, 3)
      //  | bitmask(0, 0, 3);
      //if ((old_b & mask_check_103) == mask_check_103
      //    && (front  & bitmask(1, 3, 3))
      //    && (above  & bitmask(1, 0, 0)))
      //{ new_b &= ~bitmask(1, 0, 3); }

      //constexpr uint64_t mask_check_203 = bitmask(2, 0, 3)
      //  | bitmask(2, 0, 2) | bitmask(2, 1, 3) | bitmask(3, 0, 3)
      //  | bitmask(1, 0, 3);
      //if ((old_b & mask_check_203) == mask_check_203
      //    && (front  & bitmask(2, 3, 3))
      //    && (above  & bitmask(2, 0, 0)))
      //{ new_b &= ~bitmask(2, 0, 3); }

      //constexpr uint64_t mask_check_130 = bitmask(1, 3, 0)
      //  | bitmask(1, 3, 1) | bitmask(1, 2, 0) | bitmask(2, 3, 0)
      //  | bitmask(0, 3, 0);
      //if ((old_b & mask_check_130) == mask_check_130
      //    && (behind & bitmask(1, 0, 0))
      //    && (below  & bitmask(1, 3, 3)))
      //{ new_b &= ~bitmask(1, 3, 0); }

      //constexpr uint64_t mask_check_230 = bitmask(2, 3, 0)
      //  | bitmask(2, 3, 1) | bitmask(2, 2, 0) | bitmask(3, 3, 0)
      //  | bitmask(1, 3, 0);
      //if ((old_b & mask_check_230) == mask_check_230
      //    && (behind & bitmask(2, 0, 0))
      //    && (below  & bitmask(2, 3, 3)))
      //{ new_b &= ~bitmask(2, 3, 0); }

      //constexpr uint64_t mask_check_133 = bitmask(1, 3, 3)
      //  | bitmask(1, 3, 2) | bitmask(1, 2, 3) | bitmask(2, 3, 3)
      //  | bitmask(0, 3, 3);
      //if ((old_b & mask_check_133) == mask_check_133
      //    && (behind & bitmask(1, 0, 3))
      //    && (above  & bitmask(1, 3, 0)))
      //{ new_b &= ~bitmask(1, 3, 3); }

      //constexpr uint64_t mask_check_233 = bitmask(2, 3, 3)
      //  | bitmask(2, 3, 2) | bitmask(2, 2, 3) | bitmask(3, 3, 3)
      //  | bitmask(1, 3, 3);
      //if ((old_b & mask_check_233) == mask_check_233
      //    && (behind & bitmask(2, 0, 3))
      //    && (above  & bitmask(2, 3, 0)))
      //{ new_b &= ~bitmask(2, 3, 3); }

      ////
      //// for the 24 middle side pieces (one neighboring block)
      ////

      //constexpr uint64_t mask_check_011 = bitmask(0, 1, 1)
      //  | bitmask(0, 1, 2) | bitmask(0, 2, 1) | bitmask(1, 1, 1)
      //  | bitmask(0, 1, 0) | bitmask(0, 0, 1);
      //if ((old_b & mask_check_011) == mask_check_011
      //    && (left   & bitmask(3, 1, 1)))
      //{ new_b &= ~bitmask(0, 1, 1); }

      //constexpr uint64_t mask_check_012 = bitmask(0, 1, 2)
      //  | bitmask(0, 1, 3) | bitmask(0, 2, 2) | bitmask(1, 1, 2)
      //  | bitmask(0, 1, 1) | bitmask(0, 0, 2);
      //if ((old_b & mask_check_012) == mask_check_012
      //    && (left   & bitmask(3, 1, 2)))
      //{ new_b &= ~bitmask(0, 1, 2); }

      //constexpr uint64_t mask_check_021 = bitmask(0, 2, 1)
      //  | bitmask(0, 2, 2) | bitmask(0, 3, 1) | bitmask(1, 2, 1)
      //  | bitmask(0, 2, 0) | bitmask(0, 1, 1);
      //if ((old_b & mask_check_021) == mask_check_021
      //    && (left   & bitmask(3, 2, 1)))
      //{ new_b &= ~bitmask(0, 2, 1); }

      //constexpr uint64_t mask_check_022 = bitmask(0, 2, 2)
      //  | bitmask(0, 2, 3) | bitmask(0, 3, 2) | bitmask(1, 2, 2)
      //  | bitmask(0, 2, 1) | bitmask(0, 1, 2);
      //if ((old_b & mask_check_022) == mask_check_022
      //    && (left   & bitmask(3, 2, 2)))
      //{ new_b &= ~bitmask(0, 2, 2); }

      //constexpr uint64_t mask_check_311 = bitmask(3, 1, 1)
      //  | bitmask(3, 1, 2) | bitmask(3, 2, 1) | bitmask(2, 1, 1)
      //  | bitmask(3, 1, 0) | bitmask(3, 0, 1);
      //if ((old_b & mask_check_311) == mask_check_311
      //    && (right  & bitmask(0, 1, 1)))
      //{ new_b &= ~bitmask(3, 1, 1); }

      //constexpr uint64_t mask_check_312 = bitmask(3, 1, 2)
      //  | bitmask(3, 1, 3) | bitmask(3, 2, 2) | bitmask(2, 1, 2)
      //  | bitmask(3, 1, 1) | bitmask(3, 0, 2);
      //if ((old_b & mask_check_312) == mask_check_312
      //    && (right  & bitmask(0, 1, 2)))
      //{ new_b &= ~bitmask(3, 1, 2); }

      //constexpr uint64_t mask_check_321 = bitmask(3, 2, 1)
      //  | bitmask(3, 2, 2) | bitmask(3, 3, 1) | bitmask(2, 2, 1)
      //  | bitmask(3, 2, 0) | bitmask(3, 1, 1);
      //if ((old_b & mask_check_321) == mask_check_321
      //    && (right  & bitmask(0, 2, 1)))
      //{ new_b &= ~bitmask(3, 2, 1); }

      //constexpr uint64_t mask_check_322 = bitmask(3, 2, 2)
      //  | bitmask(3, 2, 3) | bitmask(3, 3, 2) | bitmask(2, 2, 2)
      //  | bitmask(3, 2, 1) | bitmask(3, 1, 2);
      //if ((old_b & mask_check_322) == mask_check_322
      //    && (right  & bitmask(0, 2, 2)))
      //{ new_b &= ~bitmask(3, 2, 2); }

      //constexpr uint64_t mask_check_101 = bitmask(1, 0, 1)
      //  | bitmask(1, 0, 2) | bitmask(1, 1, 1) | bitmask(2, 0, 1)
      //  | bitmask(1, 0, 0) | bitmask(0, 0, 1);
      //if ((old_b & mask_check_101) == mask_check_101
      //    && (front  & bitmask(1, 3, 1)))
      //{ new_b &= ~bitmask(1, 0, 1); }

      //constexpr uint64_t mask_check_102 = bitmask(1, 0, 2)
      //  | bitmask(1, 0, 3) | bitmask(1, 1, 2) | bitmask(2, 0, 2)
      //  | bitmask(1, 0, 1) | bitmask(0, 0, 2);
      //if ((old_b & mask_check_102) == mask_check_102
      //    && (front  & bitmask(1, 3, 2)))
      //{ new_b &= ~bitmask(1, 0, 2); }

      //constexpr uint64_t mask_check_201 = bitmask(2, 0, 1)
      //  | bitmask(2, 0, 2) | bitmask(2, 1, 1) | bitmask(3, 0, 1)
      //  | bitmask(2, 0, 0) | bitmask(1, 0, 1);
      //if ((old_b & mask_check_201) == mask_check_201
      //    && (front  & bitmask(2, 3, 1)))
      //{ new_b &= ~bitmask(2, 0, 1); }

      //constexpr uint64_t mask_check_202 = bitmask(2, 0, 2)
      //  | bitmask(2, 0, 3) | bitmask(2, 1, 2) | bitmask(3, 0, 2)
      //  | bitmask(2, 0, 1) | bitmask(1, 0, 2);
      //if ((old_b & mask_check_202) == mask_check_202
      //    && (front  & bitmask(2, 3, 2)))
      //{ new_b &= ~bitmask(2, 0, 2); }

      //constexpr uint64_t mask_check_131 = bitmask(1, 3, 1)
      //  | bitmask(1, 3, 2) | bitmask(1, 2, 1) | bitmask(2, 3, 1)
      //  | bitmask(1, 3, 0) | bitmask(0, 3, 1);
      //if ((old_b & mask_check_131) == mask_check_131
      //    && (behind & bitmask(1, 0, 1)))
      //{ new_b &= ~bitmask(1, 3, 1); }

      //constexpr uint64_t mask_check_132 = bitmask(1, 3, 2)
      //  | bitmask(1, 3, 3) | bitmask(1, 2, 2) | bitmask(2, 3, 2)
      //  | bitmask(1, 3, 1) | bitmask(0, 3, 2);
      //if ((old_b & mask_check_132) == mask_check_132
      //    && (behind & bitmask(1, 0, 2)))
      //{ new_b &= ~bitmask(1, 3, 2); }

      //constexpr uint64_t mask_check_231 = bitmask(2, 3, 1)
      //  | bitmask(2, 3, 2) | bitmask(2, 2, 1) | bitmask(3, 3, 1)
      //  | bitmask(2, 3, 0) | bitmask(1, 3, 1);
      //if ((old_b & mask_check_231) == mask_check_231
      //    && (behind & bitmask(2, 0, 1)))
      //{ new_b &= ~bitmask(2, 3, 1); }

      //constexpr uint64_t mask_check_232 = bitmask(2, 3, 2)
      //  | bitmask(2, 3, 3) | bitmask(2, 2, 2) | bitmask(3, 3, 2)
      //  | bitmask(2, 3, 1) | bitmask(1, 3, 2);
      //if ((old_b & mask_check_232) == mask_check_232
      //    && (behind & bitmask(2, 0, 2)))
      //{ new_b &= ~bitmask(2, 3, 2); }

      //constexpr uint64_t mask_check_110 = bitmask(1, 1, 0)
      //  | bitmask(1, 1, 1) | bitmask(1, 2, 0) | bitmask(2, 1, 0)
      //  | bitmask(1, 0, 0) | bitmask(0, 1, 0);
      //if ((old_b & mask_check_110) == mask_check_110
      //    && (below  & bitmask(1, 1, 3)))
      //{ new_b &= ~bitmask(1, 1, 0); }

      //constexpr uint64_t mask_check_120 = bitmask(1, 2, 0)
      //  | bitmask(1, 2, 1) | bitmask(1, 3, 0) | bitmask(2, 2, 0)
      //  | bitmask(1, 1, 0) | bitmask(0, 2, 0);
      //if ((old_b & mask_check_120) == mask_check_120
      //    && (below  & bitmask(1, 2, 3)))
      //{ new_b &= ~bitmask(1, 2, 0); }

      //constexpr uint64_t mask_check_210 = bitmask(2, 1, 0)
      //  | bitmask(2, 1, 1) | bitmask(2, 2, 0) | bitmask(3, 1, 0)
      //  | bitmask(2, 0, 0) | bitmask(1, 1, 0);
      //if ((old_b & mask_check_210) == mask_check_210
      //    && (below  & bitmask(2, 1, 3)))
      //{ new_b &= ~bitmask(2, 1, 0); }

      //constexpr uint64_t mask_check_220 = bitmask(2, 2, 0)
      //  | bitmask(2, 2, 1) | bitmask(2, 3, 0) | bitmask(3, 2, 0)
      //  | bitmask(2, 1, 0) | bitmask(1, 2, 0);
      //if ((old_b & mask_check_220) == mask_check_220
      //    && (below  & bitmask(2, 2, 3)))
      //{ new_b &= ~bitmask(2, 2, 0); }

      //constexpr uint64_t mask_check_113 = bitmask(1, 1, 3)
      //  | bitmask(1, 1, 2) | bitmask(1, 2, 3) | bitmask(2, 1, 3)
      //  | bitmask(1, 0, 3) | bitmask(0, 1, 3);
      //if ((old_b & mask_check_113) == mask_check_113
      //    && (above  & bitmask(1, 1, 0)))
      //{ new_b &= ~bitmask(1, 1, 3); }

      //constexpr uint64_t mask_check_123 = bitmask(1, 2, 3)
      //  | bitmask(1, 2, 2) | bitmask(1, 3, 3) | bitmask(2, 2, 3)
      //  | bitmask(1, 1, 3) | bitmask(0, 2, 3);
      //if ((old_b & mask_check_123) == mask_check_123
      //    && (above  & bitmask(1, 2, 0)))
      //{ new_b &= ~bitmask(1, 2, 3); }

      //constexpr uint64_t mask_check_213 = bitmask(2, 1, 3)
      //  | bitmask(2, 1, 2) | bitmask(2, 2, 3) | bitmask(3, 1, 3)
      //  | bitmask(2, 0, 3) | bitmask(1, 1, 3);
      //if ((old_b & mask_check_213) == mask_check_213
      //    && (above  & bitmask(2, 1, 0)))
      //{ new_b &= ~bitmask(2, 1, 3); }

      //constexpr uint64_t mask_check_223 = bitmask(2, 2, 3)
      //  | bitmask(2, 2, 2) | bitmask(2, 3, 3) | bitmask(3, 2, 3)
      //  | bitmask(2, 1, 3) | bitmask(1, 2, 3);
      //if ((old_b & mask_check_223) == mask_check_223
      //    && (above  & bitmask(2, 2, 0)))
      //{ new_b &= ~bitmask(2, 2, 3); }

      // store this new block if it has any cells set
      if (new_b) { new_data[idx] = new_b; }
    }
    _data = new_data;
  }

protected:
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

  void dimension_check(const SparseVoxelObject &other) const {
    if (Nx != other.Nx || Ny != other.Ny || Nz != other.Nz) {
      // TODO: add the mismatching dimensions to the message
      throw std::invalid_argument("Voxel dimensions do not match");
    }
  }

  void limit_check(const SparseVoxelObject &other) const {
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
  std::map<size_t, uint64_t> _data; // sparse voxel data
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

#endif // SPARSE_VOXEL_OBJECT_H

