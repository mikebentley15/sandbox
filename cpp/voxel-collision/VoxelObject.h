#ifndef VOXEL_OBJECT_H
#define VOXEL_OBJECT_H

#include <stdexcept> // for std::length_error

#include <cstddef> // for size_t
#include <cstdint> // for uint64_t

template <size_t _Nx, size_t _Ny, size_t _Nz>
class VoxelObject {
  static_assert(_Nx % 4 == 0, "VoxelObject: x dimension must be a multiple of 4");
  static_assert(_Ny % 4 == 0, "VoxelObject: y dimension must be a multiple of 4");
  static_assert(_Nz % 4 == 0, "VoxelObject: z dimension must be a multiple of 4");

public:
  static const size_t Nx = _Nx;  // number of voxels in the x-direction
  static const size_t Ny = _Ny;  // number of voxels in the y-direction
  static const size_t Nz = _Nz;  // number of voxels in the z-direction

  static const size_t Nbx = Nx / 4; // number of blocks in the x-direction
  static const size_t Nby = Ny / 4; // number of blocks in the y-direction
  static const size_t Nbz = Nz / 4; // number of blocks in the z-direction

public:
  VoxelObject() : _data() {
    set_xlim(0.0, 1.0);
    set_ylim(0.0, 1.0);
    set_zlim(0.0, 1.0);
  }

  void set_xlim(double xmin, double xmax) {
    if (xmin >= xmax) {
      throw std::length_error("xlimits must be positive in size");
    }
    _xmin = xmin;
    _xmax = xmax;
  }

  void set_ylim(double ymin, double ymax) {
    if (ymin >= ymax) {
      throw std::length_error("ylimits must be positive in size");
    }
    _ymin = ymin;
    _ymax = ymax;
  }

  void set_zlim(double zmin, double zmax) {
    if (zmin >= zmax) {
      throw std::length_error("zlimits must be positive in size");
    }
    _zmin = zmin;
    _zmax = zmax;
  }

  uint64_t& block(size_t bx, size_t by, size_t bz) { return _data[bx][by][bz]; }
  uint64_t  block(size_t bx, size_t by, size_t bz) const { return _data[bx][by][bz]; }

  bool cell(size_t x, size_t y, size_t z) const {
    auto b = block(x/4, y/4, z/4);
    return b & bitmask(x%4, y%4, z%4);
  }

protected:
  // one in the given place, zeros everywhere else
  // for x, y, z within a block (each should be 0 <= x < 4)
  uint64_t bitmask(uint_fast8_t x, uint_fast8_t y, uint_fast8_t z) const {
    return uint64_t(1) << (x*16 + y*4 + z);
  }

private:
  uint64_t _data[Nbx][Nby][Nbz]; // voxel data
  double _xmin;
  double _xmax;
  double _ymin;
  double _ymax;
  double _zmin;
  double _zmax;
  // TODO: _dx, _dy, _dz
};

#endif // VOXEL_OBJECT_H
