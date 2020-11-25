#ifndef CT_SPARSE_VOXEL_OBJECT_H
#define CT_SPARSE_VOXEL_OBJECT_H

#include <iostream>
#include <limits>    // for std::numeric_limits
#include <map>       // for std::map
#include <stack>     // for std::stack
#include <stdexcept> // for std::length_error
#include <tuple>     // for std::tuple
#include <utility>   // for std::pair

#include <cstddef> // for size_t
#include <cstdint> // for uint64_t

template <size_t _Nx, size_t _Ny, size_t _Nz>
class CTSparseVoxelObject {
  static_assert(_Nx % 4 == 0, "CTSparseVoxelObject: x dimension must be a multiple of 4");
  static_assert(_Ny % 4 == 0, "CTSparseVoxelObject: y dimension must be a multiple of 4");
  static_assert(_Nz % 4 == 0, "CTSparseVoxelObject: z dimension must be a multiple of 4");

public:
  static constexpr size_t Nx() { return _Nx; } // number of voxels in the x-direction
  static constexpr size_t Ny() { return _Ny; } // number of voxels in the y-direction
  static constexpr size_t Nz() { return _Nz; } // number of voxels in the z-direction
  static constexpr size_t N()  { return Nx() * Ny() * Nz(); }

  static constexpr size_t Nbx() { return Nx() / 4; } // number of blocks in the x-direction
  static constexpr size_t Nby() { return Ny() / 4; } // number of blocks in the y-direction
  static constexpr size_t Nbz() { return Nz() / 4; } // number of blocks in the z-direction
  static constexpr size_t Nb()  { return Nbx() * Nby() * Nbz(); }

public:
  CTSparseVoxelObject() : _data() {
    set_xlim(0.0, 1.0);
    set_ylim(0.0, 1.0);
    set_zlim(0.0, 1.0);
  }

  CTSparseVoxelObject(const CTSparseVoxelObject &other)  // copy
    : _data(other._data)
    , _xmin(other._xmin), _xmax(other._xmax)
    , _ymin(other._ymin), _ymax(other._ymax)
    , _zmin(other._zmin), _zmax(other._zmax)
    , _dx(other._dx), _dy(other._dy), _dz(other._dz)
  {}

  CTSparseVoxelObject(CTSparseVoxelObject &&other)  // move
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
    _dx = (xmax - xmin) / Nx();
  }

  void set_ylim(double ymin, double ymax) {
    if (ymin >= ymax) {
      throw std::length_error("ylimits must be positive in size");
    }
    _ymin = ymin;
    _ymax = ymax;
    _dy = (ymax - ymin) / Ny();
  }

  void set_zlim(double zmin, double zmax) {
    if (zmin >= zmax) {
      throw std::length_error("zlimits must be positive in size");
    }
    _zmin = zmin;
    _zmax = zmax;
    _dz = (zmax - zmin) / Nz();
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

  uint64_t block(size_t bx, size_t by, size_t bz) const {
    return block(block_idx(bx, by, bz));
  }

  void set_block(size_t idx, uint64_t value) {
    _data[idx] = value;
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
          if (b) { _data[block_idx(bx, by, bz)] |= b; }
        }
      }
    }

    //// do a growing algorithm with a frontier and a visited
    //using IdxType = std::tuple<size_t, size_t, size_t>;
    //std::stack<IdxType> frontier;
    //auto visited = std::make_unique<VoxelObject<_Nx, _Ny, _Nz>>();

    //auto check_push = [&frontier, &visited](size_t _ix, size_t _iy, size_t _iz) {
    //  if (!visited->cell(_ix, _iy, _iz) && _ix < _Nx && _iy < _Ny && _iz < _Nz) {
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

  bool collides_check(const CTSparseVoxelObject<_Nx, _Ny, _Nz> &other) const {
    limit_check(other); // significantly slows down collision checking
    return collides(other);
  }

  bool collides(const CTSparseVoxelObject<_Nx, _Ny, _Nz> &other) const {
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

protected:
  size_t block_idx(size_t bx, size_t by, size_t bz) {
    return bz + Nz()*(by + Ny()*bx);
  }

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

  void limit_check(const CTSparseVoxelObject<_Nx, _Ny, _Nz> &other) const {
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

#endif // CT_SPARSE_VOXEL_OBJECT_H

