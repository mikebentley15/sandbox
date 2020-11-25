#ifndef CT_VOXEL_OCTREE_WRAP_HXX
#define CT_VOXEL_OCTREE_WRAP_HXX

#include "CTVoxelOctreeWrap.h"

#include <type_traits>

inline
CTVoxelOctreeWrap::CTVoxelOctreeWrap(size_t _N)
  : Nx(_N), Ny(_N), Nz(_N), N(Nx*Ny*Nz)
  , Nbx(_N/4), Nby(_N/4), Nbz(_N/4), Nb(Nbx*Nby*Nbz)
{
  if      (_N ==   4) { _data.emplace<CTVoxelOctree<  4>>(); }
  else if (_N ==   8) { _data.emplace<CTVoxelOctree<  8>>(); }
  else if (_N ==  16) { _data.emplace<CTVoxelOctree< 16>>(); }
  else if (_N ==  32) { _data.emplace<CTVoxelOctree< 32>>(); }
  else if (_N ==  64) { _data.emplace<CTVoxelOctree< 64>>(); }
  else if (_N == 128) { _data.emplace<CTVoxelOctree<128>>(); }
  else if (_N == 256) { _data.emplace<CTVoxelOctree<256>>(); }
  else if (_N == 512) { _data.emplace<CTVoxelOctree<512>>(); }
  else {
    throw std::invalid_argument("unsupported voxel dimension: "
                                + std::to_string(_N));
  }
}

inline
void CTVoxelOctreeWrap::set_xlim(double xmin, double xmax) {
  std::visit([&xmin, &xmax] (auto &data) { data.set_xlim(xmin, xmax); },
             _data);
}

inline
void CTVoxelOctreeWrap::set_ylim(double ymin, double ymax) {
  std::visit([&ymin, &ymax] (auto &data) { data.set_ylim(ymin, ymax); },
             _data);
}

inline
void CTVoxelOctreeWrap::set_zlim(double zmin, double zmax) {
  std::visit([&zmin, &zmax] (auto &data) { data.set_zlim(zmin, zmax); },
             _data);
}

inline
std::pair<double, double> CTVoxelOctreeWrap::xlim() const {
  return std::visit([] (auto &data) { return data.xlim(); }, _data);
}

inline
std::pair<double, double> CTVoxelOctreeWrap::ylim() const {
  return std::visit([] (auto &data) { return data.ylim(); }, _data);
}

inline
std::pair<double, double> CTVoxelOctreeWrap::zlim() const {
  return std::visit([] (auto &data) { return data.zlim(); }, _data);
}

// size of cells
inline
double CTVoxelOctreeWrap::dx() const {
  return std::visit([] (auto &data) { return data.dx(); }, _data);
}

inline
double CTVoxelOctreeWrap::dy() const {
  return std::visit([] (auto &data) { return data.dy(); }, _data);
}

inline
double CTVoxelOctreeWrap::dz() const {
  return std::visit([] (auto &data) { return data.dz(); }, _data);
}


// size of blocks
inline
double CTVoxelOctreeWrap::dbx() const {
  return std::visit([] (auto &data) { return data.dbx(); }, _data);
}

inline
double CTVoxelOctreeWrap::dby() const {
  return std::visit([] (auto &data) { return data.dby(); }, _data);
}

inline
double CTVoxelOctreeWrap::dbz() const {
  return std::visit([] (auto &data) { return data.dbz(); }, _data);
}


inline
size_t CTVoxelOctreeWrap::nblocks() const {
  return std::visit([] (auto &data) { return data.nblocks(); }, _data);
}


inline
uint64_t CTVoxelOctreeWrap::block(size_t bx, size_t by, size_t bz) const {
  return std::visit(
      [&bx, &by, &bz] (auto &data) { return data.block(bx, by, bz); }, _data);
}

inline
void CTVoxelOctreeWrap::set_block(size_t bx, size_t by, size_t bz,
                                  uint64_t value)
{
  return std::visit(
      [&bx, &by, &bz, &value] (auto &data) {
        return data.set_block(bx, by, bz, value);
      },
      _data);
}

// returns old block value before unioning or intersecting with value
inline
uint64_t CTVoxelOctreeWrap::union_block(size_t bx, size_t by, size_t bz,
                                        uint64_t value)
{
  return std::visit(
      [&bx, &by, &bz, &value] (auto &data) {
        return data.union_block(bx, by, bz, value);
      },
      _data);
}

inline
uint64_t CTVoxelOctreeWrap::intersect_block(size_t bx, size_t by, size_t bz,
                                            uint64_t value)
{
  return std::visit(
      [&bx, &by, &bz, &value] (auto &data) {
        return data.intersect_block(bx, by, bz, value);
      },
      _data);
}


inline
bool CTVoxelOctreeWrap::cell(size_t ix, size_t iy, size_t iz) const {
  return std::visit(
      [&ix, &iy, &iz] (auto &data) { return data.cell(ix, iy, iz); }, _data);
}

// sets the cell's value
// returns true if the cell's value changed
inline
bool CTVoxelOctreeWrap::set_cell(size_t ix, size_t iy, size_t iz, bool value) {
  return std::visit(
      [&ix, &iy, &iz, &value] (auto &data) {
        return data.set_cell(ix, iy, iz, value);
      },
      _data);
}

inline
uint64_t CTVoxelOctreeWrap::find_block(double x, double y, double z) const {
  return std::visit(
      [&x, &y, &z] (auto &data) { return data.find_block(x, y, z); }, _data);
}

inline
std::tuple<size_t, size_t, size_t> CTVoxelOctreeWrap::find_block_idx(
    double x, double y, double z) const
{
  return std::visit(
      [&x, &y, &z] (auto &data) { return data.find_block_idx(x, y, z); },
      _data);
}

inline
std::tuple<size_t, size_t, size_t> CTVoxelOctreeWrap::find_cell(
    double x, double y, double z) const
{
  return std::visit(
      [&x, &y, &z] (auto &data) { return data.find_cell(x, y, z); }, _data);
}

inline
void CTVoxelOctreeWrap::add_point(double x, double y, double z) {
  std::visit([&x, &y, &z] (auto &data) { data.add_point(x, y, z); }, _data);
}

inline
void CTVoxelOctreeWrap::add_sphere(double x, double y, double z, double r) {
  std::visit([&x, &y, &z, &r] (auto &data) { data.add_sphere(x, y, z, r); },
             _data);
}

inline
void CTVoxelOctreeWrap::remove_interior_slow_1() {
  std::visit([] (auto &data) { data.remove_interior_slow_1(); }, _data);
}

inline
void CTVoxelOctreeWrap::remove_interior() {
  std::visit([] (auto &data) { data.remove_interior(); }, _data);
}

inline
bool CTVoxelOctreeWrap::collides_check(const CTVoxelOctreeWrap &other) const {
  if (Nx != other.Nx) {
    throw std::invalid_argument("collides_check(): sizes do not match");
  }
  return std::visit(
      [] (auto &a, auto &b) {
        using A = std::decay_t<decltype(a)>;
        using B = std::decay_t<decltype(b)>;
        if constexpr (std::is_same_v<A, B>) {
          return a.collides_check(b);
        } else {
          return true;
        }
      },
      _data, other._data);
}

inline
bool CTVoxelOctreeWrap::collides(const CTVoxelOctreeWrap &other) const {
  return std::visit(
      [] (auto &a, auto &b) {
        using A = std::decay_t<decltype(a)>;
        using B = std::decay_t<decltype(b)>;
        if constexpr (std::is_same_v<A, B>) {
          return a.collides(b);
        } else {
          return true;
        }
      },
      _data, other._data);
}

#endif // CT_VOXEL_OCTREE_WRAP_HXX
