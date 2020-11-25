#ifndef VOXEL_OCTREE_UNION_HXX
#define VOXEL_OCTREE_UNION_HXX

#include "VoxelOctreeUnion.h"

#include <boost/iterator/iterator_adaptor.hpp>

#include <algorithm>
#include <functional>
#include <type_traits>  // for std::decay_t()

inline
VoxelOctreeUnion::VoxelOctreeUnion(size_t N_)
  : _N(N_)
{
  switch (_N) {
    case   4: new(&(tree_004)) detail::TreeNode<  4>; break;
    case   8: new(&(tree_008)) detail::TreeNode<  8>; break;
    case  16: new(&(tree_016)) detail::TreeNode< 16>; break;
    case  32: new(&(tree_032)) detail::TreeNode< 32>; break;
    case  64: new(&(tree_064)) detail::TreeNode< 64>; break;
    case 128: new(&(tree_128)) detail::TreeNode<128>; break;
    case 256: new(&(tree_256)) detail::TreeNode<256>; break;
    case 512: new(&(tree_512)) detail::TreeNode<512>; break;
  }
  set_xlim(0.0, 1.0);
  set_ylim(0.0, 1.0);
  set_zlim(0.0, 1.0);
}

inline
VoxelOctreeUnion::VoxelOctreeUnion(const VoxelOctreeUnion &other) // copy
  : _N(other._N)
  , _xmin(other._xmin), _xmax(other._xmax)
  , _ymin(other._ymin), _ymax(other._ymax)
  , _zmin(other._zmin), _zmax(other._zmax)
  , _dx(other._dx), _dy(other._dy), _dz(other._dz)
{
  switch (_N) {
    case   4: new(&(tree_004)) detail::TreeNode<  4>(other.tree_004); break;
    case   8: new(&(tree_008)) detail::TreeNode<  8>(other.tree_008); break;
    case  16: new(&(tree_016)) detail::TreeNode< 16>(other.tree_016); break;
    case  32: new(&(tree_032)) detail::TreeNode< 32>(other.tree_032); break;
    case  64: new(&(tree_064)) detail::TreeNode< 64>(other.tree_064); break;
    case 128: new(&(tree_128)) detail::TreeNode<128>(other.tree_128); break;
    case 256: new(&(tree_256)) detail::TreeNode<256>(other.tree_256); break;
    case 512: new(&(tree_512)) detail::TreeNode<512>(other.tree_512); break;
  }
}

inline
VoxelOctreeUnion::VoxelOctreeUnion(VoxelOctreeUnion &&other)  // move
  : _N(other._N)
  , _xmin(other._xmin), _xmax(other._xmax)
  , _ymin(other._ymin), _ymax(other._ymax)
  , _zmin(other._zmin), _zmax(other._zmax)
  , _dx(other._dx), _dy(other._dy), _dz(other._dz)
{
  switch (_N) {
    case   4: new(&(tree_004)) detail::TreeNode<  4>(std::move(other.tree_004)); break;
    case   8: new(&(tree_008)) detail::TreeNode<  8>(std::move(other.tree_008)); break;
    case  16: new(&(tree_016)) detail::TreeNode< 16>(std::move(other.tree_016)); break;
    case  32: new(&(tree_032)) detail::TreeNode< 32>(std::move(other.tree_032)); break;
    case  64: new(&(tree_064)) detail::TreeNode< 64>(std::move(other.tree_064)); break;
    case 128: new(&(tree_128)) detail::TreeNode<128>(std::move(other.tree_128)); break;
    case 256: new(&(tree_256)) detail::TreeNode<256>(std::move(other.tree_256)); break;
    case 512: new(&(tree_512)) detail::TreeNode<512>(std::move(other.tree_512)); break;
  }
}

inline
VoxelOctreeUnion::~VoxelOctreeUnion() {
  switch (_N) {
    case   4: tree_004.~TreeNode(); break;
    case   8: tree_008.~TreeNode(); break;
    case  16: tree_016.~TreeNode(); break;
    case  32: tree_032.~TreeNode(); break;
    case  64: tree_064.~TreeNode(); break;
    case 128: tree_128.~TreeNode(); break;
    case 256: tree_256.~TreeNode(); break;
    case 512: tree_512.~TreeNode(); break;
  }
}

inline
void VoxelOctreeUnion::set_xlim(double xmin, double xmax) {
  if (xmin >= xmax) {
    throw std::length_error("xlimits must be positive in size");
  }
  _xmin = xmin;
  _xmax = xmax;
  _dx = (xmax - xmin) / Nx();
}

inline
void VoxelOctreeUnion::set_ylim(double ymin, double ymax) {
  if (ymin >= ymax) {
    throw std::length_error("ylimits must be positive in size");
  }
  _ymin = ymin;
  _ymax = ymax;
  _dy = (ymax - ymin) / Ny();
}

inline
void VoxelOctreeUnion::set_zlim(double zmin, double zmax) {
  if (zmin >= zmax) {
    throw std::length_error("zlimits must be positive in size");
  }
  _zmin = zmin;
  _zmax = zmax;
  _dz = (zmax - zmin) / Nz();
}

inline
size_t VoxelOctreeUnion::nblocks() const {
  switch (_N) {
    case   4: return tree_004.nblocks(); break;
    case   8: return tree_008.nblocks(); break;
    case  16: return tree_016.nblocks(); break;
    case  32: return tree_032.nblocks(); break;
    case  64: return tree_064.nblocks(); break;
    case 128: return tree_128.nblocks(); break;
    case 256: return tree_256.nblocks(); break;
    case 512: return tree_512.nblocks(); break;
  }
}

inline
uint64_t VoxelOctreeUnion::block(size_t bx, size_t by, size_t bz) const {
  switch (_N) {
    case   4: return tree_004.block(bx, by, bz); break;
    case   8: return tree_008.block(bx, by, bz); break;
    case  16: return tree_016.block(bx, by, bz); break;
    case  32: return tree_032.block(bx, by, bz); break;
    case  64: return tree_064.block(bx, by, bz); break;
    case 128: return tree_128.block(bx, by, bz); break;
    case 256: return tree_256.block(bx, by, bz); break;
    case 512: return tree_512.block(bx, by, bz); break;
  }
}

inline
void VoxelOctreeUnion::set_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  switch (_N) {
    case   4: return tree_004.set_block(bx, by, bz, value); break;
    case   8: return tree_008.set_block(bx, by, bz, value); break;
    case  16: return tree_016.set_block(bx, by, bz, value); break;
    case  32: return tree_032.set_block(bx, by, bz, value); break;
    case  64: return tree_064.set_block(bx, by, bz, value); break;
    case 128: return tree_128.set_block(bx, by, bz, value); break;
    case 256: return tree_256.set_block(bx, by, bz, value); break;
    case 512: return tree_512.set_block(bx, by, bz, value); break;
  }
}

// returns old block type before unioning with value
inline
uint64_t VoxelOctreeUnion::union_block(size_t bx, size_t by, size_t bz,
                                  uint64_t value)
{
  switch (_N) {
    case   4: return tree_004.union_block(bx, by, bz, value); break;
    case   8: return tree_008.union_block(bx, by, bz, value); break;
    case  16: return tree_016.union_block(bx, by, bz, value); break;
    case  32: return tree_032.union_block(bx, by, bz, value); break;
    case  64: return tree_064.union_block(bx, by, bz, value); break;
    case 128: return tree_128.union_block(bx, by, bz, value); break;
    case 256: return tree_256.union_block(bx, by, bz, value); break;
    case 512: return tree_512.union_block(bx, by, bz, value); break;
  }
}

inline
uint64_t VoxelOctreeUnion::intersect_block(size_t bx, size_t by, size_t bz,
                                      uint64_t value)
{
  switch (_N) {
    case   4: return tree_004.intersect_block(bx, by, bz, value); break;
    case   8: return tree_008.intersect_block(bx, by, bz, value); break;
    case  16: return tree_016.intersect_block(bx, by, bz, value); break;
    case  32: return tree_032.intersect_block(bx, by, bz, value); break;
    case  64: return tree_064.intersect_block(bx, by, bz, value); break;
    case 128: return tree_128.intersect_block(bx, by, bz, value); break;
    case 256: return tree_256.intersect_block(bx, by, bz, value); break;
    case 512: return tree_512.intersect_block(bx, by, bz, value); break;
  }
}

inline
bool VoxelOctreeUnion::cell(size_t ix, size_t iy, size_t iz) const {
  auto b = block(ix / 4, iy / 4, iz / 4);
  return b && (b & bitmask(ix % 4, iy % 4, iz % 4));
}

// sets the cell's value
// returns true if the cell's value changed
inline
bool VoxelOctreeUnion::set_cell(size_t ix, size_t iy, size_t iz, bool value) {
  auto mask = bitmask(ix % 4, iy % 4, iz % 4);
  if (value) {
    auto oldval = this->union_block(ix / 4, iy / 4, iz / 4, mask);
    return oldval & mask;
  } else {
    auto oldval = this->intersect_block(ix / 4, iy / 4, iz / 4, ~mask);
    return oldval & ~mask;
  }
}

inline
uint64_t VoxelOctreeUnion::find_block(double x, double y, double z) const {
  auto [bx, by, bz] = find_block_idx(x, y, z);
  return block(bx, by, bz);
}

inline
std::tuple<size_t, size_t, size_t> VoxelOctreeUnion::find_block_idx(
    double x, double y, double z) const
{
  domain_check(x, y, z);
  size_t ix = size_t((x - _xmin) / _dx);
  size_t iy = size_t((y - _ymin) / _dy);
  size_t iz = size_t((z - _zmin) / _dz);
  return {ix / 4, iy / 4, iz / 4};
}

inline
std::tuple<size_t, size_t, size_t> VoxelOctreeUnion::find_cell(
    double x, double y, double z) const
{
  domain_check(x, y, z);
  size_t ix = size_t((x - _xmin) / _dx);
  size_t iy = size_t((y - _ymin) / _dy);
  size_t iz = size_t((z - _zmin) / _dz);
  return {ix, iy, iz};
}

inline
void VoxelOctreeUnion::add_point(double x, double y, double z) {
  auto [ix, iy, iz] = find_cell(x, y, z);
  set_cell(ix, iy, iz);
}

// TODO: easier algorithm is to go through each voxel, create an FCL AABB box
// TODO- and collision check it against the object to be added (for sphere,
// TODO- capsule, and mesh).  Slow, but effective

// sets sphere as occupied in voxel space, with center and radius specified
// adds any voxels that intersect or are inside of the sphere.
inline
void VoxelOctreeUnion::add_sphere(double x, double y, double z, double r) {
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
        if (b) { this->union_block(bx, by, bz, b); }
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

inline
void VoxelOctreeUnion::remove_interior_slow_1() {
  const VoxelOctreeUnion copy(*this);
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
inline
void VoxelOctreeUnion::remove_interior() {
  auto copy = *this;

  // iterate over copy and modify tree
  auto visitor = [this, &copy](size_t bx, size_t by, size_t bz, uint64_t old_b) {

#define my_assert(val) if (!val) { throw std::runtime_error(#val); }
    my_assert(old_b == copy.block(bx, by, bz));

    const uint64_t full = ~uint64_t(0);
    auto new_b = old_b;
    const uint64_t left   = (bx <= 0)       ? full : copy.block(bx-1, by, bz);
    const uint64_t right  = (bx >= Nbx()-1) ? full : copy.block(bx+1, by, bz);
    const uint64_t front  = (by <= 0)       ? full : copy.block(bx, by-1, bz);
    const uint64_t behind = (by >= Nby()-1) ? full : copy.block(bx, by+1, bz);
    const uint64_t below  = (bz <= 0)       ? full : copy.block(bx, by, bz-1);
    const uint64_t above  = (bz >= Nbz()-1) ? full : copy.block(bx, by, bz+1);

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
    this->set_block(bx, by, bz, new_b);
  };

  switch (_N) {
    case   4: tree_004.visit_leaves_2(visitor); break;
    case   8: tree_008.visit_leaves_2(visitor); break;
    case  16: tree_016.visit_leaves_2(visitor); break;
    case  32: tree_032.visit_leaves_2(visitor); break;
    case  64: tree_064.visit_leaves_2(visitor); break;
    case 128: tree_128.visit_leaves_2(visitor); break;
    case 256: tree_256.visit_leaves_2(visitor); break;
    case 512: tree_512.visit_leaves_2(visitor); break;
  }
}

inline
bool VoxelOctreeUnion::collides_check(const VoxelOctreeUnion &other) const {
  limit_check(other); // significantly slows down collision checking?
  return collides(other);
}

inline
bool VoxelOctreeUnion::collides(const VoxelOctreeUnion &other) const {
  if (_N != other._N) {
    throw std::domain_error("voxel objects must match in size");
  }
  switch (_N) {
    case   4: return tree_004.collides(other.tree_004); break;
    case   8: return tree_008.collides(other.tree_008); break;
    case  16: return tree_016.collides(other.tree_016); break;
    case  32: return tree_032.collides(other.tree_032); break;
    case  64: return tree_064.collides(other.tree_064); break;
    case 128: return tree_128.collides(other.tree_128); break;
    case 256: return tree_256.collides(other.tree_256); break;
    case 512: return tree_512.collides(other.tree_512); break;
  }
}

// one in the given place, zeros everywhere else
// for x, y, z within a block (each should be 0 <= x < 4)
inline
uint64_t VoxelOctreeUnion::bitmask(uint_fast8_t x, uint_fast8_t y, uint_fast8_t z) const {
  return uint64_t(1) << (x*16 + y*4 + z);
}

inline
void VoxelOctreeUnion::domain_check(double x, double y, double z) const {
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

inline
void VoxelOctreeUnion::limit_check(const VoxelOctreeUnion &other) const {
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

#endif // VOXEL_OCTREE_UNION_HXX
