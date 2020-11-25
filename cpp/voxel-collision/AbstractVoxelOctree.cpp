#include "AbstractVoxelOctree.h"

#include <stdexcept>

AbstractVoxelOctree::AbstractVoxelOctree() {
  set_xlim(0.0, 1.0);
  set_ylim(0.0, 1.0);
  set_zlim(0.0, 1.0);
}

void AbstractVoxelOctree::set_xlim(double xmin, double xmax) {
  if (xmin >= xmax) {
    throw std::length_error("xlimits must be positive in size");
  }
  _xmin = xmin;
  _xmax = xmax;
}

void AbstractVoxelOctree::set_ylim(double ymin, double ymax) {
  if (ymin >= ymax) {
    throw std::length_error("ylimits must be positive in size");
  }
  _ymin = ymin;
  _ymax = ymax;
}

void AbstractVoxelOctree::set_zlim(double zmin, double zmax) {
  if (zmin >= zmax) {
    throw std::length_error("zlimits must be positive in size");
  }
  _zmin = zmin;
  _zmax = zmax;
}

// sets the cell's value
// returns true if the cell's value changed

uint64_t AbstractVoxelOctree::find_block(double x, double y, double z) const {
  auto [bx, by, bz] = find_block_idx(x, y, z);
  return block(bx, by, bz);
}

void AbstractVoxelOctree::add_point(double x, double y, double z) {
  auto [ix, iy, iz] = find_cell(x, y, z);
  set_cell(ix, iy, iz);
}
