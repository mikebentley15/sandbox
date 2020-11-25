#ifndef TREE_NODE_H
#define TREE_NODE_H

#include <boost/iterator/iterator_adaptor.hpp>

#include <algorithm>
#include <functional>
#include <memory>

namespace detail {

template <size_t _Nt> class TreeNode {
  static_assert(_Nt % 2 == 0, "TreeNode: number of nodes must be even");

public:
  static const size_t Nt         = _Nt;       // voxel dimension size
  static const size_t N          = _Nt * _Nt * _Nt; // total voxels
  static const size_t Nbt        = _Nt / 4;   // block dimension size
  static const size_t Nb         = N / 64;    // total blocks 
  static const size_t child_Nt   = Nt / 2;    // each child node voxel dimension size
  static const size_t child_Nbt  = Nbt / 2;   // each child node block dimension size

  using Child = TreeNode<child_Nt>;
  using ChildPtr = std::unique_ptr<Child>;

  TreeNode() = default;
  TreeNode(const TreeNode<_Nt> &other); // copy
  TreeNode(TreeNode<_Nt> &&other) = default;
  ~TreeNode() = default;

  size_t nblocks() const;
  bool is_empty() const;
  uint64_t block(size_t bx, size_t by, size_t bz) const;
  void set_block(size_t bx, size_t by, size_t bz, uint64_t value);
  uint64_t union_block(size_t bx, size_t by, size_t bz, uint64_t value);
  uint64_t intersect_block(size_t bx, size_t by, size_t bz, uint64_t value);
  bool collides(const TreeNode<Nt> &other) const;

  void visit_leaves(std::function<void(size_t, size_t, size_t, uint64_t)> visitor) const;
  void visit_leaves(std::function<void(size_t, size_t, size_t, uint64_t&)> visitor);

  void visit_leaves_2(std::function<void(size_t, size_t, size_t, uint64_t)> visitor) const {
    visit_leaves_2_impl(visitor, 0, 0, 0);
  }

  void visit_leaves_2_impl(
      std::function<void(size_t, size_t, size_t, uint64_t)> visitor,
      size_t dx, size_t dy, size_t dz) const;
protected:
  size_t idx(size_t bx, size_t by, size_t bz) const {
    return (bz/child_Nbt) + 2*(by/child_Nbt) + 4*(bx/child_Nbt);
  }

protected:
  std::array<ChildPtr, 8> _children;
};

} // end of namespace detail

#include "TreeNode.hxx"

#endif // TREE_NODE_H
