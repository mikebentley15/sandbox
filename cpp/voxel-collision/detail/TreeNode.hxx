#ifndef TREE_NODE_HXX
#define TREE_NODE_HXX

#include <algorithm>
#include <functional>


namespace detail {

template <size_t _Nt>
TreeNode<_Nt>::TreeNode(const TreeNode<_Nt> &other) { // copy
  for (int i = 0; i < 8; i++) {
    auto &child = other._children[i];
    if (child) {
      _children[i] = std::make_unique<Child>(*child);
    }
  }
}

template <size_t _Nt>
size_t TreeNode<_Nt>::nblocks() const {
  size_t child_nblocks = 0;
  for (auto &child : _children) {
    if (child) {
      child_nblocks += child->nblocks();
    }
  }
  return child_nblocks;
}

template <size_t _Nt>
bool TreeNode<_Nt>::is_empty() const {
  return std::all_of(_children.begin(), _children.end(),
                     [] (auto &x) { return x.get() == nullptr; });
}

template <size_t _Nt>
uint64_t TreeNode<_Nt>::block(size_t bx, size_t by, size_t bz) const {
  const ChildPtr &child = _children[idx(bx, by, bz)];
  if (child) {
    return child->block(bx % child_Nbt, by % child_Nbt, bz % child_Nbt);
  }
  return 0;
}

template <size_t _Nt>
void TreeNode<_Nt>::set_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  ChildPtr &child = _children[idx(bx, by, bz)];
  if (!child && value) {
    child = std::make_unique<Child>();
  }
  if (child) {
    child->set_block(bx % child_Nbt, by % child_Nbt, bz % child_Nbt, value);
    if (!value && child->is_empty()) {
      child.reset(nullptr); // delete
    }
  }
}

template <size_t _Nt>
uint64_t TreeNode<_Nt>::union_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  ChildPtr &child = _children[idx(bx, by, bz)];
  if (!child) {
    child = std::make_unique<Child>();
  }
  return child->union_block(bx % child_Nbt, by % child_Nbt, bz % child_Nbt, value);
}

template <size_t _Nt>
uint64_t TreeNode<_Nt>::intersect_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  ChildPtr &child = _children[idx(bx, by, bz)];
  if (child) {
    auto oldval = child->intersect_block(
        bx % child_Nbt, by % child_Nbt, bz % child_Nbt, value);
    if (child->is_empty()) {
      child.reset(nullptr); // delete to prune this part of the tree
    }
    return oldval;
  }
  return 0;
}

template <size_t _Nt>
bool TreeNode<_Nt>::collides(const TreeNode<Nt> &other) const {
  for (int i = 0; i < 8; i++) {
    if (_children[i] && other._children[i]
        && _children[i]->collides(*other._children[i]))
    {
      return true;
    }
  }
  return false;
}

template <size_t _Nt>
void TreeNode<_Nt>::visit_leaves(
    std::function<void(size_t, size_t, size_t, uint64_t)> visitor) const
{
  for (int i = 0; i < 8; i++) {
    const auto &child = _children[i];
    if (child) {
      child->visit_leaves([i, &visitor]
        (size_t ix, size_t iy, size_t iz, uint64_t value) {
          visitor((_Nt/8) * (i/4) + ix,
                  (_Nt/8) * ((i%4)/2) + iy,
                  (_Nt/8) * (i%2) + iz,
                  value);
        });
    }
  }
}

template <size_t _Nt>
void TreeNode<_Nt>::visit_leaves(
    std::function<void(size_t, size_t, size_t, uint64_t&)> visitor)
{
  for (int i = 0; i < 8; i++) {
    auto &child = _children[i];
    if (child) {
      child->visit_leaves([i, &visitor]
        (size_t ix, size_t iy, size_t iz, uint64_t &value) {
          visitor((_Nt/8) * (i/4) + ix,
                  (_Nt/8) * ((i%4)/2) + iy,
                  (_Nt/8) * (i%2) + iz,
                  value);
        });
      if (child->is_empty()) {
        child.reset(nullptr); // prune
      }
    }
  }
}

template <size_t _Nt>
void TreeNode<_Nt>::visit_leaves_2_impl(
    std::function<void(size_t, size_t, size_t, uint64_t)> visitor,
    size_t dx, size_t dy, size_t dz) const
{
  for (size_t bx = 0; bx < Nbt; bx += child_Nbt) {
    for (size_t by = 0; by < Nbt; by += child_Nbt) {
      for (size_t bz = 0; bz < Nbt; bz += child_Nbt) {
        auto &child = _children[idx(bx, by, bz)];
        if (child) {
          child->visit_leaves_2_impl(visitor, dx + bx, dy + by, dz + bz);
        }
      }
    }
  }
}

// leaf node
template <> class TreeNode<4> {
public:
  static const size_t Nt = 4;
  static const size_t N = 64;
  TreeNode() : _data(0) {}
  TreeNode(const TreeNode<4> &other) : _data(other._data) {}
  size_t nblocks() const { return 1; }
  bool is_empty() const { return !_data; }
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
  void visit_leaves(std::function<void(size_t, size_t, size_t, uint64_t)> visitor) const {
    visitor(0, 0, 0, _data);
  }
  void visit_leaves(std::function<void(size_t, size_t, size_t, uint64_t&)> visitor) {
    visitor(0, 0, 0, _data);
  }
  void visit_leaves_2(std::function<void(size_t, size_t, size_t, uint64_t)> visitor) const {
    visitor(0, 0, 0, _data);
  }
  void visit_leaves_2_impl(
      std::function<void(size_t, size_t, size_t, uint64_t)> visitor,
      size_t dx, size_t dy, size_t dz) const
  {
    visitor(dx, dy, dz, _data);
  }
protected:
  uint64_t _data;
};

} // end of namespace detail

#endif // TREE_NODE_HXX
