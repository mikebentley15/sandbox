#ifndef UNION_FIND_H
#define UNION_FIND_H

#include <numeric>

class UnionFind {
public:

  explicit UnionFind(int n)
    : _n_components(n), _parent(n)
  {
    std::iota(_parent.begin(), _parent.end(), 0);
  }

  void connect(int a, int b) {
    if (!in_bounds(a) || !in_bounds(b)) { return; } // do nothing
    auto pa = find_root(a);
    auto pb = find_root(b);
    if (pa != pb) {
      _parent[pb] = pa;
      --_n_components;
    }
  }

  int query() const { return _n_components; }

private:

  bool in_bounds(int a) { return 0 <= a && a < static_cast<int>(_parent.size()); }

  int find_root(int a) {
    while (a != _parent[a]) {
      _parent[a] = _parent[_parent[a]]; // short circuit
      a = _parent[a];
    }
    return a;
  }

  int _n_components;
  std::vector<int> _parent;

};

#endif // UNION_FIND_H
