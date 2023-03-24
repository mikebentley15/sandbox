#ifndef CONNECTING_GRAPH_3_H
#define CONNECTING_GRAPH_3_H

#include <algorithm> // for std::find()
#include <memory>    // for std::unique_ptr and std::make_unique()
#include <vector>    // for std::vector
#include <numeric>   // for std::iota

/// A data structure that tracks connected components with undirected edges
class ConnectingGraph3 {
public:

  /// Initialize a graph with no edges and n vertices
  explicit ConnectingGraph3(int n)
    : _n(n)
    , _n_colors(n)
    , _colors(std::make_unique<int[]>(n))
    , _adj(std::make_unique<std::vector<int>[]>(n))
  {
    std::iota(_colors.get(), _colors.get()+n, 0);
  }

  /// Create an undirected edge from a to b
  void connect(int a, int b) {
    // error checking
    if (a == b) { return; }
    if (!in_bounds(a) || !in_bounds(b)) { return; }
    if (edge_exists(a, b)) { return; }

    add_edge(a, b);
    if (_colors[a] != _colors[b]) {
      --_n_colors;
      color_component(b, _colors[a]);
    }
  }

  /// Return the number of connected components
  int query() const { return _n_colors; }

private:

  bool in_bounds(int a) const { return 0 <= a && a < _n; }

  bool edge_exists(int a, int b) const {
    return _adj[a].end() != std::find(_adj[a].begin(), _adj[a].end(), b);
  }

  void add_edge(int a, int b) {
    _adj[a].emplace_back(b);
    _adj[b].emplace_back(a);
  }

  void color_component(int node, int color) {
    _colors[node] = color;
    for (auto neighbor : _adj[node]) {
      if (_colors[neighbor] != color) {
        color_component(neighbor, color);
      }
    }
  }

private:

  int _n;                                   ///< number of vertices
  int _n_colors;                            ///< number of colors
  std::unique_ptr<int[]> _colors;           ///< color
  std::unique_ptr<std::vector<int>[]> _adj; ///< adjacency list

}; // end of class ConnectingGraph3

#endif // CONNECTING_GRAPH_3_H
