#ifndef TREE_CHECKER_H
#define TREE_CHECKER_H

#include <algorithm>
#include <vector>

class TreeChecker {
public:
  using AdjList = std::vector<std::vector<int>>;
  using EdgeList = std::vector<std::vector<int>>;

  bool validTree(int n, const EdgeList &edges) const {
    if (n < 0) { return false; }
    if (n == 0) { return true; }

    const AdjList adj = make_adjlist(n, edges);
    std::vector<bool> visited(n);

    // The graph is a tree if, starting from node 0, it is a valid tree and
    // we visit every node
    return validTree_recursive(n, 0, adj, visited)
        && std::all_of(visited.begin(), visited.end(),
                       [](bool val){ return val; });
  }

private:
  AdjList make_adjlist(int n, const EdgeList &edges) const {
    AdjList adj(n);
    for (const auto &e : edges) {
      adj[e[0]].emplace_back(e[1]);
      adj[e[1]].emplace_back(e[0]); // bidirectional
    }
    return adj;
  }

  /// Recursive version using DFS
  bool validTree_recursive(
      int prev, int node, const AdjList &adj, std::vector<bool> &visited) const
  {
    if (visited[node]) { return false; } // detected cycle
    visited[node] = true;

    for (auto neighbor : adj[node]) {
      if (neighbor == prev) { continue; }
      if (!validTree_recursive(node, neighbor, adj, visited)) {
        return false;
      }
    }
    return true;
  }
};

#endif // TREE_CHECKER_H
