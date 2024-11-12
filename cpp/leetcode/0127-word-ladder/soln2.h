#ifndef SOLN_2_H
#define SOLN_2_H

// This is a search problem
// Possible approaches:
//
// 1. BFS
// 2. Iterative Deepening
// 3. Uniform Cost Search (Dijkstra's alg)
// 4. A*
//
// Notes:
// - Dijkstra's alg is the same as BFS for all edges of equal weight (which is)
// - Bidirectional search can reduce the frontier exponentially
// - Easy to do BFS with bidirectional search.  A* is harder.

// Ways to generate neighbors:
//
// 1. pre and post tries
// 2. enumerate all possible neighbors and check against a set
// 3. pre-populate an adjacency list
//
// Let's start out with approach 1 (bidirectional) and generating neighbors with approach 2
// We could actually implement all combinations of each one

#include <string>
#include <vector>

class Solution {
 public:

  int ladderLength(std::string beginWord, std::string endWord,
                   std::vector<std::string>& wordList) const
  {
    using Set = std::unordered_set<std::string_view>;

    Set words(wordList.begin(), wordList.end());
    if (0 == words.count(endWord)) { return 0; }

    const int N = static_cast<int>(beginWord.size());

    std::string buffer = beginWord;
    auto for_each_possible_neighbor = [&buffer, N](auto state, auto consume) {
        buffer = std::string(state.data(), state.size());
        for (int i = 0; i < N; ++i) {
            char tmp_ch = buffer[i];
            for (char ch = 'a'; ch <= 'z'; ++ch) {
                buffer[i] = ch;
                auto ret = consume(buffer);
                if (ret) { return ret; }
            }
            buffer[i] = tmp_ch; // restore
        }
        return 0;
    };

    Set frontier_a, frontier_b;
    frontier_a.emplace(beginWord);
    frontier_b.emplace(endWord);
    int distance = 0;
    while (!frontier_a.empty() && !frontier_b.empty()) {
      ++distance;

      // work on the smaller frontier
      if (frontier_a.size() > frontier_b.size()) {
        std::swap(frontier_a, frontier_b);
      }

      Set new_frontier_a;
      for (auto state : frontier_a) {
        auto ret = for_each_possible_neighbor(state, [&](const std::string &neighbor) {
          if (frontier_b.count(neighbor)) {
            return distance + 1;
          }
          auto found = words.find(neighbor);
          if (found != words.end()) {
              auto found_state = *found;
              words.erase(found); // mark as visited
              new_frontier_a.emplace(found_state);
          }
          return 0;
        });
        if (ret) {
          return ret;
        }
      }

      frontier_a = std::move(new_frontier_a);
    }

    std::cout << "Visited " << (wordList.size() - words.size()) << " of " << wordList.size() << " words\n";

    // if we didn't return during the search, then it wasn't found
    return 0;
  }
};

#endif  // SOLN_2_H
