#include <algorithm>
#include <chrono>
#include <iomanip>
#include <ios>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <cstring>  // for memcpy
#include <cmath>    // for sqrt and atan2

using stringlist  = std::vector<std::string>;
using intlist     = std::vector<int>;
using stringset   = std::unordered_set<std::string>;
using intset      = std::unordered_set<unsigned int>;

const bool DEBUG = true;
//const bool DEBUG = false;
template <typename T>
void debug(const T &item, const std::string &prefix = "") {
  if (DEBUG) {
    std::cout << prefix << item << std::endl;
  }
}

#define my_assert(x) \
  if (!(x)) { \
    printf("Assertion failed!, line %d, %s", __LINE__, #x); \
  }

stringlist read_input() {
  stringlist inlist;
  while (!std::cin.eof()) {
    std::string line;
    std::getline(std::cin, line);
    if (line.size() > 0) { // ignore empty lines
      inlist.emplace_back(line);
    }
  }
  return inlist;
}

template <typename T>
void print_lines(const T &outlist, bool print = true,
                 const std::string &prefix = "")
{
  if (print) {
    for (auto val : outlist) {
      debug(val, prefix);
    }
  }
}

template <typename A, typename B>
std::ostream& operator<<(std::ostream &out, const std::pair<A,B> &p) {
  out << "(" << p.first << ", " << p.second << ")";
  return out;
}

template <typename T>
std::vector<T> str_split(const std::string &instr) {
  if (instr == "") {
    return {};
  }
  std::vector<T> vals;
  std::istringstream streamer;
  streamer.str(instr);
  while(!streamer.eof()) {
    T val;
    streamer >> val;
    vals.emplace_back(val);
  }
  return vals;
}

template <typename A, typename B>
std::vector<A> map_keys(const std::map<A,B> &m) {
  std::vector<A> keys;
  for (auto &ab : m) {
    keys.emplace_back(ab.first);
  }
  return keys;
}

template <typename A, typename B>
std::vector<A> map_keys(const std::unordered_map<A,B> &m) {
  std::vector<A> keys;
  for (auto &ab : m) {
    keys.emplace_back(ab.first);
  }
  return keys;
}

template <typename A, typename B>
std::vector<B> map_values(const std::map<A,B> &m) {
  std::vector<B> values;
  for (auto &ab : m) {
    values.emplace_back(ab.second);
  }
  return values;
}

template <typename A, typename B>
std::vector<B> map_values(const std::unordered_map<A,B> &m) {
  std::vector<B> values;
  for (auto &ab : m) {
    values.emplace_back(ab.second);
  }
  return values;
}

struct Point {
  int x; int y;
  Point(int _x, int _y) : x(_x), y(_y) {}
};
using PointList = std::vector<Point>;
std::ostream& operator<<(std::ostream &out, const Point &p) {
  return out << "(" << p.x << ", " << p.y << ")";
}

struct Node {
  using NodePtrList = std::vector<Node*>;
  Point p;
  NodePtrList neighbors;
  Node(const Point &_p) : p(_p) {}
};
using NodeList = std::vector<Node>;
using NodePtrList = std::vector<Node*>;
std::ostream& operator<<(std::ostream &out, const Node &n) {
  return out << "Node{" << n.p << "} -- " << &n;
}

bool operator==(const Node &a, const Node &b) {
  return a.p == b.p;
}

struct Grid {
  uint R;
  uint C;
  stringlist g;
  Node *start = nullptr;
  Node *goal = nullptr;
  NodeList nodes;
  std::map<Node*, int> distances;
  uint reachable_count = 0;
  double avg_reachable_distance = 0;

  Grid (const stringlist &inlist) {
    start = nullptr;
    goal = nullptr;
    debug("Grid constructor");
    // read in # rows and # cols
    auto config = str_split<uint>(inlist[0]);
    my_assert(config.size() == 2);
    R = config[0];
    C = config[1];
    auto C_copy = C;
    debug("Done with config");

    // Parse the nodes to make a nodes list
    g.insert(g.begin(), inlist.begin()+1, inlist.end());
    for (int y = 0; y < g.size(); y++) {
      auto row = g[y];
      for (int x = 0; x < row.length(); x++) {
        if (row[x] != '.') {
          nodes.emplace_back(Point{x, y});
        }
      }
    }
    // Find the start and the goal
    for (Node& node : nodes) {
      if (g[node.p.y][node.p.x] == '^') {
        start = &node;
      } else if (g[node.p.y][node.p.x] == '$') {
        goal = &node;
      }
    }
    debug("Done finding nodes");
    my_assert(start != nullptr);
    my_assert(goal != nullptr);
    debug("Done with node asserts");

    // Make sure the grid follows the rows and column specified
    my_assert(g.size() == R);
    my_assert(std::all_of(g.begin(), g.end(), [&C_copy](const std::string &x) {
        return x.length() == C_copy;
      }));
    debug("Done with grid asserts");

    debug("\nNodes:");
    print_lines(nodes, DEBUG, "  ");

    // Connect up the nodes together to make a graph
    // First do by row
    for (int y = 0; y < R; y++) {
      Node* current_node = nullptr;
      for (int x = 0; x < C; x++) {
        auto it = std::find_if(nodes.begin(), nodes.end(),
                               [x,y](auto n) {return n.p.x == x && n.p.y == y;});
        if (it != nodes.end()) {
          Node *n = &(*it);
          if (current_node != nullptr) {
            n->neighbors.push_back(current_node);
            current_node->neighbors.push_back(n);
          }
          current_node = n;
        }
      }
    }
    debug("\nDone with row matching for graphs");
    // Now do by column
    for (int x = 0; x < C; x++) {
      Node* current_node = nullptr;
      for (int y = 0; y < R; y++) {
        auto it = std::find_if(nodes.begin(), nodes.end(),
                               [x,y](auto n) {return n.p.x == x && n.p.y == y;});
        if (it != nodes.end()) {
          Node *n = &(*it);
          if (current_node != nullptr) {
            n->neighbors.push_back(current_node);
            current_node->neighbors.push_back(n);
          }
          current_node = n;
        }
      }
    }
    debug("Done with col matching for graphs");

    debug("\nConnections:");
    for (auto &n : nodes) {
      debug(n);
      for (auto n2 : n.neighbors) {
        debug(n2, "  ");
      }
    }
    
    // debug("\nCompute distances:");
    // std::queue<std::pair<Node*, int>> frontier;
    // std::set<Node*> visited;
    // frontier.push({goal, 0});
    // visited.insert(goal);
    // while (!frontier.empty()) {
    //   auto pair = frontier.front();
    //   frontier.pop();
    //   auto n = pair.first;
    //   auto d = pair.second;
    //   debug(pair, "  popped - ");
    //   distances[n] = d;
    //   reachable_count += 1;
    //   avg_reachable_distance += d;
    //   for (auto next : n->neighbors) {
    //     if (visited.count(next) == 0) {
    //       visited.insert(next);
    //       frontier.push({next, d+1});
    //     }
    //   }
    // }
    // for (auto &n : nodes) {
    //   if (distances.count(&n) == 0) {
    //     distances[&n] = std::numeric_limits<int>::max();
    //   }
    // }
    // avg_reachable_distance /= reachable_count;
    // for (auto &pair : distances) {
    //   debug(pair, "  distance: ");
    // }
    // debug(reachable_count, "  reachable count: ");
    // debug(avg_reachable_distance, "  average reachable distance: ");
    
    debug("Done creating Grid");
  }

  std::map<Node*, double> value_iteration(double tolerance = 1e-6) {
    std::map<Node*, double> values;
    bool update_happened = true;
    auto N = nodes.size();
    int iter_count = 0;
    while (update_happened) {
      update_happened = false;

      double avg_value = std::accumulate(values.begin(), values.end(), 0.0,
              [](double val, const std::pair<Node*, double> &pair) {
                return val + pair.second;
              }) / N;

      debug("Value iteration " + std::to_string(iter_count));
      print_lines(values, DEBUG, "  ");
      for (Node &n : nodes) {
        Node* node = &n;
        double val = values[node];
        // Populate possible_values
        std::vector<double> possible_values;
        if (node == this->goal) {
          possible_values.push_back(0.0);
        }
        possible_values.push_back(avg_value + 1);  // value of random move
        for (Node *neighbor : node->neighbors) {
          possible_values.push_back(values[neighbor] + 1); // value of adjacent move
        }
        double new_val = *std::min_element(possible_values.begin(),
                                           possible_values.end());
        values[node] = new_val;
        if (std::abs(new_val - val) > tolerance) {
          update_happened = true;
        }
      }
      iter_count++;
    }
    return values;
  }
};

int main() {
  debug("Read Input");
  auto inlist = read_input();
  Grid grid(inlist);
  debug(grid.start, "Start: ");
  debug(grid.goal,  "Goal:  ");

  // // Create the optimal policy
  // uint seed = std::chrono::system_clock::now().time_since_epoch().count();
  // std::mt19937 generator(seed);
  // std::uniform_int_distribution<int> distribution(0, grid.nodes.size() - 1);
  // auto random_node = [&grid, &generator, &distribution]() {
  //   return &grid.nodes[distribution(generator)];
  // };
  // auto policy = [&grid, &random_node, far_away](Node *n) {
  //   auto dist = grid.distances[n];
  //   if (dist > far_away) {
  //     return random_node();
  //   }
  //   for (Node *next : n->neighbors) {
  //     if (grid.distances[next] == dist - 1) {
  //       return next;
  //     }
  //   }
  //   throw std::runtime_error("Policy had an internal error");
  // };

  // // Run simulations to test our hypothesis
  // auto simulate = [&policy](Node *start, Node *goal) {
  //   Node *current = start;
  //   int jumps = 0;
  //   while (current != goal) {
  //     current = policy(current);
  //     jumps += 1;
  //   }
  //   return jumps;
  // };
  // long total_jumps = 0;
  // int runs = 3000000;
  // for (int i = 1; i <= runs; i++) {
  //   total_jumps += simulate(grid.start, grid.goal);
  // }
  // double monte_carlo_results = double(total_jumps) / runs;
  // debug(monte_carlo_results, "Simulation Results:   ");

  auto values = grid.value_iteration(1e-4);
  debug(values[grid.start], "Value Iteration:      ");

  std::cout << std::setprecision(3) << std::fixed
            << values[grid.start] << std::endl;
  return 0;
}

