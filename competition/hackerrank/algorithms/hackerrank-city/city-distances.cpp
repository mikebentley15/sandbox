#include <iostream>
#include <vector>
#include <chrono>

std::ostream debug_stream(nullptr);
void debug_enable() { debug_stream.rdbuf(std::cout.rdbuf()); }
void debug_disable() { debug_stream.rdbuf(nullptr); }

class Timer {
public:
  Timer() : start(std::chrono::high_resolution_clock::now()) {}
  double seconds() const;
private:
  std::chrono::high_resolution_clock::time_point start;
};
Timer t;

double Timer::seconds() const {
  using namespace std::chrono;
  auto stop = high_resolution_clock::now();
  return duration_cast<duration<double>>(stop - start).count();
}

std::ostream& debug() {
  return debug_stream << "debug(" << std::to_string(t.seconds()) << "): ";
}

//
// weights: 0
//
// o
//
//
// weights added: a
// total paths: 5a + 5a + 5a + 5a + 9a = 29a
//
// o  o
// |  |
// o--o
// |  |
// o  o
// 
// 
// weights added: b
// total paths:
//   25a*3*4 + 23a*3*4 + 13*13b + 6*20b*4 + 5*21b*4
//   = 576a + 1069b
//
// o  o  o  o
// |  |  |  |
// o--o  o--o
// |  |  |  |
// o  o  o  o
//    |  |
//    o--o
//    |  |
// o  o  o  o
// |  |  |  |
// o--o  o--o
// |  |  |  |
// o  o  o  o
//
//
// weights added: c
// total paths:
//   
//
//
//
// o  o  o  o  o  o  o  o
// |  |  |  |  |  |  |  |
// o--o  o--o  o--o  o--o
// |  |  |  |  |  |  |  |
// o  o  o  o  o  o  o  o
//    |  |        |  |   
//    o--o        o--o   
//    |  |        |  |   
// o  o  o  o  o  o  o  o
// |  |  |  |  |  |  |  |
// o--o  o--o  o--o  o--o
// |  |  |  |  |  |  |  |
// o  o  o  o  o  o  o  o
//          |  |
//          o--o
//          |  |
// o  o  o  o  o  o  o  o
// |  |  |  |  |  |  |  |
// o--o  o--o  o--o  o--o
// |  |  |  |  |  |  |  |
// o  o  o  o  o  o  o  o
//    |  |        |  |   
//    o--o        o--o   
//    |  |        |  |   
// o  o  o  o  o  o  o  o
// |  |  |  |  |  |  |  |
// o--o  o--o  o--o  o--o
// |  |  |  |  |  |  |  |
// o  o  o  o  o  o  o  o
//


// This is the slow solution for verification
class City {
public:
  City(int _weight) : weight(_weight) {}
  City(const City &other) : children(other.children), weight(other.weight) {}

  // Make this node the root node
  void restructure() {
    children.emplace_back(parent);
    // TODO: finish
  }
private:
  std::shared_ptr<City> parent;
  std::vector<std::shared_ptr<City>> children;
  int weight;
};

int main() {
  debug_enable();

  debug() << "Read the input\n";
  int n;
  std::cin >> n;
  std::vector<int> weights(n);
  for (int i = 0; i < n; i++) {
    std::cin >> weights[i];
  }
  debug() << "  done\n";

  
  return 0;
}
