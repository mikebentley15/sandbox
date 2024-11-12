#include "solution.h"

#include <algorithm>
#include <iostream>

#include <cstddef>

namespace {

using V = std::vector<int>;

#define MY_ASSERT_EQ(a, b) \
  if ((a) != (b)) { std::cout << #a << " != " << #b << '\n'; }

template <typename T>
void assert_vector_eq(const std::vector<T> &a, const std::vector<T> &b) {
  if (a.size() != b.size()) {
    std::cout << "a.size() != b.size() (" << a.size() << " != " << b.size() << ")\n";
    return;
  }
  for (size_t i = 0; i < a.size(); i++) {
    if (a[i] != b[i]) {
      std::cout << "a[" << i << "] != b[" << i << "] (" << a[i] << " != " << b[i] << ")\n";
    }
  }
}

template <typename T>
std::ostream& operator<<(std::ostream &out, const std::vector<T> &v) {
  out << "[";
  bool first = true;
  for (auto &val : v) {
    if (!first) { out << ", "; }
    first = false;
    out << val;
  }
  out << "]";
  return out;
}

void test_vector(V &nums) {
  std::cout << "nums (before): " << nums << "\n";
  Solution soln;
  V expected = nums;
  std::sort(expected.begin(), expected.end());
  V actual = soln.sortArray2(nums);
  //assert_vector_eq(expected, actual);
  std::cout << "nums: (after): " << nums << "\n"
           // << "sorted:        " << actual << "\n"
            << "expected:      " << expected << "\n"
            << "pass?          " << (expected == actual ? "true" : "false") << "\n"
            << std::endl;
}

void test_1() { V nums {5, 4, 3, 2, 1}; test_vector(nums); }
void test_2() { V nums {5, 2, 3, 1}; test_vector(nums); }
void test_3() { V nums {-4,0,7,4,9,-5,-1,0,-7,-1}; test_vector(nums); }
void test_4() {
  V nums {
    -74,48,-20,2,10,-84,-5,-9,11,-24,-91,2,-71,64,63,80,28,-30,-58,-11,-44,-87,-22,54,-74,-10,-55,-28,-46,29,10,50,-72,34,26,25,8,51,13,30,35,-8,50,65,-6,16,-2,21,-78,35,-13,14,23,-3,26,-90,86,25,-56,91,-13,92,-25,37,57,-20,-69,98,95,45,47,29,86,-28,73,-44,-46,65,-84,-96,-24,-12,72,-68,93,57,92,52,-45,-2,85,-63,56,55,12,-85,77,-39
  };
  test_vector(nums);
}

} // end of unnamed namespace

int main() {
  test_1();
  test_2();
  test_3();
  test_4();
}
