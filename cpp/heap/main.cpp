#include "MinHeap.h"

#include <iostream>
#include <cstdio>

static int n_constructor_calls = 0;
static int n_copy_constructor_calls = 0;
static int n_move_constructor_calls = 0;
static int n_destructor_calls = 0;
static int n_copy_assignment_calls = 0;
static int n_move_assignment_calls = 0;

struct A {
  int x;
  bool operator < (const A &other) const { return this->x < other.x; }

  static void print_static_stats() {
    std::cout <<
      "  # A():      " << n_constructor_calls << "\n"
      "  # A(copy):  " << n_copy_constructor_calls << "\n"
      "  # A(move):  " << n_move_constructor_calls << "\n"
      "  # A=(copy): " << n_copy_assignment_calls << "\n"
      "  # A=(move): " << n_move_assignment_calls << "\n"
      "  # ~A():     " << n_destructor_calls << "\n";
  }

  // print how much we construct, move, and copy
  A(int _x) : x(_x)              { std::puts("A()");     ++n_constructor_calls;      }
  A(const A &other) : x(other.x) { std::puts("A(copy)"); ++n_copy_constructor_calls; }
  A(A &&other) : x(other.x)      { std::puts("A(move)"); ++n_move_constructor_calls; }
  ~A()                           { std::puts("~A()");    ++n_destructor_calls;       }
  A& operator = (const A &other) {
    std::puts("A=(copy)");
    this->x = other.x;
    ++n_copy_assignment_calls;
    return *this;
  }
  A& operator = (A &&other) {
    std::puts("A=(move)");
    this->x = other.x;
    ++n_move_assignment_calls;
    return *this;
  }
};

std::ostream& operator<< (std::ostream &out, const A &a) {
  return out << "A{" << a.x << "}";
}

template <typename T>
std::ostream& operator<< (std::ostream &out, const std::vector<T> &vals) {
  out << "[";
  bool first = true;
  for (const auto &val : vals) {
    if (!first) { out << ", "; }
    first = false;
    out << val;
  }
  out << "]";
  return out;
}

template <typename T, typename ...Args>
std::vector<T> make_vector(Args &&...args) {
  static_assert(
      (std::is_convertible_v<Args, T> && ...),
      "All arguments to make_vector() must be implicitly convertible to T"
  );
  std::vector<T> vec;
  vec.reserve(sizeof...(Args));
  (vec.emplace_back(std::forward<Args>(args)), ...);
  return vec;
}

void heap_sort_test() {
  auto vals = make_vector<A>(3, 1, 5, 3, 4, 2);
  std::cout << "Before heapify: " << vals << std::endl;
  MinHeap<A> heap(std::move(vals));
  std::cout << "After  heapify: " << heap.data() << std::endl;
  vals = std::vector<A>();
  vals.reserve(6);
  vals.emplace_back(heap.pop());
  std::cout << "Heap-sorting  : " << vals << std::endl;
  vals.emplace_back(heap.pop());
  std::cout << "Heap-sorting  : " << vals << std::endl;
  vals.emplace_back(heap.pop());
  std::cout << "Heap-sorting  : " << vals << std::endl;
  vals.emplace_back(heap.pop());
  std::cout << "Heap-sorting  : " << vals << std::endl;
  vals.emplace_back(heap.pop());
  std::cout << "Heap-sorting  : " << vals << std::endl;
  vals.emplace_back(heap.pop());
  std::cout << "Heap-sorted   : " << vals << std::endl;
}

int main() {
  heap_sort_test();
  A::print_static_stats();
}
