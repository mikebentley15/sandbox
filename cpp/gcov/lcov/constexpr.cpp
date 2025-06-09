#include <iostream>
#include <vector>

struct A {
  constexpr A() : x{42} {}

  void print() const { std::cout << x << '\n'; }

  int x;
};

inline std::ostream &operator<<(std::ostream &out, const A a) {
  return out << a.x;
}

inline int somewhatComplexFunction(int x) {
  if (x < 0) {
    return x - 42;
  }
  if (x < 5) {
    return x;
  }
  return x + 42;
}

void test_regular() {
  A a;
  a.x = somewhatComplexFunction(3);
  std::cout << a << '\n';
  a.print();
}

void test_in_vector_emplace_back() {
  std::vector<A> v;
  v.emplace_back();
}

void test_in_vector_push_back() {
  std::vector<A> v;
  v.push_back({});
}

int main() {
  // test_regular();
  test_in_vector_emplace_back();
  // test_in_vector_push_back();
  return 0;
}
