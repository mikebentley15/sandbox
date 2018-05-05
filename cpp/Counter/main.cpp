#include "Counter.h"

#include <iostream>
#include <string>

template<typename T, template<class,class...> class Cont, class... Args>
std::ostream& operator<<(std::ostream& out, const Cont<T, Args...> &c) {
  bool first = true;
  out << "[";
  for (const auto &x : c) {
    if (!first) out << ",";
    first = false;
    out << x;
  }
  out << "]";
  return out;
}

template <typename A, typename B>
std::ostream& operator<<(std::ostream& out, const std::pair<A,B> &p) {
  return out << "(" << p.first << "," << p.second << ")";
}

int main() {
  std::string a("hello there my helper friend");
  Counter<char> c1(a.begin(), a.end());
  UnorderedCounter<char> c2(a.begin(), a.end());

  std::cout << "c1: " << c1 << std::endl;
  std::cout << "c2: " << c2 << std::endl;

  std::cout << "c3: " << Counter<int>{1,2,3,1,3,1,3,2,3,3,3,2,3,1} << std::endl;
  std::cout << "c4: " << UnorderedCounter<int>{1,2,3,1,3,1,3,2,3,3,3,2,3,1} << std::endl;

  return 0;
}
