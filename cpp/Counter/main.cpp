#include "Counter.h"

#include <iostream>
#include <string>

int main() {
  std::string a("hello there my helper friend");
  Counter<char> c1(a.begin(), a.end());
  UnorderedCounter<char> c2(a.begin(), a.end());

  std::cout << "expression: " << a << std::endl;
  std::cout << "c1: " << c1 << std::endl;
  std::cout << "c2: " << c2 << std::endl;
  std::cout << "c1['l']: " << c1['l'] << std::endl;
  std::cout << std::endl;

  std::cout << "c3: " << Counter<int>{1,2,3,1,3,1,3,2,3,3,3,2,3,1} << std::endl;
  std::cout << "c4: " << UnorderedCounter<int>{1,2,3,1,3,1,3,2,3,3,3,2,3,1} << std::endl;

  return 0;
}
