#ifndef LIBRARY_H
#define LIBRARY_H

#include <iomanip>
#include <iostream>
#include <map>
#include <string>

inline int& get_int() {
  static int x = 0;
  return x;
}

void print_int();
inline void print_int_inline() {
  std::cout
    << "  from inline:            x = " << get_int()
    << std::hex << std::setiosflags(std::ios::showbase)
    << "   (ptr = " << &get_int() << ")"
    << std::dec << std::resetiosflags(std::ios::showbase)
    << std::endl;
}

inline std::map<std::string, void*>& get_tests() {
  static std::map<std::string, void*> tests;
  return tests;
}

inline void register_test(const std::string& name, void *ptr) {
  get_tests()[name] = ptr;
}

void print_test_size();
inline void print_test_size_inline() {
  std::cout
    << "  from inline:   tests.size = " << get_tests().size()
    << std::hex << std::setiosflags(std::ios::showbase)
    << "   (ptr = " << &get_tests() << ")"
    << std::dec << std::resetiosflags(std::ios::showbase)
    << std::endl;
}

#endif // LIBRARY_H
