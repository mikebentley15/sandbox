#include "library.h"

void print_int() {
  std::cout
    << "  from lib:               x = " << get_int()
    << std::hex << std::setiosflags(std::ios::showbase)
    << "   (ptr = " << &get_int() << ")"
    << std::dec << std::resetiosflags(std::ios::showbase)
    << std::endl;
}

void print_test_size() {
  std::cout
    << "  from lib:      tests.size = " << get_tests().size()
    << std::hex << std::setiosflags(std::ios::showbase)
    << "   (ptr = " << &get_tests() << ")"
    << std::dec << std::resetiosflags(std::ios::showbase)
    << std::endl;
}
