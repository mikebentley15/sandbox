#include "library.h"

int main(void) {
  int& x = get_int();

  std::cout << "Before changing x:" << std::endl;
  print_int();
  print_int_inline();
  std::cout << std::endl;

  x = 10;

  std::cout << "After changing x:" << std::endl;
  print_int();
  print_int_inline();
  std::cout << std::endl;

  std::cout << "Before registering a test:" << std::endl;
  print_test_size();
  print_test_size_inline();
  std::cout << std::endl;

  register_test("alice", &x);
  register_test("bob", &x);

  std::cout << "After registering two tests:" << std::endl;
  print_test_size();
  print_test_size_inline();
  std::cout << std::endl;

  return 0;
}
