#include "returns_void.h"

#include <iomanip>
#include <iostream>

int main() {
  auto nonvoidfunc = []() { return 1; };
  auto voidfunc = []() { };

  std::cout
    << std::boolalpha
    << "nonvoidfunc returns void?     "
      << returns_void_v<decltype(nonvoidfunc)> << std::endl
    << "nonvoidfunc returns value?    "
      << returns_value_v<decltype(nonvoidfunc)> << std::endl
    << "   voidfunc returns void?     "
      << returns_void_v<decltype(voidfunc)> << std::endl
    << "   voidfunc returns value?    "
      << returns_value_v<decltype(voidfunc)> << std::endl
    << "nonvoidfunc return is void?   "
      << std::is_same_v<return_type<decltype(nonvoidfunc)>::type, void> << std::endl
    << "   voidfunc return is void?   "
      << std::is_same_v<return_type<decltype(voidfunc)>::type, void> << std::endl
    ;

  return 0;
}
