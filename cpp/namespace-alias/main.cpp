#include "a.h"
#include "b.h"

#include <iostream>

int main() {
  std::cout << "foo::Foo = " << foo::Foo() << "\n"
            << "bar::Foo = " << bar::Foo() << "\n"
            << "foo::Bar = " << foo::Bar() << "\n"
            << "bar::Bar = " << bar::Bar() << "\n";
  return 0;
}
