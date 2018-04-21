#include "A.h"
#include "func.h"

#include <iostream>

int main() {
  A<int> intwrap;
  std::cout << "intwrap before assignment: " << intwrap.val() << std::endl;
  intwrap.val(3);
  std::cout << "intwrap after assignment:  " << intwrap.val() << std::endl;
  std::cout << std::endl;

  A<float> flwrap;
  std::cout << "flwrap before assignment:  " << flwrap.val() << std::endl;
  flwrap.val(3.1f);
  std::cout << "flwrap after assignment:   " << flwrap.val() << std::endl;
  std::cout << std::endl;

  std::cout << "plus_one(2)   = " << plus_one(2)    << std::endl;
  std::cout << "plus_one(2.1) = " << plus_one(2.1f) << std::endl;
  return 0;
}
