#include "A.h"
#include "B.h"
#include "func.h"

#include <iostream>

int main() {
  A<int> A_intwrap;
  std::cout << "A_intwrap before assignment: " << A_intwrap.val() << std::endl;
  A_intwrap.val(3);
  std::cout << "A_intwrap after assignment:  " << A_intwrap.val() << std::endl;
  std::cout << std::endl;

  A<float> A_flwrap;
  std::cout << "A_flwrap before assignment:  " << A_flwrap.val() << std::endl;
  A_flwrap.val(3.1f);
  std::cout << "A_flwrap after assignment:   " << A_flwrap.val() << std::endl;
  std::cout << std::endl;

  // This does not work.
  //B<int> B_intwrap;
  //std::cout << "A_intwrap before assignment: " << B_intwrap.val() << std::endl;
  //B_intwrap.val(3);
  //std::cout << "A_intwrap after assignment:  " << B_intwrap.val() << std::endl;
  //std::cout << std::endl;

  //B<float> B_flwrap;
  //std::cout << "A_flwrap before assignment:  " << B_flwrap.val() << std::endl;
  //B_flwrap.val(3.1f);
  //std::cout << "A_flwrap after assignment:   " << B_flwrap.val() << std::endl;
  //std::cout << std::endl;

  std::cout << "plus_one(2)   = " << plus_one(2)    << std::endl;
  std::cout << "plus_one(2.1) = " << plus_one(2.1f) << std::endl;
  return 0;
}
