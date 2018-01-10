#include <iostream>

class A {
public:
  A(int val) : _val(val) {}
  int _val;
};

thread_local A a(3);

int main(void) {
  std::cout << "a._val = " << a._val << std::endl;
  return 0;
}
