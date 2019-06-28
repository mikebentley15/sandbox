#include <iostream>

#include <cstdio>

void other_function(const int &value) {
  if (&value == nullptr) {
    std::cout << "WARNING: value is a null pointer" << std::endl;
  }
  std::cout << "value = " << value << std::endl;
}

int main(void) {
  int *ptr = nullptr;
  int &ref = *ptr;
  printf("&ref: %p\n", static_cast<void*>(&ref));
  fflush(stdout);
  other_function(ref);
  return 0;
}
