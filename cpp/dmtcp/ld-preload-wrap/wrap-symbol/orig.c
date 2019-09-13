#include <stdio.h>

void foo() {
  printf("Inside foo\n");
}

int main() {
  printf("Entering main\n");
  foo();
  printf("Exiting main\n");
  return 0;
}
