#include <stdio.h>

void foo();

void __wrapper_foo() {
  printf("Entering __wrapper_foo\n");
  foo();
  printf("Exiting __wrapper_foo\n");
}
