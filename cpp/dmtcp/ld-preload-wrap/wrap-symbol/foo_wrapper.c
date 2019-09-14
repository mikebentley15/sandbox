#include <stdio.h>

void __real_foo();

void foo() {
  printf("Entering __wrapper_foo\n");
  __real_foo();
  printf("Exiting __wrapper_foo\n");
}
