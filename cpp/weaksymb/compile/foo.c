#include <stdio.h>

__attribute__((weak)) void foo(void) {
  printf("My weak function\n");
}
