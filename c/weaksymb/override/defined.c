#include <stdio.h>

int missing_function(int a) {
  printf("missing_function from %s\n", __FILE__);
  return a + 3;
}

int defined_function(int a) {
  return missing_function(a);
}
