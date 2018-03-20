#include <stdio.h>

extern int defined_function(int a);

int main() {
  printf("defined_function(2) = %d\n", defined_function(2));
  return 0;
}
