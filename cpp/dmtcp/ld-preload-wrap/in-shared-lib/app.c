#include <stdio.h>

int add(int a, int b);

int main(int argCount, char** argList) {
  printf("Before adding the first time\n");
  printf("  3 + 4 = %d\n", add(3, 4));
  printf("Before adding the second time\n");
  printf("  4 + 5 = %d\n", add(4, 5));
  return 0;
}
