#include <stdio.h>

int a[8] = {1, 2, 3, 4, 5, 6, 7, 8};
const int b[8] = {1, 2, 3, 4, 5, 6, 7, 8};
int c[8];

int main() {
  printf("%p = a\n", a);
  printf("%p = b\n", b);
  printf("%p = c\n", c);
  printf("%p = main\n", main);
  return 0;
}
