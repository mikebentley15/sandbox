#include <cstdio>

#define VAR_MACRO(...) fprintf(stderr, __VA_ARGS__)

int main(void) {
  VAR_MACRO("\n");
  VAR_MACRO("%d\n", 1);
  VAR_MACRO("%d-%d\n", 1, 2);
  VAR_MACRO("%d-%d-%d\n", 1, 2, 3);
  return 0;
}
