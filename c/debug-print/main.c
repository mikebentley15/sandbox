#include "debug-print.h"

#include <time.h>

void test_debug(int n) {
  for (int i = 0; i < n; i++) {
    debug("hello %s %s%s\n", "my", "friend", "!");
  }
}

void test_mdebug(int n) {
  for (int i = 0; i < n; i++) {
    mdebug("hello %s %s%s\n", "my", "friend", "!");
  }
}

void test_empty(int n) {
  for (int i = 0; i < n; i++) {
  }
}

typedef void (*func_to_time)(int);
double time_func(func_to_time f, int n) {
  clock_t begin = clock();
  f(n);
  clock_t end = clock();
  return (double)(end - begin) / CLOCKS_PER_SEC;
}

int main(void) {
#ifndef DEBUG
  const int n = 1e9;
#else
  const int n = 50000;
#endif
  double debug_time = time_func(test_debug, n);
  double mdebug_time = time_func(test_mdebug, n);
  double empty_time = time_func(test_empty, n);
  printf("%d iterations\n", n);
  printf("timing of test_debug            = %lf sec\n", debug_time);
  printf("timing of test_mdebug           = %lf sec\n", mdebug_time);
  printf("timing of empty function        = %lf sec\n", empty_time);
  printf("corrected timing of test_debug  = %lf sec\n", debug_time - empty_time);
  printf("corrected timing of test_mdebug = %lf sec\n", mdebug_time - empty_time);
  return 0;
}
