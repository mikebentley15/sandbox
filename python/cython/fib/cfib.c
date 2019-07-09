#include "cfib.h"

double inline cfib(int n) {
  int i;
  double a = 0.0;
  double b = 1.0;
  double tmp;
  for (i = 0; i < n; i++) {
    tmp = a;
    a = a + b;
    b = tmp;
  }
  return a;
}
