#include "multiarray_print.h"

#include <stdlib.h>

int main(int argCount, char *argList[]) {
  int n = 2;
  int m = 10;
  int p = 5;
  double (*arr2d)[m] = (double(*)[m])malloc(n*m*sizeof(double));
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      arr2d[i][j] = i * j;
    }
  }
  printArray2D(n, m, (double*)arr2d);

  double (*arr3d)[m][p] = (double(*)[m][p])malloc(n*m*p*sizeof(double));
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      for (int k = 0; k < p; k++) {
        arr3d[i][j][k] = i * j + k * j;
      }
    }
  }
  printArray3D(n, m, p, (double*)arr3d);
  return 0;
}
