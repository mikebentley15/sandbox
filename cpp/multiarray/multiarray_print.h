#include <stdio.h>

void printArray2D(int n, int m, double *arr) {
  double (*x)[m] = (double(*)[m])arr;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      printf("(%d, %d): %lf\n", i, j, x[i][j]);
    }
  }
}

void printArray3D(int n, int m, int p, double *arr) {
  double (*x)[m][p] = (double(*)[m][p])arr;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      for (int k = 0; k < p; k++) {
        printf("(%d, %d, %d): %lf\n", i, j, k, x[i][j][k]);
      }
    }
  }
}
