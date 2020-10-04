#include <iostream>

const int Nx = 7;
const int Ny = 8;
const int Nz = 9;

inline int idx(int i, int j, int k) {
  return k + Nz * (j + Ny * i);
}

int main() {
  int a[Nx][Ny][Nz];
  int i = 1, j = 2, k = 3;
  for (int i = 0; i < Nx; i++) {
    for (int j = 0; j < Ny; j++) {
      for (int k = 0; k < Nz; k++) {
        a[i][j][k] = idx(i, j, k);
      }
    }
  }

  int *b = &a[0][0][0];
  for (int i = 0; i < Nx; i++) {
    for (int j = 0; j < Ny; j++) {
      for (int k = 0; k < Nz; k++) {
        std::cout
          << "a[" << i << "][" << j << "][" << k << "] =\t" << a[i][j][k]
          << ",\tb[" << idx(i, j, k) << "] =\t" << b[idx(i, j, k)] << std::endl;
      }
    }
  }
  return 0;
}
