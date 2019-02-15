#include <iostream>
#include <iomanip>

#define UNUSED_VAR(x) (void)x

float dot(float* x, float* y, int n);

int main(int argCount, char *argList[]) {
  UNUSED_VAR(argCount);
  UNUSED_VAR(argList);

  // true dot product   = 16877222
  // unoptimized output = 16877216
  int n = 8;
  float x[n] { 16777216.0f, 1.f, 1.f, 1.f, 1e5f, 1.f, 1.f, 1.f };
  float y[n] {      1.f,    1.f, 1.f, 1.f, 1.f,  1.f, 1.f, 1.f };
  
  std::cout << std::setprecision(100);
  std::cout << dot(x, y, n) << std::endl;
  return 0;
}
