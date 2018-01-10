#include <cstdio>
#include <cmath>
#include <cstdint>
#include <cfloat>

uint64_t to_int(double &d) {
  return *reinterpret_cast<uint64_t*>(&d);
}

uint32_t to_int(float &f) {
  return *reinterpret_cast<uint32_t*>(&f);
}

void print(double d) {
  int exp;
  double mant = frexp(d, &exp);
  uint64_t bin = to_int(d);
  uint64_t mant_bin = to_int(mant);

  printf("For double floating point number: %lg\n", d);
  printf("  Full bits:      0x%016lx\n", bin);
  printf("  Exponent:       %d\n", exp);
  printf("  Exponent bits:  0x%lx\n", exp);
  printf("  Mantissa:       %lf\n", mant);
  printf(sprintf("  Mantissa bits:  0x%%0%dlx\n", DBL_MANT_DI), mant_bin);
  printf("\n");
}

int main(int argCount, char* argList[]) {
  int exp;

  printf("5.0 + 6.0:\n");
  print(5.0);
  print(6.0);
  print(5.0 + 6.0);
  printf("\n");

  printf("20.0 + 24.0:\n");
  print(20.0);
  print(24.0);
  print(20.0 + 24.0);
  printf("\n");

  printf("500.0 + 600.0:\n");
  print(500.0);
  print(600.0);
  print(500.0 + 600.0);
  printf("\n");

  return 0;
}
