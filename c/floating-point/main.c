#include "float.h"

#include <assert.h>
#include <stdio.h>

void print_named_Float(const char* name, const Float a);

void print_Float_settings();
void print_common_Floats();
void print_all_Floats();

void test_Float_next();
void test_Float_add();
void test_Float_mult();

int main(int argCount, char* argList[]) {
  print_Float_settings();  fputs("\n", stdout);
  print_common_Floats();   fputs("\n", stdout);
  print_all_Floats();      fputs("\n", stdout);

  fputs("Running test_Float_next()\n", stdout);  test_Float_next();
  fputs("Running test_Float_add() \n", stdout);  test_Float_add();
  fputs("Running test_Float_mult()\n", stdout);  test_Float_mult();
  return 0;
}

void print_Float_settings() {
  fputs("Sign Bits:         1\n", stdout);
  printf("Exponent Bits:     %d\n", STRUCT_FLOAT_EXP_BITS);
  printf("Mantissa Bits:     %d\n", STRUCT_FLOAT_MANT_BITS);
  printf("Exponent Bias:     %d\n", Float_exp_bias);
  printf("Exponent Max:      %d\n", Float_exp_max);
  printf("Mantissa Max:      %d\n", Float_mant_max);
}

void print_named_Float(const char* name, const Float a) {
  fputs(name, stdout);
  Float_print(a);
  fputs("\n", stdout);
}

void print_common_Floats() {
  print_named_Float("Float_zero:        ", Float_zero);
  print_named_Float("Float_minus_zero:  ", Float_minus_zero);
  print_named_Float("Float_infty:       ", Float_infty);
  print_named_Float("Float_minus_infty: ", Float_minus_infty);
  print_named_Float("Float_nan:         ", Float_nan);
  print_named_Float("Float_one:         ", Float_one);
}

void print_all_Floats() {
  puts("Positive Floats:");
  Float current = Float_zero;
  Float stop = {0, Float_exp_max, Float_mant_max};
  while (!Float_binary_equal(current, stop)) {
    print_named_Float("  ", current);
    current = Float_next(current);
  }
  print_named_Float("  ", stop);

  puts("Negative Floats:");
  current = Float_minus_zero;
  stop.sign = 1;
  while (!Float_binary_equal(current, stop)) {
    print_named_Float("  ", current);
    current = Float_next(current);
  }
  print_named_Float("  ", stop);
}

void test_Float_next() {
  const Float smallest_positive = {0, 0, 1};
  assert(Float_equal(Float_next(Float_zero), smallest_positive));

  const Float smallest_negative = {1, 0, 1};
  assert(Float_equal(Float_next(Float_minus_zero), smallest_negative));

  const Float next_after_one = {0, Float_exp_bias, 1};
  assert(Float_equal(Float_next(Float_one), next_after_one));

  Float max_positive_subnormal = {0, 0, Float_mant_max};
  Float min_positive_normal = {0, 1, 0};
  assert(Float_equal(Float_next(max_positive_subnormal), min_positive_normal));
}

void test_Float_add() {
  assert(false);
}

void test_Float_mult() {
  assert(false);
}
