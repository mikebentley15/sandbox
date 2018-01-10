#include "float.h"

#include <stdio.h>

/// Prints the float to the console
void Float_print(const Float a) {
  printf("Float(%d, %d, %d) = ", a.sign, a.exp, a.mant);
  if (Float_is_nan(a)) {
    fputs("NaN", stdout);
  } else if (Float_is_pinf(a)) {
    fputs("+infty", stdout);
  } else if (Float_is_ninf(a)) {
    fputs("-infty", stdout);
  } else {
    // Only print to 3 decimal places
    unsigned int mant_section = (a.mant * 500) >> (STRUCT_FLOAT_MANT_BITS - 1);
    int sign = (a.sign == 1 ? -1 : 1);
    float mant_value =
      sign * (Float_is_subnormal(a) ? 0 : 1)
      + sign * ((float)mant_section) / 1000;
    int exponent = a.exp - Float_exp_bias + (Float_is_subnormal(a) ? 1 : 0);
    float value = mant_value *
      (exponent >= 0 ? (1 << exponent) : (1 / ((float)(1 << -exponent))));

    printf("%s%d.%03d * 2^(%d) = %g",
      a.sign == 1 ? "-" : "",
      !Float_is_subnormal(a),
      mant_section,
      exponent,
      value
      );
  }
}

// Convert Float -> float
float Float_to_float(const Float a) {
  unsigned int mant_section = (a.mant * 500) >> (STRUCT_FLOAT_MANT_BITS - 1);
  int sign = (a.sign == 1 ? -1 : 1);
  float mant_value =
    sign * (Float_is_subnormal(a) ? 0 : 1)
    + sign * ((float)mant_section) / 1000;
  int exponent = a.exp - Float_exp_bias + (Float_is_subnormal(a) ? 1 : 0);
  float value = mant_value *
    (exponent >= 0 ? (1 << exponent) : (1 / ((float)(1 << -exponent))));
}

/// Returns true if a represents NaN, or Not-A-Number
bool Float_is_nan(const Float a) {
  return a.exp == Float_exp_max && a.mant != 0;
}

/// Returns true for a == +\infty or a == -\infty
bool Float_is_inf(const Float a) {
  return a.exp == Float_exp_max && a.mant == 0;
}

/// Returns true for a == +\infty
bool Float_is_pinf(const Float a) {
  return a.sign == 0 && Float_is_inf(a);
}

/// Returns true for a == -\infty
bool Float_is_ninf(const Float a) {
  return a.sign == 1 && Float_is_inf(a);
}

/// Returns true if a is in the subnormal range
bool Float_is_subnormal(const Float a) {
  return a.exp == 0;
}

/// Returns true for a == b
/// Note: zero == minus zero
/// Note: nan != nan, no matter what
/// Note: +\infty == +\infty and -\infty = -\infty
bool Float_equal(const Float a, const Float b) {
  // first line checks:  zero == minus_zero
  // second line checks: a == b for each field
  // second line: nan != nan, even if bits match
  return (a.exp == 0 && b.exp == 0 && a.mant == 0 && b.mant == 0)
         || (Float_binary_equal(a, b) && !Float_is_nan(a));
}

/// Returns true if and only if the bits match exactly
bool Float_binary_equal(const Float a, const Float b) {
  return a.sign == b.sign && a.exp == b.exp && a.mant == b.mant;
}

/// Returns the ordinal away from zero in FP steps
int Float_ord(const Float a) {
  int sign = (a.sign == 1 ? -1 : 1);
  int as_int = (a.exp << STRUCT_FLOAT_MANT_BITS) + a.mant;
  return sign * as_int;
}

Float Float_from_ord(const int ord) {
  // TODO: implement
  return (Float){0, 0, 0};
}

/// Gives the next Float value away from zero
Float Float_next(const Float a) {
  if (a.mant == Float_mant_max) {
    if (a.exp != Float_exp_max) {
      return (Float){a.sign, a.exp + 1, 0};
    }
    // If we are in NaN territory, stay in NaN territory
    return a;
  } else {
    return (Float){a.sign, a.exp, a.mant + 1};
  }
}

/// Adds two floats and returns the result
Float Float_add(const Float a, const Float b) {
  // TODO: implement
  return Float_zero;
}

Float Float_subtract(const Float a, const Float b) {
  Float neg_b = {b.sign ^ 0x1, b.exp, b.mant};
  return Float_add(a, neg_b);
}

/// Multiplies two floats and returns the result
Float Float_mult(const Float a, const Float b) {
  // TODO: implement
  return Float_zero;
}

#define FLOAT_EXP_BIAS ((1 << (STRUCT_FLOAT_EXP_BITS - 1)) - 1)
#define FLOAT_EXP_MAX  ((1 << STRUCT_FLOAT_EXP_BITS) - 1)
#define FLOAT_MANT_MAX ((1 << STRUCT_FLOAT_MANT_BITS) - 1)
const unsigned int Float_exp_bias = FLOAT_EXP_BIAS;
const unsigned int Float_exp_max = FLOAT_EXP_MAX;
const unsigned int Float_mant_max = FLOAT_MANT_MAX;
const Float Float_zero = {0, 0, 0};
const Float Float_minus_zero = {1, 0, 0};
const Float Float_infty = {0, FLOAT_EXP_MAX, 0};
const Float Float_minus_infty = {1, FLOAT_EXP_MAX, 0};
const Float Float_nan = {0, FLOAT_EXP_MAX, 1};
const Float Float_one = {0, FLOAT_EXP_BIAS, 0};
//const Float Float_eps = Float_subtract(Float_one, Float_next(Float_one));
