#ifndef STRUCT_FLOAT_H
#define STRUCT_FLOAT_H

#define STRUCT_FLOAT_EXP_BITS 3
#define STRUCT_FLOAT_MANT_BITS 4

#include <stdbool.h>

struct Float {
  unsigned int sign:1;
  unsigned int exp:STRUCT_FLOAT_EXP_BITS;
  unsigned int mant:STRUCT_FLOAT_MANT_BITS;
} typedef Float;

void Float_print(const Float a);
float Float_to_float(const Float a);
bool Float_is_nan(const Float a);
bool Float_is_inf(const Float a);
bool Float_is_pinf(const Float a);
bool Float_is_ninf(const Float a);
bool Float_is_subnormal(const Float a);
bool Float_equal(const Float a, const Float b);
bool Float_binary_equal(const Float a, const Float b);
int Float_ord(const Float a);
Float Float_from_ord(const int ord);
Float Float_next(const Float a);
Float Float_add(const Float a, const Float b);
Float Float_subtract(const Float a, const Float b);
Float Float_mult(const Float a, const Float b);

extern const unsigned int Float_exp_bias;
extern const unsigned int Float_exp_max;
extern const unsigned int Float_mant_max;
extern const Float Float_zero;
extern const Float Float_minus_zero;
extern const Float Float_infty;
extern const Float Float_minus_infty;
extern const Float Float_nan;
extern const Float Float_one;
//extern const Float Float_eps;

#endif // STRUCT_FLOAT_H
