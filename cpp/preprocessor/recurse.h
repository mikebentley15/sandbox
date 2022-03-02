// See:
// - https://github.com/pfultz2/Cloak/wiki/C-Preprocessor-tricks,-tips,-and-idioms
// - http://saadahmad.ca/
// - https://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html

#include "defer.h"
#include "eval.h"
#include "detect_paren.h"

#define HEAD(x, ...) x
#define TAIL(x, ...) __VA_ARGS__
//#define _REC_EMPTY() // do nothing

// extra indirection to ensure expansion of __VA_ARGS__
#define CAT(a, ...) a ## __VA_ARGS__
#define IIF(cond) CAT(IIF_, cond)
#define IIF_0(t, f) f
#define IIF_1(t, f) t

#define IS_0(x) IS_PAREN(CAT(IS_0_, x))
#define IS_0_0 _PROBE(~)

// negation
#define NOT(b) CAT(_NOT_OF_, b)
#define _NOT_OF_0 1
#define _NOT_OF_1 0

// and
#define BITAND(x, y) EVAL_1(CAT(_BITAND_WITH_, x)(y))
#define _BITAND_WITH_0(y) 0
#define _BITAND_WITH_1(y) y

// increment (for x in {0..64})
#define INC(x) CAT(INC_, x)
#define INC_0   1
#define INC_1   2
#define INC_2   3
#define INC_3   4
#define INC_4   5
#define INC_5   6
#define INC_6   7
#define INC_7   8
#define INC_8   9
#define INC_9  10
#define INC_10 11
#define INC_11 12
#define INC_12 13
#define INC_13 14
#define INC_14 15
#define INC_15 16
#define INC_16 17
#define INC_17 18
#define INC_18 19
#define INC_19 20
#define INC_20 21
#define INC_21 22
#define INC_22 23
#define INC_23 24
#define INC_24 25
#define INC_25 26
#define INC_26 27
#define INC_27 28
#define INC_28 29
#define INC_29 30
#define INC_30 31
#define INC_31 32
#define INC_32 33
#define INC_33 34
#define INC_34 35
#define INC_35 36
#define INC_36 37
#define INC_37 38
#define INC_38 39
#define INC_39 40
#define INC_40 41
#define INC_41 42
#define INC_42 43
#define INC_43 44
#define INC_44 45
#define INC_45 46
#define INC_46 47
#define INC_47 48
#define INC_48 49
#define INC_49 50
#define INC_50 51
#define INC_51 52
#define INC_52 53
#define INC_53 54
#define INC_54 55
#define INC_55 56
#define INC_56 57
#define INC_57 58
#define INC_58 59
#define INC_59 60
#define INC_60 61
#define INC_61 62
#define INC_62 63
#define INC_63 64
#define INC_64 65
// note: if you try to increment higher, compiler error

// decrement (for x in {0..64})
#define DEC(x) CAT(DEC_, x)
#define DEC_64 63
#define DEC_63 62
#define DEC_62 61
#define DEC_61 60
#define DEC_60 59
#define DEC_59 58
#define DEC_58 57
#define DEC_57 56
#define DEC_56 55
#define DEC_55 54
#define DEC_54 53
#define DEC_53 52
#define DEC_52 51
#define DEC_51 50
#define DEC_50 49
#define DEC_49 48
#define DEC_48 47
#define DEC_47 46
#define DEC_46 45
#define DEC_45 44
#define DEC_44 43
#define DEC_43 42
#define DEC_42 41
#define DEC_41 40
#define DEC_40 39
#define DEC_39 38
#define DEC_38 37
#define DEC_37 36
#define DEC_36 35
#define DEC_35 34
#define DEC_34 33
#define DEC_33 32
#define DEC_32 31
#define DEC_31 30
#define DEC_30 29
#define DEC_29 28
#define DEC_28 27
#define DEC_27 26
#define DEC_26 25
#define DEC_25 24
#define DEC_24 23
#define DEC_23 22
#define DEC_22 21
#define DEC_21 20
#define DEC_20 19
#define DEC_19 18
#define DEC_18 17
#define DEC_17 16
#define DEC_16 15
#define DEC_15 14
#define DEC_14 13
#define DEC_13 12
#define DEC_12 11
#define DEC_11 10
#define DEC_10  9
#define DEC_9   8
#define DEC_8   7
#define DEC_7   6
#define DEC_6   5
#define DEC_5   4
#define DEC_4   3
#define DEC_3   2
#define DEC_2   1
#define DEC_1   0
#define DEC_0  -1 // note: if you try to decrement further, compiler error

#define BOOL(x) NOT(IS_0(x))
#define IF(c) IIF(BOOL(c))
#define EXPAND(...) __VA_ARGS__
#define EAT(...)
#define WHEN(c) IF(c)(EXPAND, EAT)

// given macro is given args (i, ...) where i is the repeat index
#define REPEAT(count, macro, ...) EVAL(_REPEAT(count, macro, __VA_ARGS__))
#define _REPEAT_INDIRECT() _REPEAT
#define _REPEAT(count, macro, ...) \
  WHEN(count) \
  ( \
    DEFER2(_REPEAT_INDIRECT)() (DEC(count), macro, __VA_ARGS__) \
    DEFER2(macro)(DEC(count), __VA_ARGS__) \
  )

// keep applying op(args) until pred(args) is true
// like this python pseudocode:
//   def macro_while(pred, op, *args):
//       while pred(args):
//           args = op(args)
//       print(args)
#define WHILE(pred, op, ...) EVAL(_WHILE(pred, op, __VA_ARGS__))
#define _WHILE_INDIRECT() _WHILE
#define _WHILE(pred, op, ...) \
  IF(pred(__VA_ARGS__)) \
  ( \
    DEFER2(_WHILE_INDIRECT)() (pred, op, op(__VA_ARGS__)), \
    __VA_ARGS__ \
  )

// Returns the number of arguments passed in, up to 64
#define _NUM_ARGS_IMPL(X,X64,X63,X62,X61,X60,X59,X58,X57,X56,X55,X54,X53,X52,X51,X50,X49,X48,X47,X46,X45,X44,X43,X42,X41,X40,X39,X38,X37,X36,X35,X34,X33,X32,X31,X30,X29,X28,X27,X26,X25,X24,X23,X22,X21,X20,X19,X18,X17,X16,X15,X14,X13,X12,X11,X10,X9,X8,X7,X6,X5,X4,X3,X2,X1,N,...) N
#define NUM_ARGS(...) _NUM_ARGS_IMPL(0, ##__VA_ARGS__ ,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)

// if the arguments are empty, then HEAD(__VA_ARGS__) is ill-formed
//   HEAD(): ill-formed
// 
#define IS_NOT_EMPTY(...) BOOL(NUM_ARGS(__VA_ARGS__))
#define IS_EMPTY(...) NOT(IS_NOT_EMPTY(__VA_ARGS__))

// python-like pseudo-code:
// def macro_for_each(macro, *args):
//     for x in args:
//         print(macro(x))
#define FOR_EACH(macro, ...) EVAL(_FOR_EACH(macro, __VA_ARGS__))
#define _FOR_EACH_INDIRECT() _FOR_EACH
#define _FOR_EACH(macro, ...) \
  WHEN(IS_NOT_EMPTY(__VA_ARGS__)) \
  ( \
    DEFER2(macro)(HEAD(__VA_ARGS__)) \
    DEFER2(_FOR_EACH_INDIRECT)() (macro, TAIL(__VA_ARGS__)) \
  )

// python-like pseudo-code:
// def macro_for_each(macro, *args):
//     iterator = iter(args)
//     while iterator:
//         x = next(iterator)
//         y = next(iterator)
//         print(macro(x, y))
#define FOR_EACH_2(macro, ...) EVAL(_FOR_EACH_2(macro, __VA_ARGS__))
#define _FOR_EACH_INDIRECT_2() _FOR_EACH_2
#define _FOR_EACH_2(macro, ...) \
  WHEN(IS_NOT_EMPTY(__VA_ARGS__)) \
  ( \
    DEFER2(macro)(HEAD(__VA_ARGS__), DEFER(HEAD)(TAIL(__VA_ARGS__))) \
    DEFER2(_FOR_EACH_INDIRECT_2)() (macro, DEFER(TAIL)(TAIL(__VA_ARGS__))) \
  )

// removes parens around a single argument (if they're there)
#define REMOVE_PARENS(x) IF(IS_PAREN(x))(EXPAND x,x)
