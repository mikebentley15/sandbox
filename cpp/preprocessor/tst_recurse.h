#include "recurse.h"

#define M(...) (__VA_ARGS__)

REPEAT(5, M, 3 + 2)
WHILE(BOOL, DEC, 5)
FOR_EACH(M, hi, there, my , friend)
_FOR_EACH_2(M, a, b, c, d, e, f)
head: HEAD(a, b, c, d)
tail: TAIL(a, b, c, d)
head(tail): HEAD(TAIL(a, b, c, d))
defer(head) (tail): DEFER(HEAD)(TAIL(a, b, c, d))
  EVAL(DEFER(HEAD)(TAIL(a, b, c, d)))
tail(tail): TAIL(TAIL(a, b, c, d))
defer(tail) (tail): DEFER(TAIL)(TAIL(a, b, c, d))
  EVAL(DEFER(TAIL)(TAIL(a, b, c, d)))

#define _VAR_DECLARATION(x, y) REMOVE_PARENS(x) REMOVE_PARENS(y);
#define _AUTO_DECLARATION(x) auto REMOVE_PARENS(x);
#define _AS_STRING(...) #__VA_ARGS__
#define _AS_STRING_INTERMEDIATE(...) _AS_STRING(__VA_ARGS__)
#define _SECOND_AS_STRING_IN_LIST(x, y) DEFER(_AS_STRING_INTERMEDIATE)(REMOVE_PARENS(y)),
#define VARS(...) FOR_EACH_2(_VAR_DECLARATION, __VA_ARGS__)


// Create a struct with the given name and parameters.
// Up to 32 types can be given.
//
// example usage:
//   NamedTuple(
//       MyTuple,
//       int, x,
//       int, y,
//       (std::map<int, int>), mapping
//   );
//   MyTuple val {.y = 3}; // requires C++20 to initialize this way
//   val.mapping[5] = 2;
//   val.x = val.y + val.mapping.at(5);
//
// Note the parentheses around std::map since macros assume all commas (except
// when in parens) separate arguments.  You may put parentheses around any of
// the types or around the names.
//
// TODO: add comparison operators, move and copy operators, and printing.
#define NamedTuple(name, ...) \
    struct name { \
      FOR_EACH_2(_VAR_DECLARATION, __VA_ARGS__) \
      static constexpr std::string_view _names[] = {\
        FOR_EACH_2(_SECOND_AS_STRING_IN_LIST, __VA_ARGS__) \
      }; \
    }

// The same as NamedTuple, except you give default values to all variables and
// skip the types (will use auto).
//
//   AutoNamedTuple(
//       MyTuple,
//       x, 3,
//       y, 1,
//       mapping, (std::map<int, int>{})
//   );
//
// It basically defaults all types to auto, and you need to give defaults for
// each one for type inferrence.  The value can have parentheses around it.
//
// Up to 32 types can be given in this way.
//
#define AutoNamedTuple(name, ...) \
    struct name { \
      FOR_EACH(_AUTO_DECLARATION, __VA_ARGS__) \
    }

VARS(int, x)
VARS(int, x, int, y)
NamedTuple(TypeA, int, x, std::string, y, double, z);
NamedTuple(TypeB, std::vector<int>, x, (std::map<int, int>), mapping);

REMOVE_PARENS of "(hello, there)": REMOVE_PARENS((hello, there))
REMOVE_PARENS of "(hello there)": REMOVE_PARENS((hello there))
REMOVE_PARENS of "hello there": REMOVE_PARENS(hello there)

NamedTuple(MyTuple, int, x, int, y, (std::map<int, int>), mapping);
AutoNamedTuple(MyTuple, x, 3, y, 1, mapping, (std::map<int, int>{}));
