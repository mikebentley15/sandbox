#include "detect_paren.h"

#define PRINT_EXPR(...) #__VA_ARGS__ == __VA_ARGS__

PRINT_EXPR(_CHECK(_PROBE(~)))
#if _CHECK(_PROBE(~)) != 1
#  error _CHECK(_PROBE(~)) != 1
#endif

PRINT_EXPR(_CHECK(xxx))
#if _CHECK(xxx) != 0
#  error _CHECK(xxx) != 0
#endif

PRINT_EXPR(IS_PAREN(()))
#if IS_PAREN(()) != 1
#  error IS_PAREN(()) != 1
#endif

PRINT_EXPR(IS_PAREN(xxx))
#if IS_PAREN(xxx) != 0
#  error IS_PAREN(xxx)
#endif
