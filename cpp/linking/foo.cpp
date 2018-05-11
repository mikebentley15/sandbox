#include "foo.h"

namespace {
inline int unnamed_namespace_inline_func() { return 3; }
int unnamed_namespace_func() { return 4; }
}

static inline int static_inline_func() { return 5; }
static int static_func() { return 6; }
inline int inline_func() { return 7; }
int func() {
  // 75 = sum([8,9,10,11,12,3,4,5,6,7])
  return unnamed_namespace_inline_hfunc() +
         unnamed_namespace_hfunc() +
         static_inline_hfunc() +
         static_hfunc() +
         inline_hfunc() +
         unnamed_namespace_inline_func() +
         unnamed_namespace_func() +
         static_inline_func() +
         static_func() +
         inline_func();
}
