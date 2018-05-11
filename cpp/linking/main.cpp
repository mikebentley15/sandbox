#include "foo2.h"

int main() {
  return unnamed_namespace_inline_hfunc() +
         unnamed_namespace_hfunc() +
         static_inline_hfunc() +
         static_hfunc() +
         inline_hfunc() +
         func();
}
