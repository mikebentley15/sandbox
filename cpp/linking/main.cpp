#include "foo2.h"

int main() {
  // Want this to be
  //   0 = sum([8,9,10,11,12,3,4,5,6,7]) - sum([13,14,15,16,17])
  // But can end up being
  //   5 = sum([8,9,10,11,17,3,4,5,6,7]) - sum([13,14,15,16,17])
  // Or it can be
  //   5 = sum([8,9,10,11,12,3,4,5,6,7]) - sum([13,14,15,16,12])
  return func() -
         (
           unnamed_namespace_inline_hfunc() +
           unnamed_namespace_hfunc() +
           static_inline_hfunc() +
           static_hfunc() +
           inline_hfunc()
         );
}
