#include "test_harness.h"

#include "LinkedList.h"

void tst_LinkedList_constructor() {
  // test that the default constructor does not throw an exception
  LinkedList<int> ll;
}
TH_REGISTER(tst_LinkedList_constructor);
