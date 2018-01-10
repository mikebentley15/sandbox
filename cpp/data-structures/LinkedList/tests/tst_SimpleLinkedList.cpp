#include "test_harness.h"

#include "SimpleLinkedList.h"

void tst_SimpleLinkedList_constructor() {
  // test that the default constructor does not throw an exception
  SimpleLinkedList<int> ll;
}
TH_REGISTER(tst_SimpleLinkedList_constructor);
