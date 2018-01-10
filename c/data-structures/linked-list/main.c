#include "SinglyLinkedList.h"

#include <stdio.h>
#include <string.h>

int main(int argCount, char *argList[]) {
  if (argCount > 1 && 0 == strcmp("test", argList[1])) {
    return test_SLL_ALL();
  }

  SLL* list = SLL_create();
  int a = 0;
  SLL_push(list, &a);
  SLL_at(list, 0);
  SLL_del(&list);
  return 0;
}
