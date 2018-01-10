#include "SinglyLinkedList.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>   // for exit() and malloc()

#ifndef NOT_IMPL
#define NOT_IMPL() \
  fputs("Function \"", stderr); \
  fputs(__func__, stderr); \
  fputs("\" is not implemented -- exiting\n", stderr); \
  exit(1)
#endif // NOT_IMPL

SLL* SLL_create(void) {
  SLL* list = (SLL*) malloc(sizeof(SLL));
  if (list != NULL) {
    list->head = NULL;
    list->size = 0;
  }
  return list;
}

void  SLL_del(SLL** const listptr) {
  SLL_clear(*listptr);
  free(*listptr);
  *listptr = NULL;
}

void  SLL_insert(SLL* const list, void* value, int idx) {
  assert(0 <= idx && idx <= list->size);

  // Create the new node
  SLLNode* new_node = (SLLNode*) malloc(sizeof(SLLNode));
  new_node->value = value;

  // Find the before and after
  SLLNode* before = NULL;
  SLLNode* after = list->head;
  for (; idx > 0; idx--) {
    before = after;
    after = after->next;
  }

  // Connect the before node to the new one
  if (before != NULL) {
    before->next = new_node;
  } else {
    list->head = new_node;
  }

  // Connect the new node to the after one
  new_node->next = after;

  // Increase the size
  list->size++;
}

void  SLL_push(SLL* const list, void* value) {
  SLL_insert(list, value, 0);
}

void* SLL_remove(SLL* const list, int idx) {
  assert(0 <= idx && idx < list->size);

  // Find the location to remove
  SLLNode* before = NULL;
  SLLNode* current = list->head;
  for (; idx > 0; idx--) {
    before = current;
    current = current->next;
  }

  if (before != NULL) {
    before->next = current->next;
  } else {
    list->head = current->next;
  }
  void* value = current->value;
  free(current);
  list->size--;
  return value;
}

void* SLL_pop(SLL* const list) {
  return SLL_remove(list, 0);
}

void  SLL_clear(SLL* const list) {
  SLLNode* current = list->head;
  while (current != NULL) {
    SLLNode* next = current->next;
    current->next = NULL;
    free(current);
    current = next;
  }
  list->head = NULL;
  list->size = 0;
}

SLL   SLL_iter_at(const SLL* const list, int idx) {
  assert(0 <= idx && idx <= list->size);

  SLL iter;
  iter.size = list->size - idx;
  iter.head = list->head;
  for (; idx > 0; idx--) {
    iter.head = iter.head->next;
  }
  return iter;
}

void* SLL_at(const SLL* const list, int idx) {
  assert(0 <= idx && idx < list->size);
  SLLNode* current = list->head;
  for (; idx > 0; idx--) {
    current = current->next;
  }
  return current->value;
}
