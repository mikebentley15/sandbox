#ifndef SINGLY_LINKED_LIST_H
#define SINGLY_LINKED_LIST_H

#include <stdbool.h> // for bool type
#include <stddef.h>  // for size_t

// The node type for SinglyLinkedList
typedef struct SLLNode {
  struct SLLNode *next;
  void *value;
} SLLNode;

// Just a pointer to the head node
typedef struct SinglyLinkedList {
  SLLNode *head;
  size_t size;
} SinglyLinkedList;
typedef SinglyLinkedList SLL;

// Declare functionality provided by our singly linked list
SLL*  SLL_create(void);
void  SLL_del(SLL** const listptr);
void  SLL_insert(SLL* const list, void* value, int idx);
void  SLL_push(SLL* const list, void* value);
void* SLL_remove(SLL* const list, int idx);
void* SLL_pop(SLL* const list);
void  SLL_clear(SLL* const list);
SLL   SLL_iter_at(const SLL* const list, int idx);
void* SLL_at(const SLL* const list, int idx);

// Declare test functions
int  test_SLL_ALL(void);

#endif // SINGLY_LINKED_LIST_H
