#include "SinglyLinkedList.h"
#include "test_macros.h"

bool test_SLL_create(void);
bool test_SLL_del(void);
bool test_SLL_insert(void);
bool test_SLL_remove(void);
bool test_SLL_clear(void);
bool test_SLL_iter_at(void);

int test_SLL_ALL(void) {
  int failures = 0;
  failures += TEST(test_SLL_create());
  failures += TEST(test_SLL_del());
  failures += TEST(test_SLL_insert());
  failures += TEST(test_SLL_remove());
  failures += TEST(test_SLL_clear());
  failures += TEST(test_SLL_iter_at());
  PRINT_RESULTS(failures);
  return failures;
}

bool test_SLL_create(void) {
  SLL *list = SLL_create();
  ASSERT_TRUE(list != NULL);
  ASSERT_TRUE(list->head == NULL);
  ASSERT_TRUE(list->size == 0);
  SLL_del(&list);
  return true;
}

bool test_SLL_del(void) {
  SLL *list = SLL_create();
  SLL_del(&list);
  ASSERT_TRUE(list == NULL);
  // TODO: how to test del?
  return true;
}

bool test_SLL_insert(void) {
  SLL *list = SLL_create();
  int value = 3;
  SLL_insert(list, &value, 0);
  ASSERT_TRUE(1 == list->size);
  ASSERT_TRUE(value == *(int*)SLL_at(list, 0));
  // Since we stored by pointer, we can modify it here and still pull it out.
  value = 5;
  ASSERT_TRUE(value == *(int*)SLL_at(list, 0));

  int new_value = 6;
  SLL_insert(list, &new_value, 1);
  ASSERT_TRUE(2 == list->size);
  ASSERT_TRUE(value == *(int*)SLL_at(list, 0));
  ASSERT_TRUE(new_value == *(int*)SLL_at(list, 1));

  int another = 10;
  SLL_insert(list, &another, 1);
  ASSERT_TRUE(3 == list->size);
  ASSERT_TRUE(value == *(int*)SLL_at(list, 0));
  ASSERT_TRUE(another == *(int*)SLL_at(list, 1));
  ASSERT_TRUE(new_value == *(int*)SLL_at(list, 2));

  SLL_del(&list);
  return true;
}

bool test_SLL_remove(void) {
  SLL *list = SLL_create();
  
  int values[] = {1, 2, 3, 4, 5};
  for (int i = 0; i < 5; i++) {
    SLL_insert(list, values + i, 0);
  }

  // Make sure the list is correct at first
  ASSERT_TRUE(5 == list->size);
  for (int i = 0; i < 5; i++) {
    ASSERT_TRUE(values + 4 - i == SLL_at(list, i));
  }

  ASSERT_TRUE(values + 1 == SLL_remove(list, 3));
  ASSERT_TRUE(4 == list->size);
  ASSERT_TRUE(values + 0 == SLL_remove(list, 3));
  ASSERT_TRUE(3 == list->size);
  ASSERT_TRUE(values + 4 == SLL_remove(list, 0));
  ASSERT_TRUE(2 == list->size);
  ASSERT_TRUE(values + 2 == SLL_remove(list, 1));
  ASSERT_TRUE(1 == list->size);
  ASSERT_TRUE(values + 3 == SLL_remove(list, 0));
  ASSERT_TRUE(0 == list->size);
  SLL_del(&list);
  return true;
}

bool test_SLL_clear(void) {
  SLL *list = SLL_create();

  int values[] = {1, 2, 3, 4, 5};
  for (int i = 0; i < 5; i++) {
    SLL_insert(list, values + i, 0);
  }

  // Make sure the list is correct at first
  ASSERT_TRUE(5 == list->size);
  for (int i = 0; i < 5; i++) {
    ASSERT_TRUE(values + 4 - i == SLL_at(list, i));
  }

  SLL_clear(list);
  ASSERT_TRUE(0 == list->size);
  ASSERT_TRUE(NULL == list->head);

  SLL_del(&list);
  return true;
}

bool test_SLL_iter_at(void) {
  SLL *list = SLL_create();

  SLL itr = SLL_iter_at(list, 0);
  ASSERT_TRUE(NULL == itr.head);
  ASSERT_TRUE(0 == itr.size);

  int values[] = {1, 2, 3, 4, 5};
  for (int i = 4; i >= 0; i--) {
    SLL_push(list, values + i);
  }
  
  itr = SLL_iter_at(list, 0);
  for (int i = 0; i < 4; i++) {
    ASSERT_TRUE(itr.head->value == values + i);
    ASSERT_TRUE(itr.size == 5 - i);
    itr = SLL_iter_at(&itr, 1);
  }

  SLL_del(&list);
  return true;
}

