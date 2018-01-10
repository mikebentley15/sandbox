#ifndef TEST_MACROS_H
#define TEST_MACROS_H

#include <stdio.h>
#include <stdbool.h>

#define ASSERT_TRUE(x) \
  if (!(x)) { \
    fprintf(stderr, "%s:%d: Assert failure \"%s\"\n", \
            __func__, __LINE__, #x); \
    return false; \
  }
#define TEST(x) TEST_impl(x, #x)

int TEST_impl(bool x, char* name) {
  if (!(x)) {
    printf("TEST FAILED: %s\n", name);
  } else {
    printf("Test passed: %s\n", name);
  }
  return (x ? 0 : 1);
}

#define PRINT_RESULTS(failures) \
  if (failures > 0) { \
    printf("# test failures: %d\n", failures); \
  } else { \
    puts("All tests passed"); \
  }

#endif // TEST_MACROS_H
