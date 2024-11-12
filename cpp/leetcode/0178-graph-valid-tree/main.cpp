#include "TreeChecker.h"

#include <iostream>

size_t g_num_asserts = 0;
size_t g_failed_asserts = 0;

#define ASSERT_EQ(a, b) assert_equal(a, b, #a, #b, __LINE__)

template <typename A, typename B>
void assert_equal(
    const A &a, const B &b,
    const char *astr, const char *bstr,
    int line)
{
  ++g_num_asserts;
  if (!(a == b)) {
    std::cout << "Assert FAIL on line " << line << ": "
              << astr << " != " << bstr << " (" << a << " != " << b << ")\n";
    ++g_failed_asserts;
  }
}

void test() {
  TreeChecker checker;
  ASSERT_EQ(checker.validTree(-1, {}), false);
  ASSERT_EQ(checker.validTree(0, {}), true);
  ASSERT_EQ(checker.validTree(1, {}), true);
  ASSERT_EQ(checker.validTree(2, {}), false);

  TreeChecker::EdgeList edges;

  edges.push_back({0, 1});
  ASSERT_EQ(checker.validTree(2, edges), true);
  ASSERT_EQ(checker.validTree(3, edges), false);

  edges.push_back({1, 2});
  ASSERT_EQ(checker.validTree(3, edges), true);

  edges.push_back({0, 2});
  ASSERT_EQ(checker.validTree(3, edges), false);
  ASSERT_EQ(checker.validTree(4, edges), false);

  edges.pop_back();
  ASSERT_EQ(checker.validTree(3, edges), true);

  edges.push_back({1, 3});
  ASSERT_EQ(checker.validTree(4, edges), true);

  edges.push_back({0, 3});
  ASSERT_EQ(checker.validTree(4, edges), false);

  edges.pop_back();
  ASSERT_EQ(checker.validTree(4, edges), true);

  edges.push_back({4, 5});
  ASSERT_EQ(checker.validTree(6, edges), false);

  edges.pop_back();
  ASSERT_EQ(checker.validTree(4, edges), true);

  edges.push_back({0, 0});
  ASSERT_EQ(checker.validTree(4, edges), false);
}

void report() {
  std::cout << "Passed "
            << (g_num_asserts - g_failed_asserts) << "/" << g_num_asserts
            << " Asserts\n";
  if (g_failed_asserts > 0) {
    std::cout << "Failed " << g_failed_asserts << " Asserts\n";
  }
}

int main() {
  test();
  report();
  return g_failed_asserts;
}
