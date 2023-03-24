#include "ConnectingGraph3.h"
#include "UnionFind.h"

#include <iostream>  // for std::cout

#include <cstdio>    // for std::printf()

#define ASSERT_EQUAL(x, y) assert_equal((x), (y), #x, #y, __LINE__)

namespace {

size_t g_num_asserts = 0;
size_t g_failed_asserts = 0;

template <typename T, typename U>
void assert_equal(const T &x, const U &y,
                  const char *xstr, const char *ystr, size_t line)
{
  if (x != y) {
    std::cout << "ASSERT FAILED on line " << line << ": "
              << xstr << " != " << ystr << " (" << x << " != " << y << ")\n";
    ++g_failed_asserts;
  }
  ++g_num_asserts;
}

template <typename GraphType>
void test_1() {
  const int N = 10;
  GraphType g(N);
  ASSERT_EQUAL(N, g.query());
  g.connect(0, 1); ASSERT_EQUAL(N-1, g.query());
  g.connect(1, 3); ASSERT_EQUAL(N-2, g.query());
  g.connect(0, 3); ASSERT_EQUAL(N-2, g.query());
  g.connect(1, 3); ASSERT_EQUAL(N-2, g.query());
  g.connect(1, 1); ASSERT_EQUAL(N-2, g.query());
  g.connect(0, 2); ASSERT_EQUAL(N-3, g.query());
  g.connect(4, 5); ASSERT_EQUAL(N-4, g.query());
  g.connect(3, 5); ASSERT_EQUAL(N-5, g.query());
  g.connect(20, 21); ASSERT_EQUAL(N-5, g.query()); // try connecting out of range

  // fully connected
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      g.connect(i, j);
    }
  }
  ASSERT_EQUAL(1, g.query());
}

void test_report() {
  std::cout << g_num_asserts - g_failed_asserts << "/" << g_num_asserts
            << " asserts passed\n";
  if (g_failed_asserts) {
    std::cout << g_failed_asserts << " asserts failed\n";
  }
}

} // end of unnamed namespace

int main() {
  test_1<ConnectingGraph3>();
  test_1<UnionFind>();
  test_report();
  return g_failed_asserts;
} // end of main()
