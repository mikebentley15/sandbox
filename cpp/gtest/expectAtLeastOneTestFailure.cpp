#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include <string>

/// Given a callable with google test assertion macros, this expects at least
/// one test failure to happen when calling callable.
///
/// @param msg A message to print when there are no test failures triggered by
/// calling callable.
void expectAtLeastOneTestFailure(auto callable, const std::string &msg = {}) {
  // capture all expect failures in test result array and run it in its own
  // scope.
  ::testing::TestPartResultArray gtest_failures;
  ::testing::ScopedFakeTestPartResultReporter gtest_reporter(
      ::testing::ScopedFakeTestPartResultReporter::
          INTERCEPT_ONLY_CURRENT_THREAD,
      &gtest_failures);
  callable();
  if (msg.empty()) {
    EXPECT_GT(gtest_failures.size(), 0);
  } else {
    EXPECT_GT(gtest_failures.size(), 0) << msg;
  }
}
