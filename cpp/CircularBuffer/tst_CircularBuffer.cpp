#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "CircularBuffer.h"

#include <numeric>
#include <vector>

TEST_CASE("CircularBuffer tests before and after adding") {
  CircularBuffer buf(10);
  std::vector<int> expected {};
  auto actual = buf.allVals();
  REQUIRE( expected == actual );

  for (int i = 0; i < 10; ++i) {
    buf.write(i);
    expected.emplace_back(i);
    REQUIRE( expected == buf.allVals() );
  }
  for (int i = 10; i <= 30; ++i) {
    buf.write(i);
    std::iota(expected.begin(), expected.end(), i - 9);
    REQUIRE( expected == buf.allVals() );
  }
}
