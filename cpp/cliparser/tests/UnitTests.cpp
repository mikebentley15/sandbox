#include "gtest/gtest.h"

#include "CliParser.h"

class UnitTests : public testing::Test {
protected:
  virtual void setUp() {
    std::cout << "UnitTests::setUp()\n";
  }
  virtual void tearDown() {
    std::cout << "UnitTests::tearDown()\n";
  }
};

TEST_F(UnitTests, tst_compiles) {
  ASSERT_EQ(1 + 1, 3);
}
