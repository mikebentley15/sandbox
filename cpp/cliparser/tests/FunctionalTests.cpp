#include "gtest/gtest.h"

#include "CliParser.h"

class FunctionalTests : public testing::Test {
protected:
  virtual void setUp() {
    std::cout << "FunctionalTests::setUp()\n";
  }
  virtual void tearDown() {
    std::cout << "FunctionalTests::tearDown()\n";
  }
};

TEST_F(FunctionalTests, tst_compiles) {
  EXPECT_EQ(2 + 2, 5);
}
