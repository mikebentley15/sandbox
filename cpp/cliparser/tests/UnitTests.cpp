#include "gtest/gtest.h"

#include "CliParser.h"

namespace {

class UnitTests : public testing::Test {
protected:
  virtual void setUp() {}
  virtual void tearDown() {}
};

}

TEST_F(UnitTests, construct_with_no_arguments_throws) {
  ASSERT_THROW(CliParser(std::vector<std::string>{}), std::invalid_argument);
}

TEST_F(UnitTests, program_name_returns_first_argument) {
  std::vector<std::string> args { "first", "second", "third" };
  CliParser parser(args);
  ASSERT_EQ(parser.program_name(), args[0]);
}

TEST_F(UnitTests, args_returns_given_vector_args) {
  std::vector<std::string> args { "a", "b", "c" };
  CliParser parser(args);
  ASSERT_EQ(parser.args(), args);
}

TEST_F(UnitTests, args_returns_given_cstring_args) {
  const char* a1 = "mike";
  const char* a2 = "bentley";
  const char* a3 = "is";
  const char* a4 = "great";
  int argc = 4;
  const char * const argv[] = {a1, a2, a3, a4};
  CliParser parser(argc, argv);
  std::vector<std::string> expected_args {a1, a2, a3, a4};
  ASSERT_EQ(parser.args(), expected_args);
}
