#include "gtest/gtest.h"

#include "CliParser.h"

namespace {

class FunctionalTests : public testing::Test {
protected:
  virtual void setUp() {}
  virtual void tearDown() {}
};

}

TEST_F(FunctionalTests, EmptyArgsMistake) {
  // the user, trying to use the constructor without any
  // command-line arguments will get an exception with a
  // helpful message
  ASSERT_ANY_THROW(CliParser(std::vector<std::string>{}));
}

TEST_F(FunctionalTests, NoArguments) {
  // Fred tries to parse arguments, but only the program name
  // is given
  std::vector<std::string> args { "program-name" };
  CliParser parser(args);

  // Fred sees that the program name is given, that the remaining
  // arguments are empty, that he can retrieve the arguments he
  // gave, and that any flag he checks returns false.
  // Fred is happy.
  EXPECT_EQ(parser.program_name(), "program-name");
  EXPECT_EQ(parser.remaining_args(), decltype(args){});
  EXPECT_EQ(parser.args(), args);
  EXPECT_FALSE(parser.has_argument("-h"));
  EXPECT_FALSE(parser.has_argument("--verbose"));
  EXPECT_FALSE(parser.has_argument("--freds-birthday"));
}

TEST_F(FunctionalTests, OptionArguments) {
  // Bob is parsing arguments and there are a few options he wants to find
  std::vector<std::string> args {
    "prog-name", "-h", "--verbose", "--not-quiet"
  };
  CliParser parser(args);
  EXPECT_TRUE(parser.has_argument("-h", "--help"));
  EXPECT_TRUE(parser.has_argument("-v", "--verbose"));
  EXPECT_TRUE(parser.has_argument("--not-quiet"));
  EXPECT_FALSE(parser.has_argument("-q", "--quiet"));
  EXPECT_FALSE(parser.has_argument("-bbday", "--bobs-birthday"));
  EXPECT_FALSE(parser.has_argument("--get-fridge"));
  // all arguments have been queried, so no remaining arguments
  EXPECT_EQ(parser.remaining_args(), decltype(args){});
}

TEST_F(FunctionalTests, RemainingArgumentsNoOptions) {
  std::vector<std::string> args {
    "program-name", "-h", "--verbose", "--not-quiet",
  };
  std::vector<std::string> expected_remaining(args.begin()+1, args.end());
  CliParser parser(args);
  EXPECT_EQ(expected_remaining, parser.remaining_args());
}

TEST_F(FunctionalTests, RemainingArgumentsSomeOptions) {
  std::vector<std::string> args {
    "program-name", "-h", "--verbose", "--not-quiet",
  };
  std::vector<std::string> expected_remaining {"-h", "--not-quiet"};
  CliParser parser(args);
  EXPECT_TRUE(parser.has_argument("-v", "--verbose"));
  EXPECT_EQ(expected_remaining, parser.remaining_args());
}

TEST_F(FunctionalTests, ManyOptionTypesGiven) {
  std::vector<std::string> args {
    "program-name", "-h", "--help", "--verbose"
  };
  std::vector<std::string> expected_remaining {"--verbose"};
  CliParser parser(args);
  EXPECT_TRUE(parser.has_argument("-h", "--help"));
  EXPECT_EQ(expected_remaining, parser.remaining_args());
}
