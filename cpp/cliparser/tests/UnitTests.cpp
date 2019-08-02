#include "gtest/gtest.h"

#include "CliParser.h"

#include <memory>

namespace {

class UnitTests : public testing::Test {
protected:
  virtual void setUp() {}
  virtual void tearDown() {}
};

class TestParser : public CliParser {
public:
  using CliParser::Option;
  using CliParser::PositionArg;
  using CliParser::OpPtr;
  using CliParser::PosPtr;

  using CliParser::_args;
  using CliParser::_positional;
  using CliParser::_optionmap;
  using CliParser::_parsed;
  using CliParser::_recognized;
  using CliParser::_remaining;
};

void compare(const TestParser::OpPtr &a, const TestParser::OpPtr &b) {
  ASSERT_EQ(a->variants,    b->variants);
  ASSERT_EQ(a->required,    b->required);
  ASSERT_EQ(a->expects_arg, b->expects_arg);
}

void compare(const TestParser::PosPtr &a, const TestParser::PosPtr &b) {
  ASSERT_EQ(a->name, b->name);
  ASSERT_EQ(a->required, b->required);
}

} // end of unnamed namespace

TEST_F(UnitTests, program_name_without_parsing) {
  CliParser parser;
  ASSERT_THROW(parser.program_name(), std::out_of_range);
}

TEST_F(UnitTests, program_name_returns_first_argument) {
  std::vector<std::string> args { "first", "second", "third" };
  CliParser parser;
  parser.parse(args);
  ASSERT_EQ(parser.program_name(), args[0]);
}

TEST_F(UnitTests, add_flag_adds_to_flags) {
  auto help_option = std::make_shared<TestParser::Option>(
                       std::vector<std::string>{"-h"}, false);
  auto verbose_option = std::make_shared<TestParser::Option>(
                          std::vector<std::string>{"-v", "--verbose"}, false);

  TestParser parser;
  parser.add_flag("-h");
  parser.add_flag("-v", "--verbose");

  ASSERT_TRUE(parser._optionmap.find("-h") != parser._optionmap.end());
  ASSERT_TRUE(parser._optionmap.find("-v") != parser._optionmap.end());
  ASSERT_TRUE(
        parser._optionmap.find("--verbose") != parser._optionmap.end());

  compare(parser._optionmap["-h"], help_option);
  compare(parser._optionmap["-v"], verbose_option);
  compare(parser._optionmap["--verbose"], verbose_option);
  ASSERT_EQ(parser._optionmap["-v"], parser._optionmap["--verbose"]);
}

TEST_F(UnitTests, add_flag_adds_to_recognized) {
  TestParser parser;
  parser.add_flag("-h");
  parser.add_flag("-v", "--verbose");
  ASSERT_EQ(parser._recognized.count("-h"), 1);
  ASSERT_EQ(parser._recognized.count("-v"), 1);
  ASSERT_EQ(parser._recognized.count("--verbose"), 1);
  ASSERT_EQ(parser._recognized.count("--not-there"), 0);
}

TEST_F(UnitTests, add_argflag_adds_to_optionmap) {
  auto N_option = std::make_shared<TestParser::Option>(
                    std::vector<std::string>{"-N"}, true);
  auto k_option = std::make_shared<TestParser::Option>(
                    std::vector<std::string>{"-k", "--biggest"}, true);

  TestParser parser;
  parser.add_argflag("-N");
  parser.add_argflag("-k", "--biggest");

  ASSERT_TRUE(parser._optionmap.find("-N") != parser._optionmap.end());
  ASSERT_TRUE(parser._optionmap.find("-k") != parser._optionmap.end());
  ASSERT_TRUE(
        parser._optionmap.find("--biggest") != parser._optionmap.end());

  compare(parser._optionmap["-N"], N_option);
  compare(parser._optionmap["-k"], k_option);
  compare(parser._optionmap["--biggest"], k_option);
  ASSERT_EQ(parser._optionmap["-k"], parser._optionmap["--biggest"]);
}

TEST_F(UnitTests, add_argflag_adds_to_recognized) {
  TestParser parser;
  parser.add_argflag("-N");
  parser.add_argflag("-k", "--biggest");

  ASSERT_EQ(parser._recognized.count("-N"), 1);
  ASSERT_EQ(parser._recognized.count("-k"), 1);
  ASSERT_EQ(parser._recognized.count("--biggest"), 1);
  ASSERT_EQ(parser._recognized.count("--not-there"), 0);
}

TEST_F(UnitTests, add_positional_adds_to_vector) {
  TestParser parser;
  parser.add_positional("hello");
  parser.add_positional("mike");
  auto hello_pos = std::make_shared<TestParser::PositionArg>("hello", false);
  auto mike_pos = std::make_shared<TestParser::PositionArg>("mike", false);
  ASSERT_EQ(parser._positional.size(), 2);
  compare(parser._positional[0], hello_pos);
  compare(parser._positional[1], mike_pos);
}

TEST_F(UnitTests, add_positional_adds_to_recognized) {
  TestParser parser;
  parser.add_positional("hello");
  parser.add_positional("mike");
  ASSERT_EQ(parser._recognized.count("hello"), 1);
  ASSERT_EQ(parser._recognized.count("mike"), 1);
  ASSERT_EQ(parser._recognized.count("hi"), 0);
}

TEST_F(UnitTests, add_argument_duplicate_exception) {
  TestParser parser;
  parser.add_flag("one", "two", "three");
  ASSERT_THROW(parser.add_flag("one"), std::invalid_argument);
  ASSERT_THROW(parser.add_argflag("other", "two"), std::invalid_argument);
  ASSERT_THROW(parser.add_positional("three"), std::invalid_argument);
}

TEST_F(UnitTests, set_required_flag) {
  TestParser parser;
  parser.add_flag("-h", "--help");
  parser.set_required("--help");
  ASSERT_TRUE(parser._optionmap["-h"]->required);
}

TEST_F(UnitTests, set_required_argflag) {
  TestParser parser;
  parser.add_argflag("-N", "-number");
  parser.set_required("-N");
  ASSERT_TRUE(parser._optionmap["-N"]->required);
}

TEST_F(UnitTests, set_required_positional) {
  TestParser parser;
  parser.add_positional("hello");
  parser.add_positional("mike");
  parser.set_required("hello");
  ASSERT_TRUE(parser._positional[0]->required);
  ASSERT_FALSE(parser._positional[1]->required);
}

TEST_F(UnitTests, set_required_nonmatching_throws) {
  TestParser parser;
  parser.add_positional("hello");
  ASSERT_THROW(parser.set_required("does not exist"), std::invalid_argument);
}

TEST_F(UnitTests, args_returns_given_vector_args) {
  std::vector<std::string> args { "a", "b", "c" };
  CliParser parser;
  parser.parse(args);
  ASSERT_EQ(parser.args(), args);
}

TEST_F(UnitTests, args_returns_given_cstring_args) {
  const char* a1 = "mike";
  const char* a2 = "bentley";
  const char* a3 = "is";
  const char* a4 = "great";
  int argc = 4;
  const char * const argv[] = {a1, a2, a3, a4};
  CliParser parser;
  parser.parse(argc, argv);
  std::vector<std::string> expected_args {a1, a2, a3, a4};
  ASSERT_EQ(parser.args(), expected_args);
}

TEST_F(UnitTests, parse_finds_flags) {
  std::vector<std::string> args {
    "progname", "-h", "-N"
  };
  TestParser parser;
  parser.add_flag("-h", "--help");
  parser.add_flag("-k");
  parser.parse(args);
  ASSERT_TRUE(parser._parsed.find("-h") != parser._parsed.end());
  ASSERT_TRUE(parser._parsed.find("--help") != parser._parsed.end());
  ASSERT_TRUE(parser._parsed.find("-k") == parser._parsed.end());
  ASSERT_EQ(*parser._parsed["-h"], "-h");
  ASSERT_EQ(*parser._parsed["--help"], "-h");
}

TEST_F(UnitTests, parse_finds_positional) {
  std::vector<std::string> args {
    "progname", "-h", "-N", "out.txt", "extra"
  };
  TestParser parser;
  parser.add_flag("-h", "--help");
  parser.add_flag("-N");
  parser.add_positional("output");
  parser.parse(args);
  ASSERT_TRUE(parser._parsed.find("output") != parser._parsed.end());
  ASSERT_EQ(*parser._parsed["output"], "out.txt");
}

TEST_F(UnitTests, parse_finds_flags_with_args) {
  std::vector<std::string> args {
    "progname", "--output", "out.txt", "-N", "3"
  };
  TestParser parser;
  parser.add_argflag("-N");
  parser.add_argflag("-o", "--output");
  parser.parse(args);
  ASSERT_TRUE(parser._parsed.find("-N") != parser._parsed.end());
  ASSERT_TRUE(parser._parsed.find("-o") != parser._parsed.end());
  ASSERT_TRUE(parser._parsed.find("--output") != parser._parsed.end());
  ASSERT_EQ(parser["-N"], "3");
  ASSERT_EQ(parser["-o"], "out.txt");
  ASSERT_EQ(parser["--output"], "out.txt");
}

TEST_F(UnitTests, parse_sets_remaining_args_empty) {
  std::vector<std::string> args {
    "progname", "-v", "--output", "out.txt", "-N", "3", "in.txt"
  };
  TestParser parser;
  parser.add_flag("-v");
  parser.add_argflag("-o", "--output");
  parser.add_argflag("-N");
  parser.add_positional("input");
  parser.parse(args);
  std::vector<std::string> expected_remaining {};
  ASSERT_EQ(parser.remaining(), expected_remaining);
}

TEST_F(UnitTests, parse_sets_remaining_args_all) {
  std::vector<std::string> args {
    "progname", "-v", "--output", "out.txt", "-N", "3", "in.txt"
  };
  TestParser parser;
  parser.parse(args);
  std::vector<std::string> expected_remaining {
    "-v", "--output", "out.txt", "-N", "3", "in.txt"
  };
  ASSERT_EQ(parser.remaining(), expected_remaining);
}

TEST_F(UnitTests, parse_sets_remaining_args_part) {
  std::vector<std::string> args {
    "progname", "-v", "--output", "out.txt", "-N", "3", "in.txt"
  };
  TestParser parser;
  parser.add_argflag("-o", "--output");
  parser.add_positional("input");
  parser.parse(args);
  std::vector<std::string> expected_remaining { "-N", "3", "in.txt" };
  ASSERT_TRUE(parser._parsed.find("input") != parser._parsed.end());
  ASSERT_EQ(*parser._parsed["input"], "-v");
  ASSERT_EQ(parser.remaining(), expected_remaining);
}

TEST_F(UnitTests, has_finds_parsed) {
  TestParser parser;
  parser.add_flag("-h", "--help");
  parser.add_flag("-N");
  parser.add_argflag("-k");
  parser.add_positional("positional");
  parser._args = {"progname", "-h"};
  parser._parsed["-h"] = &parser._args[1];
  parser._parsed["--help"] = &parser._args[1];
  ASSERT_TRUE(parser.has("-h"));
  ASSERT_TRUE(parser.has("--help"));
  ASSERT_FALSE(parser.has("-N"));
  ASSERT_FALSE(parser.has("-k"));
  ASSERT_FALSE(parser.has("positional"));
}

TEST_F(UnitTests, has_throws_for_unrecognized_args) {
  TestParser parser;
  parser.add_flag("-h", "--help");
  ASSERT_THROW(parser.has("anything"), std::invalid_argument);
}

TEST_F(UnitTests, array_operator_returns_parsed) {
  std::vector<std::string> args {
    "a.out", "-N", "-3", "--help", "out.txt"
  };
  TestParser parser;
  // parsed manually
  parser._parsed["-h"] = &args[3];
  parser._parsed["-help"] = parser._parsed["-h"];
  parser._parsed["--help"] = parser._parsed["-h"];
  parser._parsed["-N"] = &args[2];
  parser._parsed["positional"] = &args[4];
  // check it reads from _parsed
  ASSERT_EQ(parser["-h"], "--help");
  ASSERT_EQ(parser["-help"], "--help");
  ASSERT_EQ(parser["--help"], "--help");
  ASSERT_EQ(parser["-N"], "-3");
  ASSERT_EQ(parser["positional"], "out.txt");
}

TEST_F(UnitTests, array_operator_throws_not_parsed) {
  TestParser parser;
  parser.add_flag("-f", "--flag");
  ASSERT_THROW(parser["-f"], std::out_of_range);
}

TEST_F(UnitTests, array_operator_throws_not_recognized) {
  TestParser parser;
  parser.add_flag("-f", "--flag");
  ASSERT_THROW(parser["-nothing"], std::invalid_argument);
}

TEST_F(UnitTests, usage_before_parse) {
  TestParser parser;
  EXPECT_EQ(parser.usage(),
            "Usage:\n"
            "  <program-name>\n"
            "\n");
}

TEST_F(UnitTests, usage_empty) {
  TestParser parser;
  parser.parse({"hello"});
  EXPECT_EQ(parser.usage(),
            "Usage:\n"
            "  hello\n"
            "\n");
}

TEST_F(UnitTests, usage_flags) {
  // usage flags are sorted alphabetically (on first variant) after removing dashes
  TestParser parser;
  parser.add_flag("-h", "--help");
  parser.add_flag("--verbose");
  parser.parse({"progname"});
  EXPECT_EQ(parser.usage(),
            "Usage:\n"
            "  progname\n"
            "    [-h]\n"
            "    [--verbose]\n"
            "\n"
            "Optional Flags:\n"
            "  -h, --help\n"
            "  --verbose\n"
            "\n");
}

TEST_F(UnitTests, usage_flags_required) {
  TestParser parser;
  parser.add_flag("-h", "--help");
  parser.add_flag("--verbose");
  parser.set_required("--help");
  parser.set_required("--verbose");
  parser.parse({"progname"});
  EXPECT_EQ(parser.usage(),
            "Usage:\n"
            "  progname\n"
            "    -h\n"
            "    --verbose\n"
            "\n"
            "Required Flags:\n"
            "  -h, --help\n"
            "  --verbose\n"
            "\n");
}

TEST_F(UnitTests, usage_argflags) {
  TestParser parser;
  parser.add_argflag("-N", "--number");
  parser.add_argflag("--out", "-o");
  parser.parse({"progname"});
  EXPECT_EQ(parser.usage(),
            "Usage:\n"
            "  progname\n"
            "    [-N <arg>]\n"
            "    [--out <arg>]\n"
            "\n"
            "Optional Flags:\n"
            "  -N <arg>, --number <arg>\n"
            "  --out <arg>, -o <arg>\n"
            "\n");
}

TEST_F(UnitTests, usage_argflags_required) {
  TestParser parser;
  parser.add_argflag("--out", "-o");
  parser.add_argflag("--number", "-N");
  parser.set_required("--number");
  parser.set_required("--out");
  parser.parse({"progname"});
  EXPECT_EQ(parser.usage(),
            "Usage:\n"
            "  progname\n"
            "    --number <arg>\n"
            "    --out <arg>\n"
            "\n"
            "Required Flags:\n"
            "  --number <arg>, -N <arg>\n"
            "  --out <arg>, -o <arg>\n"
            "\n");
}

TEST_F(UnitTests, usage_positional) {
  TestParser parser;
  parser.add_positional("number");
  parser.add_positional("outfile");
  parser.parse({"progname"});
  EXPECT_EQ(parser.usage(),
            "Usage:\n"
            "  progname\n"
            "    [<number>]\n"
            "    [<outfile>]\n"
            "\n"
            "Optional Positional Arguments:\n"
            "  number\n"
            "  outfile\n"
            "\n");
}

TEST_F(UnitTests, usage_positional_required) {
  TestParser parser;
  parser.add_positional("number");
  parser.add_positional("outfile");
  parser.set_required("number");
  parser.set_required("outfile");
  parser.parse({"progname"});
  EXPECT_EQ(parser.usage(),
            "Usage:\n"
            "  progname\n"
            "    <number>\n"
            "    <outfile>\n"
            "\n"
            "Required Positional Arguments:\n"
            "  number\n"
            "  outfile\n"
            "\n");
}

TEST_F(UnitTests, usage_all) {
  TestParser parser;
  parser.add_flag("-h", "-help", "--help");
  parser.add_flag("-v", "--verbose");
  parser.add_argflag("-k", "--biggest");
  parser.add_argflag("-N");
  parser.add_argflag("-o", "--output");
  parser.add_positional("input");
  parser.set_required("--output");
  parser.set_required("input");
  parser._args = {"progname"};
  EXPECT_EQ(parser.usage(),
            "Usage:\n"
            "  progname\n"
            "    [-h]\n"
            "    [-v]\n"
            "    [-k <val>]\n"
            "    [-N <val>]\n"
            "    -o <val>\n"
            "    <input>\n"
            "\n"
            "Required Positional Arguments:\n"
            "  input\n"
            "\n"
            "Required Flags:\n"
            "  -o <val>, --output <val>\n"
            "\n"
            "Optional Flags:\n"
            "  -h, --help\n"
            "  -v, --verbose\n"
            "  -k <val>, --biggest <val>\n"
            "  -N <val>\n"
            "\n");
}
