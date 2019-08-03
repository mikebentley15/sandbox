#include "CliParser.h"
#include "test_helpers.h"

#include <gtest/gtest.h>

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

struct Point { int x, y; };
std::istream &operator>>(std::istream &in, Point &p) {
  return in >> p.x >> p.y;
}
bool operator==(const Point &a, const Point &b) {
  return a.x == b.x && a.y == b.y;
}
std::ostream& operator<<(std::ostream& out, const Point &p) {
  return out << "Point(" << p.x << ", " << p.y << ")";
}

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

TEST_F(UnitTests, constructor_creates_help_flag) {
  TestParser parser;
  ASSERT_EQ(parser._optionmap.size(), 2);
  ASSERT_TRUE(parser._optionmap.find("-h") != parser._optionmap.end());
  ASSERT_TRUE(parser._optionmap.find("--help") != parser._optionmap.end());
  ASSERT_EQ(parser._optionmap["-h"], parser._optionmap["--help"]);
  ASSERT_EQ(parser._optionmap["-h"]->variants.size(), 2);
  ASSERT_EQ(parser._optionmap["-h"]->variants[0], "-h");
  ASSERT_EQ(parser._optionmap["-h"]->variants[1], "--help");
  ASSERT_FALSE(parser._optionmap["-h"]->expects_arg);
  ASSERT_FALSE(parser._optionmap["-h"]->required);
  ASSERT_EQ(parser._recognized.count("-h"), 1);
  ASSERT_EQ(parser._recognized.count("--help"), 1);
}

TEST_F(UnitTests, add_flag_adds_to_flags) {
  auto m_option = std::make_shared<TestParser::Option>(
                    std::vector<std::string>{"-m"}, false);
  auto verbose_option = std::make_shared<TestParser::Option>(
                          std::vector<std::string>{"-v", "--verbose"}, false);

  TestParser parser;
  parser.add_flag("-m");
  parser.add_flag("-v", "--verbose");

  ASSERT_TRUE(parser._optionmap.find("-m") != parser._optionmap.end());
  ASSERT_TRUE(parser._optionmap.find("-v") != parser._optionmap.end());
  ASSERT_TRUE(
        parser._optionmap.find("--verbose") != parser._optionmap.end());

  compare(parser._optionmap["-m"], m_option);
  compare(parser._optionmap["-v"], verbose_option);
  compare(parser._optionmap["--verbose"], verbose_option);
  ASSERT_EQ(parser._optionmap["-v"], parser._optionmap["--verbose"]);
}

TEST_F(UnitTests, add_flag_adds_to_recognized) {
  TestParser parser;
  parser.add_flag("-m");
  parser.add_flag("-v", "--verbose");
  ASSERT_EQ(parser._recognized.count("-m"), 1);
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
  ASSERT_THROW(parser.add_flag("-h"), std::invalid_argument);
  ASSERT_THROW(parser.add_flag("--help"), std::invalid_argument);
}

TEST_F(UnitTests, set_required_flag) {
  TestParser parser;
  parser.add_flag("-m", "--move");
  parser.set_required("--move");
  ASSERT_TRUE(parser._optionmap["-m"]->required);
  ASSERT_TRUE(parser._optionmap["--move"]->required);
}

TEST_F(UnitTests, set_required_argflag) {
  TestParser parser;
  parser.add_argflag("-N", "-number");
  parser.set_required("-N");
  ASSERT_TRUE(parser._optionmap["-N"]->required);
  ASSERT_TRUE(parser._optionmap["-number"]->required);
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

TEST_F(UnitTests, parse_help_exits) {
  CliParser parser;
  std::string usage {
    "Usage:\n"
    "  ./a.out --help\n"
    "  ./a.out\n"
    "\n"
    "Optional Flags:\n"
    "  -h, --help    Print this help and exit\n"
    "\n"
  };
  assert_help_exit(parser, {"./a.out", "-h"}, usage);
  assert_help_exit(parser, {"./a.out", "--help"}, usage);
}

TEST_F(UnitTests, parse_help_exits_with_correct_message) {
  CliParser parser;
  parser.add_argflag("-x");
  parser.add_positional("infile");
  parser.set_required("infile");
  std::string usage {
    "Usage:\n"
    "  progname --help\n"
    "  progname\n"
    "    [-x <val>]\n"
    "    <infile>\n"
    "\n"
    "Required Positional Arguments:\n"
    "  infile\n"
    "\n"
    "Optional Flags:\n"
    "  -h, --help    Print this help and exit\n"
    "  -x <val>\n"
    "\n"
  };
  ASSERT_EQ(parser.usage("progname"), usage);
  ASSERT_THROW(parser.parse_with_exceptions({"progname", "-h"}),
               CliParser::HelpRequest);
  ASSERT_THROW(parser.parse_with_exceptions({"progname", "--help"}),
               CliParser::HelpRequest);
  assert_help_exit(parser, {"progname", "-h"}, usage);
  assert_help_exit(parser, {"progname", "--help"}, usage);
}

TEST_F(UnitTests, parse_finds_flags) {
  TestParser parser;
  parser.add_flag("-m", "--move");
  parser.add_flag("-k");
  parser.parse({"progname", "-m", "-N"});
  ASSERT_TRUE(parser._parsed.find("-m") != parser._parsed.end());
  ASSERT_TRUE(parser._parsed.find("--move") != parser._parsed.end());
  ASSERT_TRUE(parser._parsed.find("-k") == parser._parsed.end());
  ASSERT_EQ(*parser._parsed["-m"], "-m");
  ASSERT_EQ(*parser._parsed["--move"], "-m");
}

TEST_F(UnitTests, parse_finds_positional) {
  TestParser parser;
  parser.add_flag("-m", "--move");
  parser.add_flag("-N");
  parser.add_positional("output");
  parser.parse({"progname", "-m", "-N", "out.txt", "extra"});
  ASSERT_TRUE(parser._parsed.find("output") != parser._parsed.end());
  ASSERT_EQ(*parser._parsed["output"], "out.txt");
}

TEST_F(UnitTests, parse_finds_flags_with_args) {
  TestParser parser;
  parser.add_argflag("-N");
  parser.add_argflag("-o", "--output");
  parser.parse({"progname", "--output", "out.txt", "-N", "3"});
  ASSERT_TRUE(parser._parsed.find("-N") != parser._parsed.end());
  ASSERT_TRUE(parser._parsed.find("-o") != parser._parsed.end());
  ASSERT_TRUE(parser._parsed.find("--output") != parser._parsed.end());
  ASSERT_EQ(parser["-N"], "3");
  ASSERT_EQ(parser["-o"], "out.txt");
  ASSERT_EQ(parser["--output"], "out.txt");
}

TEST_F(UnitTests, parse_finds_all_argflags) {
  // A bug found by a functional test, minimized here
  CliParser parser;
  parser.add_argflag("-N");
  parser.parse({"progname", "-N", "3", "-o", "out.txt", "in.txt", "extra", "args"});
  ASSERT_TRUE(parser.has("-N"));
}

TEST_F(UnitTests, parse_sets_remaining_args_empty) {
  TestParser parser;
  parser.add_flag("-v");
  parser.add_argflag("-o", "--output");
  parser.add_argflag("-N");
  parser.add_positional("input");
  parser.parse({"progname", "-v", "--output", "out.txt", "-N", "3", "in.txt"});
  std::vector<std::string> expected_remaining {};
  ASSERT_EQ(parser.remaining(), expected_remaining);
}

TEST_F(UnitTests, parse_sets_remaining_args_all) {
  TestParser parser;
  parser.parse({"progname", "-v", "--output", "out.txt", "-N", "3", "in.txt"});
  std::vector<std::string> expected_remaining {
    "-v", "--output", "out.txt", "-N", "3", "in.txt"
  };
  ASSERT_EQ(parser.remaining(), expected_remaining);
}

TEST_F(UnitTests, parse_sets_remaining_args_part) {
  TestParser parser;
  parser.add_argflag("-o", "--output");
  parser.add_positional("input");
  parser.parse({"progname", "-v", "--output", "out.txt", "-N", "3", "in.txt"});
  std::vector<std::string> expected_remaining { "-N", "3", "in.txt" };
  ASSERT_TRUE(parser._parsed.find("input") != parser._parsed.end());
  ASSERT_EQ(*parser._parsed["input"], "-v");
  ASSERT_EQ(parser.remaining(), expected_remaining);
}

TEST_F(UnitTests, parse_missing_required_flag) {
  TestParser parser;
  parser.add_flag("-x");
  parser.set_required("-x");
  ASSERT_THROW(parser.parse_with_exceptions({"progname"}), std::invalid_argument);
  assert_error_exit(parser, {"progname"}, "ParseError");
}

TEST_F(UnitTests, parse_missing_required_argflag) {
  TestParser parser;
  parser.add_argflag("-x");
  parser.set_required("-x");
  ASSERT_THROW(parser.parse_with_exceptions({"progname"}), std::invalid_argument);
  assert_error_exit(parser, {"progname"}, "ParseError");
}

TEST_F(UnitTests, parse_missing_required_positional_arg) {
  TestParser parser;
  parser.add_positional("input");
  parser.set_required("input");
  ASSERT_THROW(parser.parse_with_exceptions({"progname"}), std::invalid_argument);
  assert_error_exit(parser, {"progname"}, "ParseError");
}

TEST_F(UnitTests, parse_missing_argflag_value) {
  TestParser parser;
  parser.add_argflag("-x");
  ASSERT_THROW(parser.parse_with_exceptions({"progname", "-x"}),
               std::invalid_argument);
  assert_error_exit(parser, {"progname", "-x"}, "ParseError");
}

TEST_F(UnitTests, parse_missing_argflag_required_value) {
  TestParser parser;
  parser.add_argflag("-x");
  parser.set_required("-x");
  ASSERT_THROW(parser.parse_with_exceptions({"progname", "-x"}),
               std::invalid_argument);
  assert_error_exit(parser, {"progname", "-x"}, "ParseError");
}

TEST_F(UnitTests, has_finds_parsed) {
  TestParser parser;
  parser.add_flag("-m", "--move");
  parser.add_flag("-N");
  parser.add_argflag("-k");
  parser.add_positional("positional");
  parser._args = {"progname", "-h"};
  parser._parsed["-m"] = &parser._args[1];
  parser._parsed["--move"] = &parser._args[1];
  ASSERT_TRUE(parser.has("-m"));
  ASSERT_TRUE(parser.has("--move"));
  ASSERT_FALSE(parser.has("-N"));
  ASSERT_FALSE(parser.has("-k"));
  ASSERT_FALSE(parser.has("positional"));
}

TEST_F(UnitTests, has_throws_for_unrecognized_args) {
  TestParser parser;
  parser.add_flag("-m", "--move");
  ASSERT_THROW(parser.has("anything"), std::invalid_argument);
}

TEST_F(UnitTests, array_operator_returns_parsed) {
  std::vector<std::string> args {
    "a.out", "-N", "-3", "--move", "out.txt"
  };
  TestParser parser;
  // parsed manually
  parser._parsed["-m"] = &args[3];
  parser._parsed["-move"] = parser._parsed["-m"];
  parser._parsed["--move"] = parser._parsed["-m"];
  parser._parsed["-N"] = &args[2];
  parser._parsed["positional"] = &args[4];
  // check it reads from _parsed
  ASSERT_EQ(parser["-m"], "--move");
  ASSERT_EQ(parser["-move"], "--move");
  ASSERT_EQ(parser["--move"], "--move");
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

TEST_F(UnitTests, get_non_argflag) {
  TestParser parser;
  parser.add_flag("-g");
  parser.parse({"progname", "-g"});
  ASSERT_EQ(parser["-g"], "-g");
  ASSERT_THROW(parser.get<bool              >("-g"), std::invalid_argument);
  ASSERT_THROW(parser.get<int               >("-g"), std::invalid_argument);
  ASSERT_THROW(parser.get<long              >("-g"), std::invalid_argument);
  ASSERT_THROW(parser.get<long long         >("-g"), std::invalid_argument);
  ASSERT_THROW(parser.get<unsigned int      >("-g"), std::invalid_argument);
  ASSERT_THROW(parser.get<unsigned long     >("-g"), std::invalid_argument);
  ASSERT_THROW(parser.get<unsigned long long>("-g"), std::invalid_argument);
  ASSERT_THROW(parser.get<float             >("-g"), std::invalid_argument);
  ASSERT_THROW(parser.get<double            >("-g"), std::invalid_argument);
  ASSERT_THROW(parser.get<long double       >("-g"), std::invalid_argument);
  ASSERT_EQ(parser.get<std::string>("-g"), "-g");
}

TEST_F(UnitTests, get_different_types) {
  TestParser parser;
  parser.add_positional("boolean");
  parser.add_positional("number");
  parser.parse({"progname", "true", "3"});
  ASSERT_EQ(parser["boolean"], "true");
  ASSERT_EQ(parser["number"], "3");
  ASSERT_EQ(parser.get<bool              >("boolean"), true);
  ASSERT_EQ(parser.get<std::string       >("boolean"), "true");
  ASSERT_EQ(parser.get<int               >("number"), 3);
  ASSERT_EQ(parser.get<long              >("number"), 3L);
  ASSERT_EQ(parser.get<long long         >("number"), 3LL);
  ASSERT_EQ(parser.get<unsigned int      >("number"), 3U);
  ASSERT_EQ(parser.get<unsigned long     >("number"), 3UL);
  ASSERT_EQ(parser.get<unsigned long long>("number"), 3ULL);
  ASSERT_EQ(parser.get<float             >("number"), 3.0f);
  ASSERT_EQ(parser.get<double            >("number"), 3.0);
  ASSERT_EQ(parser.get<long double       >("number"), 3.0L);
  ASSERT_EQ(parser.get<std::string       >("number"), "3");
}

TEST_F(UnitTests, get_missing_arg) {
  TestParser parser;
  parser.add_positional("boolean");
  parser.add_positional("number");
  parser.parse({"progname"});
  ASSERT_THROW(parser["boolean"],                         std::out_of_range);
  ASSERT_THROW(parser["number"],                          std::out_of_range);
  ASSERT_THROW(parser.get<bool              >("boolean"), std::out_of_range);
  ASSERT_THROW(parser.get<std::string       >("boolean"), std::out_of_range);
  ASSERT_THROW(parser.get<int               >("number"),  std::out_of_range);
  ASSERT_THROW(parser.get<long              >("number"),  std::out_of_range);
  ASSERT_THROW(parser.get<long long         >("number"),  std::out_of_range);
  ASSERT_THROW(parser.get<unsigned int      >("number"),  std::out_of_range);
  ASSERT_THROW(parser.get<unsigned long     >("number"),  std::out_of_range);
  ASSERT_THROW(parser.get<unsigned long long>("number"),  std::out_of_range);
  ASSERT_THROW(parser.get<float             >("number"),  std::out_of_range);
  ASSERT_THROW(parser.get<double            >("number"),  std::out_of_range);
  ASSERT_THROW(parser.get<long double       >("number"),  std::out_of_range);
  ASSERT_THROW(parser.get<std::string       >("number"),  std::out_of_range);
}

TEST_F(UnitTests, get_invalid_cast) {
  TestParser parser;
  parser.add_positional("boolean");
  parser.add_positional("number");
  parser.parse({"progname", "true", "5"});
  ASSERT_THROW(parser.get<int               >("boolean"), std::invalid_argument);
  ASSERT_THROW(parser.get<long              >("boolean"), std::invalid_argument);
  ASSERT_THROW(parser.get<long long         >("boolean"), std::invalid_argument);
  ASSERT_THROW(parser.get<unsigned int      >("boolean"), std::invalid_argument);
  ASSERT_THROW(parser.get<unsigned long     >("boolean"), std::invalid_argument);
  ASSERT_THROW(parser.get<unsigned long long>("boolean"), std::invalid_argument);
  ASSERT_THROW(parser.get<float             >("boolean"), std::invalid_argument);
  ASSERT_THROW(parser.get<double            >("boolean"), std::invalid_argument);
  ASSERT_THROW(parser.get<long double       >("boolean"), std::invalid_argument);
  ASSERT_THROW(parser.get<bool              >("number"),  std::invalid_argument);
}

TEST_F(UnitTests, get_with_default) {
  TestParser parser;
  parser.add_positional("boolean");
  parser.add_positional("number");
  parser.parse({"progname"});
  ASSERT_THROW(parser["boolean"], std::out_of_range);
  ASSERT_THROW(parser["number"],  std::out_of_range);

  // it's nice to have template deduction
  ASSERT_EQ(parser.get("boolean", true               ), true  );
  ASSERT_EQ(parser.get("boolean", std::string("true")), "true");
  ASSERT_EQ(parser.get("number",  3                  ), 3     );
  ASSERT_EQ(parser.get("number",  3L                 ), 3L    );
  ASSERT_EQ(parser.get("number",  3LL                ), 3LL   );
  ASSERT_EQ(parser.get("number",  3U                 ), 3U    );
  ASSERT_EQ(parser.get("number",  3UL                ), 3UL   );
  ASSERT_EQ(parser.get("number",  3ULL               ), 3ULL  );
  ASSERT_EQ(parser.get("number",  3.0f               ), 3.0f  );
  ASSERT_EQ(parser.get("number",  3.0                ), 3.0   );
  ASSERT_EQ(parser.get("number",  3.0L               ), 3.0L  );
  ASSERT_EQ(parser.get("number",  std::string("3")   ), "3"   );

  parser.parse({"progname", "false", "5"});
  ASSERT_EQ(parser["boolean"], "false");
  ASSERT_EQ(parser["number"],  "5"    );

  ASSERT_EQ(parser.get("boolean", true               ), false  );
  ASSERT_EQ(parser.get("boolean", std::string("true")), "false");
  ASSERT_EQ(parser.get("number",  3                  ), 5     );
  ASSERT_EQ(parser.get("number",  3L                 ), 5L    );
  ASSERT_EQ(parser.get("number",  3LL                ), 5LL   );
  ASSERT_EQ(parser.get("number",  3U                 ), 5U    );
  ASSERT_EQ(parser.get("number",  3UL                ), 5UL   );
  ASSERT_EQ(parser.get("number",  3ULL               ), 5ULL  );
  ASSERT_EQ(parser.get("number",  3.0f               ), 5.0f  );
  ASSERT_EQ(parser.get("number",  3.0                ), 5.0   );
  ASSERT_EQ(parser.get("number",  3.0L               ), 5.0L  );
  ASSERT_EQ(parser.get("number",  std::string("3")   ), "5"   );
}

TEST_F(UnitTests, get_with_default_invalid_cast) {
  TestParser parser;
  parser.add_positional("boolean");
  parser.add_positional("number");
  parser.parse({"progname", "true", "5"});
  ASSERT_THROW(parser.get("boolean", 3   ), std::invalid_argument);
  ASSERT_THROW(parser.get("boolean", 3L  ), std::invalid_argument);
  ASSERT_THROW(parser.get("boolean", 3LL ), std::invalid_argument);
  ASSERT_THROW(parser.get("boolean", 3U  ), std::invalid_argument);
  ASSERT_THROW(parser.get("boolean", 3UL ), std::invalid_argument);
  ASSERT_THROW(parser.get("boolean", 3ULL), std::invalid_argument);
  ASSERT_THROW(parser.get("boolean", 3.0f), std::invalid_argument);
  ASSERT_THROW(parser.get("boolean", 3.0 ), std::invalid_argument);
  ASSERT_THROW(parser.get("boolean", 3.0L), std::invalid_argument);
  ASSERT_THROW(parser.get("number", true),  std::invalid_argument);
}

TEST_F(UnitTests, get_custom_type) {
  TestParser parser;
  parser.add_positional("point");
  parser.add_argflag("--start");
  parser.parse({"a.out", "3 5"});
  Point expected_point {3, 5};
  Point other_point {1, 2};
  ASSERT_EQ(parser["point"], "3 5");
  ASSERT_EQ(parser.get<Point>("point"), expected_point);
  ASSERT_EQ(parser.get("point", other_point), expected_point);
  ASSERT_THROW(parser["--start"], std::out_of_range);
  ASSERT_THROW(parser.get<Point>("--start"), std::out_of_range);
  ASSERT_EQ(parser.get("--start", other_point), other_point);
}

TEST_F(UnitTests, usage_before_parse) {
  TestParser parser;
  ASSERT_EQ(parser.usage(),
            "Usage:\n"
            "  <program-name> --help\n"
            "  <program-name>\n"
            "\n"
            "Optional Flags:\n"
            "  -h, --help    Print this help and exit\n"
            "\n");
}

TEST_F(UnitTests, usage_empty) {
  TestParser parser;
  parser.parse({"hello"});
  ASSERT_EQ(parser.usage(),
            "Usage:\n"
            "  hello --help\n"
            "  hello\n"
            "\n"
            "Optional Flags:\n"
            "  -h, --help    Print this help and exit\n"
            "\n");
}

TEST_F(UnitTests, usage_empty_with_given_name) {
  TestParser parser;
  ASSERT_EQ(parser.usage("a.out"),
            "Usage:\n"
            "  a.out --help\n"
            "  a.out\n"
            "\n"
            "Optional Flags:\n"
            "  -h, --help    Print this help and exit\n"
            "\n");
}

TEST_F(UnitTests, usage_flags) {
  // usage flags are sorted alphabetically (on first variant) after removing dashes
  TestParser parser;
  parser.add_flag("-a", "--about");
  parser.add_flag("--verbose");
  parser.parse({"progname"});
  ASSERT_EQ(parser.usage(),
            "Usage:\n"
            "  progname --help\n"
            "  progname\n"
            "    [-a]\n"
            "    [--verbose]\n"
            "\n"
            "Optional Flags:\n"
            "  -a, --about\n"
            "  -h, --help    Print this help and exit\n"
            "  --verbose\n"
            "\n");
}

TEST_F(UnitTests, usage_flags_required) {
  TestParser parser;
  parser.add_flag("-a", "--about");
  parser.add_flag("--verbose");
  parser.set_required("--about");
  parser.set_required("--verbose");
  ASSERT_EQ(parser.usage("progname"),
            "Usage:\n"
            "  progname --help\n"
            "  progname\n"
            "    -a\n"
            "    --verbose\n"
            "\n"
            "Required Flags:\n"
            "  -a, --about\n"
            "  --verbose\n"
            "\n"
            "Optional Flags:\n"
            "  -h, --help    Print this help and exit\n"
            "\n");
}

TEST_F(UnitTests, usage_argflags) {
  TestParser parser;
  parser.add_argflag("-N", "--number");
  parser.add_argflag("--out", "-o");
  ASSERT_EQ(parser.usage("progname"),
            "Usage:\n"
            "  progname --help\n"
            "  progname\n"
            "    [-N <val>]\n"
            "    [--out <val>]\n"
            "\n"
            "Optional Flags:\n"
            "  -N <val>, --number <val>\n"
            "  -h, --help    Print this help and exit\n"
            "  --out <val>, -o <val>\n"
            "\n");
}

TEST_F(UnitTests, usage_argflags_required) {
  TestParser parser;
  parser.add_argflag("--out", "-o");
  parser.add_argflag("--number", "-N");
  parser.set_required("--number");
  parser.set_required("--out");
  ASSERT_EQ(parser.usage("progname"),
            "Usage:\n"
            "  progname --help\n"
            "  progname\n"
            "    --number <val>\n"
            "    --out <val>\n"
            "\n"
            "Required Flags:\n"
            "  --number <val>, -N <val>\n"
            "  --out <val>, -o <val>\n"
            "\n"
            "Optional Flags:\n"
            "  -h, --help    Print this help and exit\n"
            "\n");
}

TEST_F(UnitTests, usage_positional) {
  TestParser parser;
  parser.add_positional("number");
  parser.add_positional("outfile");
  ASSERT_EQ(parser.usage("progname"),
            "Usage:\n"
            "  progname --help\n"
            "  progname\n"
            "    [<number>]\n"
            "    [<outfile>]\n"
            "\n"
            "Optional Positional Arguments:\n"
            "  number\n"
            "  outfile\n"
            "\n"
            "Optional Flags:\n"
            "  -h, --help    Print this help and exit\n"
            "\n");
}

TEST_F(UnitTests, usage_positional_required) {
  TestParser parser;
  parser.add_positional("number");
  parser.add_positional("outfile");
  parser.set_required("number");
  parser.set_required("outfile");
  ASSERT_EQ(parser.usage("progname"),
            "Usage:\n"
            "  progname --help\n"
            "  progname\n"
            "    <number>\n"
            "    <outfile>\n"
            "\n"
            "Required Positional Arguments:\n"
            "  number\n"
            "  outfile\n"
            "\n"
            "Optional Flags:\n"
            "  -h, --help    Print this help and exit\n"
            "\n");
}

TEST_F(UnitTests, usage_all) {
  TestParser parser;
  parser.add_flag("-m", "-move", "--move");
  parser.add_flag("-v", "--verbose");
  parser.add_argflag("-k", "--biggest");
  parser.add_argflag("-N");
  parser.add_argflag("-o", "--output");
  parser.add_positional("input");
  parser.add_positional("output");
  parser.set_required("--output");
  parser.set_required("input");
  parser._args = {"progname"};
  ASSERT_EQ(parser.usage(),
            "Usage:\n"
            "  progname --help\n"
            "  progname\n"
            "    [-N <val>]\n"
            "    [-k <val>]\n"
            "    [-m]\n"
            "    [-v]\n"
            "    -o <val>\n"
            "    <input>\n"
            "    [<output>]\n"
            "\n"
            "Required Positional Arguments:\n"
            "  input\n"
            "\n"
            "Optional Positional Arguments:\n"
            "  output\n"
            "\n"
            "Required Flags:\n"
            "  -o <val>, --output <val>\n"
            "\n"
            "Optional Flags:\n"
            "  -N <val>\n"
            "  -h, --help    Print this help and exit\n"
            "  -k <val>, --biggest <val>\n"
            "  -m, -move, --move\n"
            "  -v, --verbose\n"
            "\n");
}
