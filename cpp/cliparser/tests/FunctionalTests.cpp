#include "CliParser.h"
#include "test_helpers.h"

#include <gtest/gtest.h>

#include <stdexcept>

namespace {

class FunctionalTests : public testing::Test {
protected:
  virtual void setUp() {}
  virtual void tearDown() {}
};

}

TEST_F(FunctionalTests, CommandLineExample) {
  std::vector<std::string> args {
    "progname", "-N", "3", "-o", "out.txt", "in.txt", "extra", "args"
  };

  CliParser parser;
  parser.add_flag("-v", "--verbose");
  parser.add_argflag("-k", "-big", "--biggest");
  parser.add_argflag("-N");
  parser.add_argflag("-o", "--output");
  parser.add_positional("input");
  parser.set_required("--output");
  parser.set_required("input");

  // TODO: add description functions
  //parser.prog_description("program-description");
  //parser.set_description("-N", "N-description");
  //parser.set_description("--output", "my output description");

  parser.parse(args);

  // see that we can get the arguments back unchanged
  ASSERT_EQ(parser.args(), args);

  // see that we can check for which arguments were present
  ASSERT_FALSE(parser.has("-k"));
  ASSERT_FALSE(parser.has("-big"));
  ASSERT_FALSE(parser.has("--biggest"));
  ASSERT_FALSE(parser.has("-v"));
  ASSERT_FALSE(parser.has("--verbose"));
  ASSERT_TRUE(parser.has("-N"));
  ASSERT_TRUE(parser.has("-o"));
  ASSERT_TRUE(parser.has("--output"));
  ASSERT_TRUE(parser.has("input"));

  // see that checking a flag we didn't add throws an exception
  ASSERT_THROW(parser.has("--non-existent"), std::invalid_argument);

  // get the values of the arguments
  ASSERT_THROW(parser["-v"], std::out_of_range);
  ASSERT_THROW(parser["--verbose"], std::out_of_range);
  ASSERT_THROW(parser["-k"], std::out_of_range);
  ASSERT_THROW(parser["-big"], std::out_of_range);
  ASSERT_THROW(parser["--biggest"], std::out_of_range);
  ASSERT_EQ(parser["-N"], "3");
  ASSERT_EQ(parser["-o"], "out.txt");
  ASSERT_EQ(parser["--output"], "out.txt");

  // see that getting the value of an undefined flag throws an exception
  ASSERT_THROW(parser["--non-existent"], std::invalid_argument);

  // convert some values
  ASSERT_EQ(parser.get<int>("-N"), 3);
  ASSERT_THROW(parser.get<int>("-k"), std::out_of_range);
  ASSERT_EQ(parser.get<int>("-N", 10), 3);
  ASSERT_EQ(parser.get<int>("-k", 5), 5);
  ASSERT_EQ(parser.get<std::string>("--output"), "out.txt");
  ASSERT_EQ(parser.get<std::string>("input"), "in.txt");

  // see which arguments were not parsed
  std::vector<std::string> expected_remaining {"extra", "args"};
  ASSERT_EQ(parser.program_name(), "progname");
  ASSERT_EQ(parser.remaining(), expected_remaining);

  ASSERT_EQ(parser.usage(),
            "Usage:\n"
            "  progname --help\n"
            "  progname\n"
            "    [-N <val>]\n"
            "    [-k <val>]\n"
            "    [-v]\n"
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
            "  -N <val>\n"
            "  -h, --help    Print this help and exit\n"
            "  -k <val>, -big <val>, --biggest <val>\n"
            "  -v, --verbose\n"
            "\n");
}

TEST_F(FunctionalTests, InvalidArguments) {
  // Bob wants to make sure that invalid arguments are handled properly
  // when users use his command-line interface wrong
  CliParser parser;

  // required
  parser.add_flag("-?");
  parser.set_required("-?");
  parser.add_argflag("-c", "--config");
  parser.set_required("--config");
  parser.add_positional("input");
  parser.set_required("input");

  // optional
  parser.add_argflag("-k", "--biggest");
  parser.add_positional("output");

  std::vector<std::string> args;

  // missing required flag
  args = {"progname", "-c", "c.ini", "in.txt"};
  ASSERT_THROW(parser.parse_with_exceptions(args), std::invalid_argument);

  // missing required argflag
  args = {"progname", "-?", "in.txt"};
  ASSERT_THROW(parser.parse_with_exceptions(args), std::invalid_argument);

  // missing required positional
  args = {"progname", "-?", "-c", "c.ini"};
  ASSERT_THROW(parser.parse_with_exceptions(args), std::invalid_argument);

  // has all required flags
  args = {"progname", "-?", "in.txt", "-c", "c.ini"};
  ASSERT_NO_THROW(parser.parse_with_exceptions(args));

  // required argflag ending prematurely
  args = {"progname", "-?", "in.txt", "-c"};
  ASSERT_THROW(parser.parse_with_exceptions(args), std::invalid_argument);

  // optional argflag ending prematurely
  args = {"progname", "in.txt", "-?", "-c", "c.ini", "-k"};
  ASSERT_THROW(parser.parse_with_exceptions(args), std::invalid_argument);
}

TEST_F(FunctionalTests, UseHelpWithoutRequiredFlags) {
  CliParser parser;
  parser.add_argflag("-x");
  parser.add_positional("infile");
  parser.set_required("infile");
  std::string usage_regex {
    "Usage:\n"
    "  progname --help\n"
    "  progname\n"
    "    \\[-x <val>\\]\n"
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
  ASSERT_THROW(parser.parse_with_exceptions({"progname", "-h"}),
               CliParser::HelpRequest);
  ASSERT_THROW(parser.parse_with_exceptions({"progname", "--help"}),
               CliParser::HelpRequest);
  assert_help_exit(parser, {"progname", "-h"}, usage_regex);
  assert_help_exit(parser, {"progname", "--help"}, usage_regex);
}

TEST_F(FunctionalTests, UsageWithoutParsing) {
  CliParser parser;
  parser.add_argflag("-x");
  parser.add_positional("infile");
  parser.set_required("infile");
  ASSERT_EQ(parser.usage(),
            "Usage:\n"
            "  <program-name> --help\n"
            "  <program-name>\n"
            "    [-x <val>]\n"
            "    <infile>\n"
            "\n"
            "Required Positional Arguments:\n"
            "  infile\n"
            "\n"
            "Optional Flags:\n"
            "  -h, --help    Print this help and exit\n"
            "  -x <val>\n"
            "\n");
  ASSERT_EQ(parser.usage("my-program-name"),
            "Usage:\n"
            "  my-program-name --help\n"
            "  my-program-name\n"
            "    [-x <val>]\n"
            "    <infile>\n"
            "\n"
            "Required Positional Arguments:\n"
            "  infile\n"
            "\n"
            "Optional Flags:\n"
            "  -h, --help    Print this help and exit\n"
            "  -x <val>\n"
            "\n");
}

TEST_F(FunctionalTests, UsageWithDescriptions) {
  CliParser parser;
  parser.set_program_description("Parse a file and add XML stuff.");
  parser.add_flag("-d", "--debug");
  parser.set_description("--debug", "Turn on debugging messages");
  parser.add_argflag("-x");
  parser.set_description("-x", "add this XML element");
  parser.add_argflag("--really-long-option-here");
  parser.set_description("--really-long-option-here", "Really long option");
  parser.add_positional("infile");
  parser.set_description("infile", "File to parse");
  parser.set_required("infile");
  std::string usage_regex {
    "Usage:\n"
    "  progname --help\n"
    "  progname\n"
    "    \\[-d\\]\n"
    "    \\[--really-long-option-here <val>\\]\n"
    "    \\[-x <val>\\]\n"
    "    <infile>\n"
    "\n"
    "Description:\n"
    "  Parse a file and add XML stuff.\n"
    "\n"
    "Required Positional Arguments:\n"
    "  infile        File to parse\n"
    "\n"
    "Optional Flags:\n"
    "  -d, --debug   Turn on debugging messages\n"
    "  -h, --help    Print this help and exit\n"
    "  --really-long-option-here <val>\n"
    "                Really long option\n"
    "  -x <val>      add this XML element\n"
    "\n"
  };
  assert_help_exit(parser, {"progname", "-h"}, usage_regex);
  assert_help_exit(parser, {"progname", "--help"}, usage_regex);
}

/// TODO: test a metavar instead of <val> in usage string
/// TODO: test making a later positional required, but not an earlier
/// TODO: line wrap long descriptions (program, flag, and positional)
