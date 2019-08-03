#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include "CliParser.h"

#include <gtest/gtest.h>

#include <iostream>

/** Replaces a stream buffer with another to redirect output.
 *
 * Destructor restores the buffer.
 */
struct StreamBufReplace {
  std::ios &old_stream;
  std::ios &replace_stream;
  std::streambuf *old_buffer;
  StreamBufReplace(std::ios &_old_stream, std::ios &_replace_stream)
    : old_stream(_old_stream)
    , replace_stream(_replace_stream)
    , old_buffer(_old_stream.rdbuf())
  {
    old_stream.rdbuf(replace_stream.rdbuf());
  }

  ~StreamBufReplace() {
    old_stream.rdbuf(old_buffer);
  }
};


inline void assert_help_exit(CliParser &parser,
                             const std::vector<std::string> &args,
                             const std::string &usage)
{
  ASSERT_EXIT(
      StreamBufReplace replacer(std::cout, std::cerr);
      parser.parse(args);
  , testing::ExitedWithCode(0), usage);
}

inline void assert_error_exit(CliParser &parser, const std::vector<std::string> &args,
                       const std::string &message)
{
  ASSERT_EXIT(
      StreamBufReplace replacer(std::cout, std::cerr);
      parser.parse(args);
  , testing::ExitedWithCode(1), message);
}

#endif // TEST_HELPERS_H
