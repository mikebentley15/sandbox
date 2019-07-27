#ifndef CLIPARSER_H
#define CLIPARSER_H

#include <algorithm>
#include <vector>
#include <string>

class CliParser {
public:
  CliParser(int argCount, const char* const argList[])
    : _args(argList, argList + argCount)
  {
    verify_program_name_exists();
  }
  CliParser(const std::vector<std::string> &args) : _args(args) {
    verify_program_name_exists();
  }
  CliParser(std::vector<std::string> &&args) : _args(std::move(args)) {
    verify_program_name_exists();
  }

  const std::vector<std::string> &args() const { return _args; }
  const std::string program_name() const { return _args[0]; }
  std::vector<std::string> remaining_args() const { return {}; }
  const bool has_argument(const std::string &arg) { return false; }
  const bool has_argument(const std::string &arg1, const std::string &arg2)
  { return false; }

protected:
  void verify_program_name_exists() {
    if (_args.size() == 0) {
      throw std::invalid_argument(
            "CliParser expects at least one command-line argument");
    }
  }

protected:
  std::vector<std::string> _args;
};

#endif // CLIPARSER_H
