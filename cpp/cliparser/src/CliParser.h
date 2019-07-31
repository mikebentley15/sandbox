#ifndef CLIPARSER_H
#define CLIPARSER_H

#include <algorithm>
#include <vector>
#include <stdexcept>
#include <string>

class CliParser {
public:
  CliParser(int argCount, const char* const argList[])
    : _args(argList, argList + argCount)
    , _is_parsed(argCount, false)
  {
    verify_program_name_exists();
  }
  CliParser(const std::vector<std::string> &args)
    : _args(args)
    , _is_parsed(args.size(), false)
  {
    verify_program_name_exists();
  }
  CliParser(std::vector<std::string> &&args)
    : _args(std::move(args))
    , _is_parsed(args.size(), false)
  {
    verify_program_name_exists();
  }

  const std::vector<std::string> &args() const { return _args; }
  const std::string program_name() const { return _args[0]; }
  std::vector<std::string> remaining_args() const {
    std::vector<std::string> remaining;
    for (std::size_t i = 1; i < _args.size(); i++) {
      if (!_is_parsed[i]) {
        remaining.emplace_back(_args[i]);;
      }
    }
    return remaining;
  }

  template <typename ... Args>
  bool has_argument(Args ... args) {
    return has_argument_impl(args ...);
  }

  bool has_argument(const std::string &arg) { return false; }
  bool has_argument(const std::string &arg1, const std::string &arg2)
  { return false; }

protected:
  void verify_program_name_exists() {
    if (_args.size() == 0) {
      throw std::invalid_argument(
            "CliParser expects at least one command-line argument");
    }
  }

  template <typename ... Args>
  bool has_argument_impl(const std::string &arg, Args ... args) {
    // we do the or in this order because we do not want to short circuit parsing for all of the args.
    // if both "-h" and "--help" are requested and are both given, then we would mark them as parsed
    // regardless of having both.
    return has_argument_impl(args ...) || has_argument_impl(arg);
  }

  bool has_argument_impl(const std::string &arg) {
    // mark the argument(s) as parsed
    auto iter = std::find(_args.begin(), _args.end(), arg);
    bool was_found = false;
    if (iter != _args.end()) {
      was_found = true;
      _is_parsed[std::distance(_args.begin(), iter)] = true;
    }
    return was_found;
  }

protected:
  std::vector<std::string> _args;
  std::vector<bool> _is_parsed;
};

#endif // CLIPARSER_H
