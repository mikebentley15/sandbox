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

  // only one argument
  std::size_t count_argument(const std::string &arg) {
    // mark the argument(s) as parsed
    std::size_t count = 0;
    auto iter = std::find(_args.begin(), _args.end(), arg);
    while (iter != _args.end()) {
      count++;
      _is_parsed[std::distance(_args.begin(), iter)] = true;
      iter = std::find(iter+1, _args.end(), arg);
    }
    return count;
  }

  /// any number of arguments
  template <typename ... Args>
  std::size_t count_argument(const std::string &arg, Args ... args) {
    return count_argument(arg) + count_argument(args ...);
  }

  /// any number of arguments
  template <typename ... Args>
  bool has_argument(Args ... args) {
    return count_argument(args ...) > 0;
  }

  /// returns the next parameter after this given argument
  /// only returns the first instance, does not support repeats
  /// @throws std::invalid_argument if not found
  std::string get_param(const std::string &arg) {
    return "";
  }
  template <typename ... Args>
  std::string get_param(const std::string &arg, Args ... args) {
    return get_param(arg);
  }
protected:
  void verify_program_name_exists() {
    if (_args.size() == 0) {
      throw std::invalid_argument(
            "CliParser expects at least one command-line argument");
    }
  }

protected:
  std::vector<std::string> _args;
  std::vector<bool> _is_parsed;
};

#endif // CLIPARSER_H
