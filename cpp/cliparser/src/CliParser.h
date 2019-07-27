#ifndef CLIPARSER_H
#define CLIPARSER_H

#include <algorithm>
#include <vector>
#include <string>

class CliParser {
public:
  CliParser(int argCount, const char* const argList[])
    : _args(argList, argList + argCount) {}
  CliParser(const std::vector<std::string> &args) : _args(args) {}
  const std::vector<std::string> &args() const { return _args; }
private:
  std::vector<std::string> _args;
};

#endif // CLIPARSER_H
