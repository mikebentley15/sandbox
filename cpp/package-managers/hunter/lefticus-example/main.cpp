#include <string>
#include <iostream>

#include <fmt/format.h>

int main() {
  std::string s = fmt::format("My fmt_test output: {}", 42);
  std::cout << s << '\n';
  return 0;
}
