#include <string>
#include <iostream>

std::string& replace_all(std::string& str, const std::string& needle,
                         const std::string& replacement)
{
  auto idx = str.find(needle, 0);
  while (idx != std::string::npos) {
    str.replace(idx, needle.size(), replacement);
    idx = str.find(needle, idx + replacement.size());
  }
  return str;
}

int main() {
  const std::string a = "These.are.the.best.times.of.my.life.";
  const std::string expected =
    "These.\nare.\nthe.\nbest.\ntimes.\nof.\nmy.\nlife.\n";

  std::string temporary = a;
  auto idx = temporary.find_first_of('.', 0);
  while (idx != std::string::npos) {
    temporary.replace(idx, 1, ".\n");
    idx = temporary.find_first_of('.', idx+2);
  }
  std::cout << "passed? " << ((temporary == expected) ? "true\n" : "false\n");
  std::cout << temporary << std::endl;
  std::cout << std::endl;

  temporary = a;
  std::cout << temporary << std::endl;
  replace_all(temporary, ".", ".\n");
  std::cout << "passed? " << ((temporary == expected) ? "true\n" : "false\n");
  std::cout << temporary << std::endl;
  std::cout << std::endl;

  return 0;
}
