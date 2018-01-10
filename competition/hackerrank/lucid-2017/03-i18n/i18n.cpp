#include <algorithm>
#include <ios>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

using stringlist = std::vector<std::string>;
using intlist  = std::vector<int>;

stringlist read_input() {
  stringlist inlist;
  while (!std::cin.eof()) {
    std::string line;
    std::getline(std::cin, line);
    if (line.size() > 0) { // ignore empty lines
      inlist.emplace_back(line);
    }
  }
  return inlist;
}

template <typename T>
void print_lines(const std::vector<T> outlist) {
  for (auto val : outlist) {
    std::cout << val << std::endl;
  }
}

intlist str_to_ints(const std::string &instr) {
  if (instr == "") {
    return {};
  }
  intlist vals;
  std::istringstream streamer;
  streamer.str(instr);
  while(!streamer.eof()) {
    int val;
    streamer >> val;
    vals.emplace_back(val);
  }
  return vals;
}

std::string i18n_compress(const std::string &to_compress) {
  if (to_compress.size() <= 2) {
    return to_compress;
  }
  std::string compressed = std::to_string(to_compress.size() - 2);
  compressed.append(1u, to_compress.back());
  compressed.insert(0, 1u, to_compress[0]);
  return compressed;
}

int main() {
  auto inlist = read_input();
  stringlist compressed(inlist.size() - 1);
  //stringlist compressed(inlist.begin() + 1, inlist.end());
  //std::generate(compressed.begin(), compressed.end(), 
  std::transform(inlist.begin() + 1, inlist.end(),
                 compressed.begin(), i18n_compress);
  print_lines(compressed);
}

