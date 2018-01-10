#include <algorithm>
#include <ios>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <unordered_map>

#include <cstring>  // for memcpy
#include <cmath>    // for sqrt and atan2

using stringlist  = std::vector<std::string>;
using intlist     = std::vector<int>;

//const bool DEBUG = true;
const bool DEBUG = false;
template <typename T>
void debug(const T &item) {
  if (DEBUG) {
    std::cout << item << std::endl;
  }
}

#define my_assert(x) \
  if (!x) { \
    printf("Assertion failed!, line %d, %s", __LINE__, #x); \
  }

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
void print_lines(const std::vector<T> outlist, bool print = true) {
  if (print) {
    for (auto val : outlist) {
      std::cout << val << std::endl;
    }
  }
}

template <typename T>
std::vector<T> str_split(const std::string &instr) {
  if (instr == "") {
    return {};
  }
  std::vector<T> vals;
  std::istringstream streamer;
  streamer.str(instr);
  while(!streamer.eof()) {
    T val;
    streamer >> val;
    vals.emplace_back(val);
  }
  return vals;
}

int main() {
  // Read input
  auto inlist = read_input();
  int N = std::stoi(inlist[0]);
  int bits = 0;
  for (; N > 0; bits++) { N = N >> 1; }
  std::cout << (1 << bits) << std::endl;
}

