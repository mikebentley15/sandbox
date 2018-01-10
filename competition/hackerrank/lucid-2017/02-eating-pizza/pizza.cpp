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

stringlist pizza_competition(const stringlist &inlist) {
  stringlist winners;
  for (int i = 1; i < inlist.size(); i++) {
    auto competition = str_to_ints(inlist[i]);
    if (competition.size() == 0) {
      continue;
    }
    int P = competition[0];
    int m = competition[1];
    int n = competition[2];
    if ((P % (m + n)) < m) {
      winners.emplace_back("Jeremy");
    } else {
      winners.emplace_back("Megan");
    }
  }
  return winners;
}

int main() {
  auto inlist = read_input();
  auto winners = pizza_competition(inlist);
  print_lines(winners);
}

