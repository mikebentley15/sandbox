#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using stringlist = std::vector<std::string>;
using intlist    = std::vector<int>;

const bool DEBUG = true;
//const bool DEBUG = false;
template <typename T>
void debug(const T &msg, const std::string &prefix = "") {
  std::cout << prefix << msg << std::cout;
}

stringlist read_stream(std::istream &in = std::cin) {
  stringlist inlist;
  std::string line;
  while (!in.eof()) {
    std::getline(in, line);
    if (line != "") {
      inlist.emplace_back(line);
    }
  }
  return inlist;
}

template <typename T>
std::vector<T> line_split(const std::string &line) {
  std::vector<T> split;
  T val;
  std::istringstream out(line);
  while (!out.eof()) {
    out >> val;
    split.emplace_back(val);
  }
  return split;
}

int main() {
  auto inlist = read_stream();
  int n = std::stoi(inlist[0]);
  auto X = line_split<int>(inlist[1]);
  auto W = line_split<int>(inlist[2]);

  double mean = 0.0;
  double weight_sum = 0.0;
  for (int i = 0; i < n; i++) {
    mean += X[i] * W[i];
    weight_sum += W[i];
  }
  mean /= weight_sum;

  std::cout << std::setprecision(1) << std::fixed << mean << std::endl;

  return 0;
}
