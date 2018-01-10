#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>

using stringlist = std::vector<std::string>;
using intlist    = std::vector<int>;

//const bool DEBUG = true;
const bool DEBUG = false;
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

template <typename T>
void print_vector(const std::vector<T> &vec, const std::string &prefix = "",
                  bool should_print = true) {
  if (should_print) {
    for (const T &x : vec) {
      std::cout << x << std::endl;
    }
  }
}

int main() {
  auto inlist = read_stream();
  int n = std::stoi(inlist[0]);
  auto X = line_split<int>(inlist[1]);
  std::sort(X.begin(), X.end());
  print_vector(X, "", DEBUG);

  int mid1 = (n-1) / 2;
  int mid2 = n / 2;
  double q2 = (X[mid1] + X[mid2]) / 2.0;

  int l_mid1 = (mid2-1) / 2;
  int l_mid2 = mid2 / 2;
  double q1 = (X[l_mid1] + X[l_mid2]) / 2.0;

  int h_mid1 = mid1 + 1 + ((n - mid1 - 2) / 2.0);
  int h_mid2 = mid1 + 1 + ((n - mid1 - 1) / 2.0);
  double q3 = q3 = (X[h_mid1] + X[h_mid2]) / 2.0;

  std::cout << q1 << std::endl
            << q2 << std::endl
            << q3 << std::endl;

  return 0;
}
