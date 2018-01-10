#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>

using stringlist = std::vector<std::string>;
using intlist    = std::vector<int>;

const bool DEBUG = true;
//const bool DEBUG = false;
template <typename T>
void debug(const T &msg, const std::string &prefix = "") {
  if (DEBUG) {
    std::cout << prefix << msg << std::endl;
  }
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

template <typename T>
class CompressedVector {
private:
  struct ValType {
    T val;            // value
    int frequency;    // frequency
    int ending_index; // index of where it ends
    ValType(const T &_val, int _freq, int _end)
      : val(_val), frequency(_freq), ending_index(_end) {}
  };
public:
  CompressedVector(const std::vector<T> &values,
                   const std::vector<int> frequencies)
  {
    int n = frequencies.size();
    std::vector<int> val_end(n);
    for (int i = 0; i < n; i++) {
      vals.emplace_back(values[i], frequencies[i], 0);
    }
    std::sort(vals.begin(), vals.end(),
        [](const ValType &a, const ValType &b) {
          return a.val < b.val;
        });
    int count = 0;
    for (int i = 0; i < n; i++) {
      count += vals[i].frequency;
      vals[i].ending_index = count;
    }
  }

  const size_t size() const {
    return vals.back().ending_index;
  }

  const T& at(int idx) const {
    auto it = std::find_if(vals.begin(), vals.end(),
        [idx](const ValType &v) {
          return idx < v.ending_index;
        });
    return it->val;
  }
  const T& operator[](int idx) const { return this->at(idx); }

  void print(bool should = true) {
    //print_vector(vals, "", should);
    if (!should) {
      return;
    }

    for (auto &val : vals) {
      std::cout << "("
                << val.val << ", "
                << val.frequency << ", "
                << val.ending_index << ")"
                << std::endl;
    }
  }

private:
  std::vector<ValType> vals;

};

int main() {
  auto inlist = read_stream();
  auto X = line_split<int>(inlist[1]);
  auto F = line_split<int>(inlist[2]);

  CompressedVector<int> cv(X, F);
  debug("\nCompressed Vector:");
  cv.print(DEBUG);

  debug("\nDecompresed Vector:");
  for (int i = 0; i < cv.size(); i++) {
    debug(cv.at(i));
  }
  debug("");

  int n = cv.size();

  int mid1 = (n-1) / 2;
  int mid2 = n / 2;
  //double q2 = (X[mid1] + X[mid2]) / 2.0;

  int l_mid1 = (mid2-1) / 2;
  int l_mid2 = mid2 / 2;
  double q1 = (cv[l_mid1] + cv[l_mid2]) / 2.0;

  int h_mid1 = mid1 + 1 + ((n - mid1 - 2) / 2.0);
  int h_mid2 = mid1 + 1 + ((n - mid1 - 1) / 2.0);
  double q3 = q3 = (cv[h_mid1] + cv[h_mid2]) / 2.0;

  std::cout << std::setprecision(1) << std::fixed
            << q3 - q1 << std::endl;

  return 0;
}
