#include <vector>
#include <iostream>

template <typename T>
std::vector<std::vector<T>>
split_vector(const std::vector<T> &ti, const std::vector<int> &sizes) {
  std::vector<std::vector<T>> split;
  auto start = ti.begin();
  auto stop = start;
  for (int i = 0; i < sizes.size(); i++) {
    start = stop;
    stop += sizes[i];
    split.emplace_back(start, stop);
  }
  return split;
}

template <typename T>
std::ostream& operator<<(std::ostream& out, std::vector<T> v) {
  out << "[";
  bool first = true;
  for (T &x : v) {
    if (!first)
      out << ", ";
    first = false;
    out << x;
  }
  out << "]";
  return out;
}

int main() {
  auto v = split_vector<int>({1, 2, 3, 4, 5, 6, 7}, {3, 2, 2});
  auto &v1 = v[0];
  auto &v2 = v[1];
  auto &v3 = v[2];
  std::cout << "v:  " << v  << std::endl;
  std::cout << "v1: " << v1 << std::endl;
  std::cout << "v2: " << v2 << std::endl;
  std::cout << "v3: " << v3 << std::endl;
  return 0;
}
