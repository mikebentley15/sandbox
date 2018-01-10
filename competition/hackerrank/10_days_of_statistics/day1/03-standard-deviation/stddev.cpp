#include <iomanip>
#include <iostream>

#include <cmath>

int main() {
  int n;
  std::cin >> n;
  auto X = new int[n];
  for (int i = 0; i < n; i++) {
    std::cin >> X[i];
  }
  auto mean = 0.0;
  for (int i = 0; i < n; i++) {
    mean += X[i];
  }
  mean /= n;
  auto stddev = 0.0;
  for (int i = 0; i < n; i++) {
    auto from_mean = X[i] - mean;
    stddev += from_mean * from_mean;
  }
  stddev = std::sqrt(stddev / n);
  std::cout << std::setprecision(1) << std::fixed << stddev << std::endl;
  return 0;
}
