#include <matplotlibcpp.h>

#include <vector>

#include <cmath>

namespace plt = matplotlibcpp;

int main() {
  // prepare data
  int n = 5000;
  std::vector<double> x(n), y(n), z(n), w(n, 2);
  for (int i = 0; i < n; i++) {
    x[i] = i * i;
    y[i] = std::sin(2 * M_PI * i / 360.0);
    z[i] = std::log(i);
  }

  plt::figure_size(1200, 780);
  plt::plot(x, y);
  plt::plot(x, w, "r--");
  plt::named_plot("log(x)", x, z);
  plt::xlim(0, 1000 * 1000);
  plt::title("Sample figure");
  plt::legend();
  plt::save("ex02.png");
  plt::save("ex02.svg");
  plt::show();
}
