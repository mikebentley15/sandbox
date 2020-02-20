#include <matplotlibcpp.h>

#include <vector>

#include <cmath>

namespace plt = matplotlibcpp;

int main() {
  // Prepare data
  const int n = 5000;
  std::vector<double> x(n), y(n);
  for (int i = 0; i < n; i++) {
    const double t = 2 * M_PI * i / n;
    x[i] = 16 * sin(t) * sin(t) * sin(t);
    y[i] = 13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t);
  }

  // plot() takes an arbitrary number of (x,y,format) triples.
  // x must be iterable (that is, anything providing begin(x) and end(x)),
  // y must either be callable (providing operator() const) or iterable.
  //plt::plot(x, y, "r-", x, [](double d) { return 12.5 + abs(sin(d)); }, "k-");

  // The above segfaults for some reason...  This does not
  plt::plot(x, y, "r-");
  plt::plot(x, [](double d) { return 12.5 + abs(sin(d)); }, "k-");

  plt::save("ex03.png");
  plt::save("ex03.svg");
  plt::show();
}
