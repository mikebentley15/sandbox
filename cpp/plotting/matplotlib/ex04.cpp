#include <matplotlibcpp.h>

#include <vector>

#include <cmath>

namespace plt = matplotlibcpp;

int main() {
  using V = std::vector<double>;
  V t(1000);
  V x(t.size());

  for (V::size_type i = 0; i < t.size(); i++) {
    t[i] = i / 100.0;
    x[i] = sin(2.0 * M_PI * 1.0 * t[i]);
  }

  plt::xkcd();
  plt::plot(t, x);
  plt::title("AN ORDINARY SINE WAVE");
  plt::save("xkcd.png");
  plt::save("ex04.png");
  plt::save("ex04.svg");
  plt::show();
}
