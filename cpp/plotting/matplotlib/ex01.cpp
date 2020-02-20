#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

int main(void) {
  plt::plot({1,3,2,4});
  plt::save("ex01.png");
  plt::save("ex01.svg");
  plt::show();
}
