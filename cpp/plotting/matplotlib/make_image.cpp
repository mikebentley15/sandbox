#include <matplotlibcpp.h>

#include <iostream>
#include <cmath>

namespace plt = matplotlibcpp;

int main() {
  unsigned char *x = new unsigned char[256*256];
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {
      x[i*256 + j] = std::max(i, j); // L_inf norm
    }
  }
  plt::imshow(x, 256, 256, 1, {{"cmap", "bone"}});
  plt::save("make_image.png");
  plt::save("make_image.svg");
  plt::show();
  return 0;
}
