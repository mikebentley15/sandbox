#include <string>
#include <stdexcept>

#include <cstdio>

int maxval_1(int a, int b, int c) noexcept {
  if (a > b) {
    if (c > a) {
      return c;
    } else {
      return a;
    }
  } else if (c > b) {
    return c;
  } else {
    return b;
  }
}

int maxval(int a, int b) {
  return (a > b) ? a : b;
}

int maxval_2(int a, int b, int c) {
  return std::max(a, std::max(b, c));
}

int maxval_3(int a, int b, int c) {
  return maxval(a, maxval(b, c));
}

int maxval_4(int a, int b, int c) {
  if (a > b && a > c) { return a; }
  else if (b > c) { return b; }
  else { return c; }
}

void run_maxval(int a, int b, int c) {
  auto v1 = maxval_1(a, b, c);
  auto v2 = maxval_2(a, b, c);
  auto v3 = maxval_3(a, b, c);
  auto v4 = maxval_4(a, b, c);
  if (v1 == v2 && v3 == v4 && v1 == v3) {
    std::puts("All equal");
  } else {
    std::puts("One is not equal");
  }
}

int main([[maybe_unused]] int arg_count, char *arg_list[]) {
  int a, b, c;
  try {
    a = std::stoi(arg_list[1]);
    b = std::stoi(arg_list[2]);
    c = std::stoi(arg_list[3]);
  } catch (std::invalid_argument&) {
    std::puts("Invalid Argument");
    return 0;
  } catch (std::out_of_range&) {
    std::puts("Out of Range");
    return 0;
  }
  run_maxval(a, b, c);
  return 0;
}
