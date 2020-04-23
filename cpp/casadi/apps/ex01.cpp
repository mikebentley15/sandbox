#include <casadi/casadi.hpp>

#include <vector>

int main() {
  using namespace casadi;

  auto x = MX::sym("x", 5, 1);
  auto y = norm_2(x);
  auto grad_y = gradient(y, x);
  auto f = Function("f", {x}, {grad_y});
  auto grad_y_num = f(DM({1,2,3,4,5}));
  std::cout << grad_y_num << std::endl;

  return 0;
}
