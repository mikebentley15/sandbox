#include <casadi/casadi.hpp>

#include <iostream>
#include <vector>

// dynamic system
//
//   \dot{x}_1 = (1 - x_2^2) x_1 - x_2
//   \dot{x}_2 = x_1
//
//   x_1(0) = 0
//   x_2(0) = 1

int main() {
  using namespace casadi;

  auto x = MX::sym("x", 2);
  auto z = 1 - pow(x(1),2);
  auto rhs = vertcat(z * x(0) - x(1), x(0));

  MXDict ode;
  ode["x"]   = x;
  ode["ode"] = rhs;

  // construct an integrator that integrates over 4 seconds
  auto f = integrator("f", "cvodes", ode, {{"tf", 4}});

  // Start from x = [0; 1]
  auto result = f(DMDict{{"x0", std::vector<double>{0, 1}}});

  std::cout << "result: " << result << std::endl;

  return 0;
}
