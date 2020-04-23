// demo from youtube

#include <casadi/casadi.hpp>

#include <iostream>
#include <vector>

int main() {
  using namespace casadi;
  using vd = std::vector<double>;
  using vDM = std::vector<DM>;

  auto x = MX::sym("x", 2); // two states
  auto u = MX::sym("u");          // control
  auto ode = vertcat((1-x(1)*x(1))*x(0) - x(1) + u, x(0));

  std::cout << "ode: " << ode << "\n";

  auto f = Function("f", {x, u}, {ode}, {"x", "u"}, {"ode"});
  auto result = f(vDM{{vd{0.2, 0.8}}, {vd{0.1}}});

  std::cout << "f: " << f << "\n";
  std::cout << "f([0.2; 0.8], 0.1) = " << result << "\n";

  return 0;
}
