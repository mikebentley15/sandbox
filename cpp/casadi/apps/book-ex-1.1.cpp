#include <casadi/casadi.hpp>

#include <vector>

// Example is
// 
//   min (x_1 - 2)^2 + (x_2 - 1)^2
//
//   s.t.
//
//   x_1^2 - x_2 <= 0
//   x_1 + x_2 <= 2

int main() {
  using namespace casadi;
  using vd = std::vector<double>;

  auto x = MX::sym("x", 2);
  auto f = pow(x(0) - 2, 2) + pow(x(1) - 1, 2);
  auto g = vertcat(x(0)*x(0) - x(1),
                   x(0) + x(1) - 2);  // g <= 0
  auto nlp_opts = MXDict{{"x", x}, {"f", f}, {"g", g}};
  auto solver = nlpsol("solver", "ipopt", nlp_opts);
  auto result = solver(DMDict{{"x0", vd{0.0, 0.0}}, {"ubg", 0}});
  std::cout << "\n"
            << solver << "\n"
            << "result: " << result << "\n";

  return 0;
}
