// demo from youtube

#include <casadi/casadi.hpp>

#include <iostream>
#include <vector>

int main() {
  using namespace casadi;
  // DM = Matrix<double>
  // MX = SymbolMatrix
  // SX = Matrix<symbol>
  using vd = std::vector<double>;
  using vDM = std::vector<DM>;
  using vMX = std::vector<MX>;

  auto x = MX::sym("x", 2); // two states
  auto u = MX::sym("u");          // control
  auto ode = vertcat((1-x(1)*x(1))*x(0) - x(1) + u, x(0));

  std::cout << "ode: " << ode << "\n";

  auto F = Function("F", {x, u}, {ode}, {"x", "u"}, {"ode"});
  std::cout << F << "\n";
  std::cout << "F([0.2; 0.8], 0.1) = "
            << F(vDM{{vd{0.2, 0.8}}, {vd{0.1}}})
            << "\n";

  int T = 10;  // time horizon (seconds)
  int N = 20;  // number of control intervals

  // DAE problem structure
  //   "x": variable of integration
  //   "p": fixed variables (i.e., fixed values) throughout the integration
  //        can represent symbolic control to be solved later
  //   "ode": function being integrated
  MXDict dae {{"x", x}, {"p", u}, {"ode", ode}};
  Dict integrator_options {
    {"tf", double(T)/double(N)},
    {"simplify", true},
    {"number_of_finite_elements", 4},
  };

  // integrator to discretize the system
  //   "rk" = runge-kutta
  //   "ipopt" = ipopt
  auto f = integrator("f", "rk", dae, integrator_options);
  std::cout << "integrator: " << f << "\n";

  std::cout << "f([0;1], 0) = "
            << f(DMDict{{"x0", vd{0, 1}}, {"p", vd{0}}})
            << "\n";

  // one step simplified function
  auto symbolically_integrated = f(MXDict{{"x0", x}, {"p", u}});
  auto x_next = symbolically_integrated["xf"];
  std::cout << "x_next: " << x_next << "\n";
  auto fstep = Function("fstep", {x, u}, {x_next}, {"x", "u"}, {"x_next"});
  std::cout << fstep << "\n";
  std::cout << "fstep([0.1;0.9], 0.1) = "
            << fstep(vDM{{vd{0.1, 0.9}}, {vd{0.1}}})
            << "\n";

  // cascade one step by N times
  auto sim = fstep.mapaccum(N);
  std::cout << "sim: " << sim << "\n";

  
  auto sim_result = sim(MXDict{
    {"x", vd{0.0, 1.0}},
    {"u", sin(DM{{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20}})},
    });
  std::cout << "sim_result: " << sim_result << "\n";

  return 0;
}
