#include <casadi/casadi.hpp>

#include <iostream>
#include <vector>

// composition to generate optimal control of a dynamical system

int main() {
  using namespace casadi;

  auto x = MX::sym("x", 2);  // two states
  auto p = MX::sym("p");     // one free parameter

  // ODE right-hand side, similar to ex02.cpp
  auto z = 1 - pow(x(1), 2);
  auto rhs = vertcat(z * x(0) - x(1) + 2 * tanh(p), x(0));

  MXDict ode {{"x", x}, {"p", p}, {"ode", rhs}};
  std::cout << ode << std::endl << std::endl;

  // construct an integrator that integrates over 1 second
  auto f = integrator("f", "cvodes", ode, {{"tf", 1}});

  // control vector
  auto u = MX::sym("u", 4, 1);

  x = DM(std::vector<double>{0, 1});  // initial state
  for (int k = 0; k < 4; ++k) {
    // integrate one second forward in time
    // call integrator symbolically
    auto result = f(MXDict{{"x0", x}, {"p", u(k)}});
    std::cout << "result(" << k << " seconds): " << result << std::endl;
    x = result["xf"];
  }

  // NLP declaration
  MXDict nlp {{"x", u}, {"f", dot(u, u)}, {"g", x}};
  std::cout << "\n"
               "nonlinear-program: " << nlp << std::endl;

  // solve using ipopt
  auto solver = nlpsol("solver", "ipopt", nlp);
  auto result = solver(DMDict{{"x0", 0.2}, {"lbg", 0}, {"ubg", 0}});

  std::cout << std::endl
            << "result: " << result << std::endl;

  return 0;
}

