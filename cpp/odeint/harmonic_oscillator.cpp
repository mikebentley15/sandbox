#include <matplotlibcpp.h>

#include <boost/numeric/odeint.hpp>

#include <array>
#include <iostream>
#include <vector>

#ifndef UNUSED
#define UNUSED(x) (void)x
#endif

namespace ode = boost::numeric::odeint;
namespace plt = matplotlibcpp;

//using State = std::vector<double>;
using State = std::array<double, 2>;

/**
 * Represents a harmonic oscillator
 *   x''(t) = -x'(t) - gamma * x(t)
 */
class HarmonicOscillator {
public:
  HarmonicOscillator(double gamma) : _gamma(gamma) {}

  // calculate dx/dt from x and t
  void operator() (const State &x, State &dxdt, const double t) {
    UNUSED(t);
    dxdt[0] = x[1];
    dxdt[1] = -x[0] - _gamma * x[1];
  }

private:
  double _gamma;
};

int main() {
  double gamma = 0.15;
  auto harmonic_oscillator = [&gamma](const State &x, State &dxdt, const double t) {
    UNUSED(t);
    dxdt[0] = x[1];
    dxdt[1] = -x[0] - gamma * x[1];
  };

  State x0 {1.0, 0.0}; // start at x'(0) = 1, x(0) = 0
  State x = x0;
  const double t_min = 0.0;
  const double t_max = 10.0;
  const double initial_dt = 0.1;

  // using a lambda expression
  x = x0;
  auto steps = ode::integrate(harmonic_oscillator, x, t_min, t_max, initial_dt);

  // using a functor
  x = x0;
  steps = ode::integrate(HarmonicOscillator(gamma), x, t_min, t_max, initial_dt);

  // involving an observer that captures values
  x = x0;
  std::vector<double> tvec;
  std::vector<State> xvec;
  steps = ode::integrate(harmonic_oscillator, x, t_min, t_max, initial_dt,
      [&tvec, &xvec](const State &x, double t) {
        tvec.push_back(t);
        xvec.push_back(x);
      });

  std::cout << "Number of steps: " << steps << std::endl;

  std::vector<double> positions;
  std::vector<double> velocities;
  for (auto x : xvec) {
    velocities.emplace_back(x[0]);
    positions.emplace_back(x[1]);
  }

  plt::plot(tvec, positions, "k-");
  plt::plot(tvec, velocities, "r-");
  plt::save("harmonic_oscillator.png");
  plt::save("harmonic_oscillator.svg");
  plt::show();

  return 0;
}
