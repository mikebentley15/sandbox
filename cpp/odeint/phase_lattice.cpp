#include <matplotlibcpp.h>
#include <boost/numeric/odeint.hpp>

#include <map>
#include <sstream>
#include <vector>

#include <cmath>
#include <cstdlib>

#ifndef UNUSED
#define UNUSED(x) (void)x
#endif

namespace ode = boost::numeric::odeint;
namespace plt = matplotlibcpp;

int main() {
  using State = std::vector<float>;
  using Map = std::map<std::string, std::string>;
  const size_t N = 128;
  const double gamma = 2.5;
  const double t_min = 0.0;
  const double t_max = 300;
  const double dt_i = 0.1;

  auto coupling = [gamma](float d) {
    return std::sin(d) - gamma * (1.0 - std::cos(d));
  };

  auto phase_lattice = [gamma, N, &coupling](
      const State &x, State &dxdt, double t)
  {
    UNUSED(t);
    for (size_t i = 1; i < N - 1; i++) {
      for (size_t j = 1; j < N - 1; j++) {
        dxdt[i*N + j] =
          coupling(x[(i+1)*N + j] - x[i*N + j]) +
          coupling(x[(i-1)*N + j] - x[i*N + j]) +
          coupling(x[i*N + j + 1] - x[i*N + j]) +
          coupling(x[i*N + j - 1] - x[i*N + j]);
      }
    }

    for (size_t i = 0; i < N; i++) {
      dxdt[i*N] = dxdt[i*N + N - 1] = 0.0;
    }
    for (size_t j = 0; j < N; j++) {
      dxdt[j] = dxdt[(N-1)*N + j] = 0.0;
    }
  };

  const size_t every = 20;
  size_t count = 0;
  auto observer = [N, &count, every](const State &x, double t) {
    UNUSED(t);
    if (count % every == 0) {
      std::ostringstream fname;
      fname << "phase_lattice_" << t << ".png";
      plt::clf(); // clear figure
      plt::imshow(&(x[0]), N, N, 1, Map{{"cmap", "bone"}});
      std::cout << "Writing " << fname.str() << std::endl;
      plt::save(fname.str());
    }
    count++;
  };

  // initialize the data
  State x(N * N); // zero initialized
  for (size_t i = (N/2 - 10); i < (N/2 + 10); i++) {
    for (size_t j = (N/2 - 1); j < (N/2 + 10); j++) {
      x[i*N + j] = static_cast<float>(std::rand()) / RAND_MAX * 2.0 * M_PI;
    }
  }

  ode::integrate_const(ode::runge_kutta4<State>(), phase_lattice,
                       x, t_min, t_max, dt_i, observer);

  return 0;
}
