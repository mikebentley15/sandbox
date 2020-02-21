#include <matplotlibcpp.h>

#include <iterator>
#include <stdexcept>
#include <vector>

#include <cmath>

namespace plt = matplotlibcpp;

struct Cubic {
  double c0, c1, c2, c3;
  double operator() (double x) const {
    return c0 + c1*x + c2*x*x + c3*x*x*x;
  }
};

class CubicSpline {
public:
  CubicSpline(double x1, double y1, double y1p, double x2, double y2, double y2p)
    : _x1(x1)
  {
    _c.c0 = y1;
    _c.c1 = y1p;
    double dx = x2 - x1;
    _c.c3 = (2*y1 - 2*y2 + dx*(y1p + y2p)) / (dx*dx*dx);
    _c.c2 = (y2p - y1p - 3 * _c.c3 * dx * dx) / (2 * dx);
  }

  double operator() (double x) const {
    double dx = x - _x1;
    return _c(dx);
  }

private:
  double _x1;
  Cubic _c;
};

class CubicSplineSequence {
public:
  CubicSplineSequence(std::vector<double> x, std::vector<double> y,
                      std::vector<double> yp)
    : _x(x)
  {
    if (x.size() != y.size() || x.size() != yp.size()) {
      throw std::out_of_range("Given vectors must be the same size");
    }
    if (x.size() < 2) {
      throw std::out_of_range("Must have at least two points to create cubic splines");
    }

    for (size_t i = 1; i < x.size(); i++) {
      if (x[i-1] >= x[i]) {
        throw std::invalid_argument("x vector must be monotonically increasing");
      }
      _splines.emplace_back(x[i-1], y[i-1], yp[i-1], x[i], y[i], yp[i]);
    }
  }

  double operator() (double x) const {
    // make sure x is in _x range
    if (x < _x.front() || x > _x.back()) {
      throw std::out_of_range("Outside spline interpolation range");
    }

    // find the correct spline such that x is in [spine.x1, spline.x2)
    auto it = std::lower_bound(_x.begin(), _x.end(), x);
    if (it == _x.begin()) {
      return _splines[0](x);
    } else {
      return _splines[std::distance(_x.begin(), it) - 1](x);
    }
  }

private:
  std::vector<double> _x;
  std::vector<CubicSpline> _splines;
};

int main() {
  double x1, y1, y1p, x2, y2, y2p, x3, y3, y3p;
  x1  = 1.0;  x2  = 2.5;   x3  = 5.0;
  y1  = 1.5;  y2  = 4.1;   y3  = 0.5;
  y1p = 1.5;  y2p = -2.5;  y3p = 2.5;

  using V = std::vector<double>;
  using std::hypot;

  double dx = 0.01;
  V xvec;
  for (double x = x1; x <= x3; x += dx) {
    xvec.push_back(x);
  }

  CubicSpline spline(x1, y1, y1p, x2, y2, y2p);
  CubicSplineSequence splines(
      V{x1, x2, x3}, V{y1, y2, y3}, V{y1p, y2p, y3p});

  //plt::quiver(V{x1, x2}, V{y1, y2},
  //            V{  1 / hypot(1, y1p),   1 / hypot(1, y2p)},
  //            V{y1p / hypot(1, y1p), y2p / hypot(1, y2p)});
  auto plot_tangent = [](double x, double y, double yp) {
    plt::plot(V{x-.2, x+.2}, V{y-.2*yp, y+.2*yp}, "r-");
  };
  plot_tangent(x1, y1, y1p);
  plot_tangent(x2, y2, y2p);
  plot_tangent(x3, y3, y3p);
  plt::plot(xvec, splines, "k-");
  plt::show();

  //// u and v are respectively the x and y components of the arrows
  //std::vector<int> x, y, u, v;
  //for (int i = -5; i <= 5; i++) {
  //  for (int j = -5; j <= 5; j++) {
  //    x.push_back(i);
  //    u.push_back(-i);
  //    y.push_back(j);
  //    v.push_back(-j);
  //  }
  //}

  //plt::quiver(x, y, u, v);
  //plt::save("ex05.png");
  //plt::save("ex05.svg");
  //plt::show();
}

