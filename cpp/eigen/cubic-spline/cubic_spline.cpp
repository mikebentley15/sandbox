#include "CubicSplineSequence.h"

#include <matplotlibcpp.h>
#include <Eigen/Dense>

#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>

#include <cmath>

namespace plt = matplotlibcpp;
namespace E = Eigen;

void cubic_spline_double() {
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

  CubicSplineSequence<double, double> splines(
      V{x1, x2, x3}, V{y1, y2, y3}, V{y1p, y2p, y3p});

  auto plot_tangent = [](double x, double y, double yp) {
    plt::plot(V{x-.2, x+.2}, V{y-.2*yp, y+.2*yp}, "r-");
  };
  plot_tangent(x1, y1, y1p);
  plot_tangent(x2, y2, y2p);
  plot_tangent(x3, y3, y3p);
  plt::plot(xvec, splines, "k-");
  plt::save("cubic_spline.png");
  plt::save("cubic_spline.svg");
  plt::show();
}

void cubic_spline_vec3f() {
  double x1 = 1.0, x2 = 2.5, x3 = 5.0;
  E::Vector3f y1, y2, y3, y1p, y2p, y3p;
  y1  <<  1.5,  0.5,  4.1;
  y2  <<  4.1,  4.1,  1.5;
  y3  <<  0.5,  1.5,  0.5;
  y1p <<  1.5, -2.5, -1.0;
  y2p << -2.5,  2.5, -2.0;
  y3p <<  2.5, -1.5, -3.0;

  using Vd = std::vector<double>;
  using Vv = std::vector<E::Vector3f>;

  double dx = 0.01;
  Vd xvec;
  for (double x = x1; x <= x3; x += dx) {
    xvec.push_back(x);
  }

  CubicSplineSequence<double, E::Vector3f> splines(
      Vd{x1, x2, x3}, Vv{y1, y2, y3}, Vv{y1p, y2p, y3p});

  auto plot_tangent = [](double x, E::Vector3f y, E::Vector3f yp) {
    plt::plot(Vd{x-.2, x+.2}, Vd{y[0]-.2*yp[0], y[0]+.2*yp[0]}, "r-");
    plt::plot(Vd{x-.2, x+.2}, Vd{y[1]-.2*yp[1], y[1]+.2*yp[1]}, "r-");
    plt::plot(Vd{x-.2, x+.2}, Vd{y[2]-.2*yp[2], y[2]+.2*yp[2]}, "r-");
  };
  plot_tangent(x1, y1, y1p);
  plot_tangent(x2, y2, y2p);
  plot_tangent(x3, y3, y3p);

  auto spline_i = [&splines](int dimension) {
    return [&splines, dimension](double x) { return splines(x)[dimension]; };
  };
  plt::plot(xvec, spline_i(0), "k-");
  plt::plot(xvec, spline_i(1), "k-");
  plt::plot(xvec, spline_i(2), "k-");
  plt::show();
}

int main() {
  using std::cout;
  using std::endl;
  cout << "Beginning cubic spline experiments" << endl;
  
  cout << "  running cubic_spline_double()..." << endl;
  cubic_spline_double();

  cout << "  running cubic_spline_vec3f()..." << endl;
  cubic_spline_vec3f();

  cout << "Done." << endl;
  return 0;
}

