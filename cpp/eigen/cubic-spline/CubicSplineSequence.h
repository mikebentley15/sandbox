#include "CubicSpline.h"

#include <iterator>
#include <stdexcept>
#include <vector>

template <typename XType, typename YType>
class CubicSplineSequence {
public:
  using x_type = XType;
  using y_type = YType;

  CubicSplineSequence(std::vector<XType> x, std::vector<YType> y,
                      std::vector<YType> yp)
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

  YType operator() (XType x) const {
    // make sure x is in _x range
    if (!(x >= _x.front() && x <= _x.back())) {
      throw std::out_of_range("Outside spline interpolation range");
    }

    // find the correct spline such that x is in [spine.x1, spline.x2)
    // std::lower_bound() returns the first element A such that x <= A
    auto it = std::lower_bound(_x.begin(), _x.end(), x);
    if (it == _x.begin()) {
      return _splines[0](x);
    } else {
      return _splines[std::distance(_x.begin(), it) - 1](x);
    }
  }

private:
  std::vector<XType> _x;
  std::vector<CubicSpline<XType, YType>> _splines;
};
