#include "GridGenerator.h"

namespace {

/**
 * Creates a vector from begin to end at increments of diff.
 *
 * The first element is begin, the last element is end.  The end is guaranteed
 * to be at least diff away from the previous element (may be up to 2*diff).
 */
inline std::vector<double> range(double begin, double end, double diff) {
  std::vector<double> vals;
  vals.reserve(size_t((end - begin) / diff) + 2);
  for (double p = begin; p <= end - (diff / 2); p += diff) {
    vals.emplace_back(p);
  }
  vals.emplace_back(end);
  return vals;
}

} // end of unnamed namespace

const std::vector<GridGenerator::Iterator::VecType>
  GridGenerator::Iterator::empty;

GridGenerator::Iterator::Iterator(const std::vector<VecType> &vecs)
  : _vecs(vecs)
  , _is_end(false)
  , _N(vecs.size())
{
  for (auto &v : vecs) {
    _iters.emplace_back(v.begin());
    _current.emplace_back(v.front());
    if (v.size() == 0) {
      _is_end = true;
      break;
    }
  }
}

GridGenerator::Iterator& GridGenerator::Iterator::operator++() {
  if (_is_end) { return *this; }

  for (size_t i = 0; i < _N; ++i) {
    ++_iters[i];
    if (_iters[i] == _vecs[i].end()) {
      if (i < _N - 1) {
        _iters[i] = _vecs[i].begin();
        _current[i] = _vecs[i].front(); // update current
      } else {
        _is_end = true;
        return *this;
      }
    } else {
      _current[i] = *_iters[i];
      break;
    }
  }

  return *this;
}

bool GridGenerator::Iterator::operator== (const Iterator &other) const {
  if (_is_end && other._is_end) { return true;  }
  if (_is_end || other._is_end) { return false; }

  for (size_t i = 0; i < _N; ++i) {
    if (_iters[i] != other._iters[i]) {
      return false;
    }
  }
  return true;
}

void GridGenerator::add_dim(double lower, double upper, size_t N) {
  auto dx = (upper - lower) / (N - 1);
  add_dim(range(lower, upper, dx));
}
