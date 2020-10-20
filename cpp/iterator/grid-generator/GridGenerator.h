#ifndef GRID_GENERATOR_H
#define GRID_GENERATOR_H

#include <vector>

class GridGenerator {
public:
  class Iterator {
  public:
    using VecType = std::vector<double>;
    using IterType = VecType::const_iterator;

    // the end iterator
    Iterator() : _vecs(empty), _is_end(true), _N(0) {}   // end iter
    explicit Iterator(const std::vector<VecType> &vecs); // begin iter

    Iterator& operator++(); // prefix
    operator bool() const { return !_is_end; } // checks for end too

    const VecType& operator*()  const { return  _current; }
    const VecType* operator->() const { return &_current; }

    bool operator== (const Iterator &other) const;
    bool operator!= (const Iterator &other) const { return !(*this == other); }

  private:
    static const std::vector<VecType> empty;

    const std::vector<VecType> &_vecs;
    bool                        _is_end;
    size_t                      _N;
    std::vector<IterType>       _iters;
    VecType                     _current;
  }; // end of class GridGenerator::Iterator

public:
  GridGenerator() {}

  void add_dim(const std::vector<double> &values) {
    _vecs.emplace_back(values);
  }

  void add_dim(double lower, double upper, size_t N);

  GridGenerator::Iterator begin() const {
    return GridGenerator::Iterator(_vecs);
  }

  GridGenerator::Iterator end()   const {
    return GridGenerator::Iterator();
  }

private:
  std::vector<std::vector<double>> _vecs;
}; // end of class GridGenerator

#endif // GRID_GENERATOR_H
