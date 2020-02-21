/**
 * Represents a cubic polynomial
 *
 * T is a type that has implemented operator+() and operator*()
 */
template <typename T=double>
struct Cubic {
  using ValueType = T;
  T c0, c1, c2, c3;

  /**
   * Evaluate the polynomial at x
   *
   * x can be a different type, as long as it can be multiplied by itself and
   * multiplied by the constants defined by type T.
   *
   * Return type is automatically determined by the type system depending on
   * types T and U.
   */
  template <typename U>
  auto operator() (U x) const {
    return c0 + c1*x + c2*x*x + c3*x*x*x;
  }
};
