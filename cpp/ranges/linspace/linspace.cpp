#include "PairWise.hpp"

#include <concepts>
#include <iostream>
#include <ranges>
#include <stdexcept>
#include <utility>

namespace view {

/// Returns count evenly spaced samples, calculated over the interval [start,
/// stop] Returned as a std::view
auto linspace(size_t start, size_t stop, size_t count) {
  const auto range = stop - start;
  return std::views::iota(size_t{0}, count) |
         std::views::transform([start, range, count](size_t index) {
           return start + index * range / (count - 1);
         });
}

} // namespace view

namespace {

template <class A, class B, class C>
concept either = std::same_as<A, B> || std::same_as<A, C>;

template <class Range>
  requires either<std::ranges::range_value_t<Range>, std::size_t,
                  std::pair<std::size_t, std::size_t>>
std::ostream &operator<<(std::ostream &out, const Range &range) {
  out << "[ ";
  bool first = true;
  for (const auto &val : range) {
    if (!first) {
      out << ", ";
    }
    first = false;
    out << val;
  }
  out << " ]";
  return out;
}

template <class A, class B>
std::ostream &operator<<(std::ostream &out, const std::pair<A, B> &pair) {
  return out << "[ " << pair.first << ", " << pair.second << " ]";
}

} // namespace

#define PRINT_RANGE(x)                                                         \
  {                                                                            \
    auto val = x;                                                              \
    std::cout << #x << ": " << val << std::endl;                               \
  }

int main() {
  auto linspace1 = view::linspace(10, 20, 4);
  auto linspace2 = view::linspace(10, 20, 5);
  PRINT_RANGE(view::linspace(10, 20, 4));
  PRINT_RANGE(view::linspace(10, 20, 5));
  auto tmp = view::PairWise(linspace1);
  for (auto val : tmp) {
    std::cout << val;
  }
  std::cout << std::endl;
  // PRINT_RANGE(tmp);
  // PRINT_RANGE(view::pairwise(linspace1));
  // PRINT_RANGE(view::pairwise(linspace2));
  return 0;
}
