#ifndef AT_ITERATOR_H
#define AT_ITERATOR_H

#include <iterator>  // for std::random_access_iterator_tag

#include <cstdlib>   // for std::size_t

/** Generic LegacyRandomAccessIterator class that uses operator[]
 *
 * @type Parent: any container type that implements operator[], where
 *   operator[] has the valid domain of 0
 * @type T: contained type. A reference to T should be returned from operator[].
 *
 * Note: You can make this a const iterator by passing in "const Parent" and
 * "const T" to the Parent and T types, respectively. (TODO: check this)
 */
template <typename Parent, typename T>
class AtIterator {
public:
  using value_type        = T;
  using reference         = value_type&;
  using pointer           = value_type*;
  using iterator_category = std::random_access_iterator_tag;
  using difference_type   = ssize_t;

  AtIterator(Parent *parent = nullptr, size_t i = 0)
    : _parent(parent), _i(i)
  { }

  constexpr bool operator==(const AtIterator &other) const noexcept = default;
  constexpr bool operator!=(const AtIterator &other) const noexcept = default;

  reference operator*()  { return (*_parent)[_i]; }
  pointer   operator->() { return &this->operator*(); }

  // prefix ++ operator
  auto& operator++() noexcept {
    ++_i;
    return *this;
  }

  // prefix -- operator
  auto& operator--() noexcept {
    --_i;
    return *this;
  }

  // postfix ++ operator
  auto operator++(int) noexcept {
    auto before = *this;
    ++(*this);
    return before;
  }

  // postfix -- operator
  auto operator--(int) noexcept {
    auto before = *this;
    --(*this);
    return before;
  }

  auto& operator+=(const difference_type diff) noexcept {
    _i += diff;
    return *this;
  }

  auto& operator-=(const difference_type diff) noexcept {
    _i -= diff;
    return *this;
  }

  auto operator+(const difference_type diff) const noexcept {
    return AtIterator(_parent, _i + diff);
  }

  auto operator-(const difference_type diff) const noexcept {
    return AtIterator(_parent, _i - diff);
  }

  difference_type operator-(const AtIterator &other) const noexcept {
    difference_type diff = _i;
    diff -= other._i;
    return diff;
  }

  reference operator[](size_t index) { return (*_parent)[_i + index]; }
  const reference operator[](size_t index) const { return (*_parent)[_i + index]; }

  bool operator< (const AtIterator &other) const noexcept { return _i <  other._i; }
  bool operator> (const AtIterator &other) const noexcept { return _i >  other._i; }
  bool operator<=(const AtIterator &other) const noexcept { return _i <= other._i; }
  bool operator>=(const AtIterator &other) const noexcept { return _i >= other._i; }

private:
  Parent *_parent;
  size_t _i;
}; // end of AtIterator

#endif // AT_ITERATOR_H
