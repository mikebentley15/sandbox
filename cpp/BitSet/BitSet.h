#ifndef BIT_SET_H
#define BIT_SET_H

#include "AtIterator.h"

#include <bit>        // for std::popcount() -- c++20
#include <memory>     // for std::unique_ptr<>
#include <stdexcept>  // for std::out_of_range

#include <cstring>    // for std::memcmp
#include <cstdint>    // for uint64_t

/** An efficient array of booleans that can do some set-like operations.
 *
 * Similar to std::bitset but where size is not known until runtime.
 *
 * Each bool value is stored in a single bit.
 */
class BitSet {
public:
  using Iterator = AtIterator<BitSet, bool>;
  using ConstIterator = AtIterator<const BitSet, const bool>;

  class reference {
  public:
    explicit reference(uint64_t &val, uint8_t pos) : _val(val), _pos(pos) { }

    operator bool() const noexcept { return _val & (1 << _pos); }

    reference& operator = (bool x) noexcept {
      if (x) {
        _val |= (1 << _pos);
      } else {
        _val &= ~(1 << _pos);
      }
      return *this;
    }

    bool operator ~ () const noexcept { return !bool(*this); }
    reference& flip() noexcept { return *this = ~(*this); }

  private:
    uint64_t &_val;
    uint8_t _pos;
  };

public:
  explicit BitSet(size_t n)
    : _size(n)
    , _data(std::make_unique<uint64_t[]>(block_size(n))
  { }

  BitSet(const BitSet &other)
    : _size(other._size)
    , _data(std::make_unique<uint64_t[]>(block_size(n))
  {
    std::memcpy(_data.get(), other._data.get(),
                (block_size(n) * sizeof(uint64_t));
  }

  BitSet(BitSet &&other) = default; // move constructor

  BitSet& operator = (const BitSet &other) { // copy assignment
    _size = other._size;
    std::memcpy(_data.get(), other._data.get(),
                (block_size(_size) * sizeof(uint64_t));
  }

  BitSet& operator = (BitSet &&other) { // move assignment
    _size = other._size;
    _data = std::move(other._data);
    other._size = 0;
  }

  bool operator == (const BitSet &other) const noexcept {
    if (_size != other._size) { return false; }
    return 0 == std::memcmp(_data.get(), other._data.get(),
                            (block_size(_size) * sizeof(uint64_t));
  }

  bool operator != (const BitSet &other) const noexcept { return !(*this == other); }

  const uint64_t* data() const noexcept { return _data; }
  uint64_t* data() noexcept { return _data; }

  reference operator [] (size_t i)  noexcept { return reference(_data[i / 64], i % 64); }
  bool operator [] (size_t i) const noexcept { return reference(_data[i / 64], i % 64); }

  bool test(size_t i) const noexcept {
    if (i >= _size) {
      throw std::out_of_range("Bitset::test(): Beyond end");
    }
    return (*this)[i];
  }

  bool any() const noexcept {
    const auto end = block_size(_size);
    for (size_t i = 0; i < end; ++i) {
      if (_data[i]) {
        return true;
      }
    }
    return false;
  }

  bool all() const noexcept {
    const auto end = block_size(_size) - 1;
    // all but last one
    for (size_t i = 0; i < end; ++i) {
      if (_data[i] != UINT64_MAX) {
        return false;
      }
    }
    // check last one
    const uint64_t mask = make_mask(_size % 64);
    return (_data[end] & mask) == mask;
  }

  bool none() const noexcept { return !any(); }

  size_t count() const noexcept {
    const auto end = block_size(_size);
    size_t count_val = 0;
    for (size_t i = 0; i < end; ++i) {
      count_val += std::popcount(_data[i]);
    }
    return count_val;
  }

  size_t size() const noexcept { return _size; }

  auto& operator &= (const BitSet &other) noexcept {
    const auto end = block_size(std::min(_size, other._size));
    for (size_t i = 0; i < end; ++i) {
      _data[i] &= other._data[i];
    }
    return *this;
  }

  auto& operator |= (const BitSet &other) noexcept {
    const auto end = block_size(std::min(_size, other._size));
    for (size_t i = 0; i < end; ++i) {
      _data[i] |= other._data[i];
    }
    return *this;
  }

  auto& operator ^= (const BitSet &other) noexcept {
    const auto end = block_size(std::min(_size, other._size));
    for (size_t i = 0; i < end; ++i) {
      _data[i] ^= other._data[i];
    }
    return *this;
  }

  auto operator ~ () const noexcept {
    auto copy = *this;
    const auto end = block_size(_size) - 1;
    for (size_t i = 0; i < end; ++i) {
      copy._data[i] = ~(copy._data[i]);
    }
    // invert last one
    const uint64_t mask = make_mask(_size % 64);
    copy._data[end] = (~(copy._data[end])) & mask;
    return copy;
  }

  //auto operator << (size_t pos) const noexcept {
  //  auto copy = *this;
  //  // TODO: implement
  //  return copy;
  //}
  //auto& operator <<= (size_t pos) noexcept { return *this; }
  //auto operator >> (size_t pos) const noexcept { auto copy = *this; return copy; }
  //auto& operator >>= (size_t pos) noexcept { return *this; }

  // Set all bits to true
  BitSet& set() noexcept {
    const auto end = block_size(_size) - 1;
    for (size_t i = 0; i < end; ++i) {
      _data[i] = UINT64_MAX;
    }
    // set last one (not going off the end)
    _data[end] = make_mask(_size % 64);
  }

  BitSet& set(size_t pos, bool val = true) {
    (*this)[pos] = val;
    return *this;
  }

  BitSet& reset() noexcept {
    const auto end = block_size(_size);
    std::memset(_data.get(), 0, end * sizeof(uint64_t));
    return *this;
  }

  BitSet& reset(size_t pos) { return set(pos, false); }

  BitSet& flip() noexcept {
    const auto end = block_size(_size) - 1;
    for (size_t i = 0; i < end; ++i) {
      _data[i] = ~_data[i];
    }
    // set last one (not going off the end)
    _data[end] = (~data[i]) & make_mask(_size % 64);
    return *this;
  }

  BitSet& flip(size_t pos) {
    return set(pos, !bool((*this)[pos]));
  }
}
    

private:
  static constexpr size_t block_size(size_t n) noexcept { return (n+63) / 64; }
  static constexpr uint64_t make_mask(size_t count) noexcept {
    uint64_t mask = 0;
    for (size_t i = 0; i < count; ++i) {
      mask = (mask << 1) + 1;
    }
    return mask;
  }

private:
  size_t _size;
  std::unique_ptr<uint64_t[]> _data;
}; // end of class BitSet

#endif // BIT_SET_H
