#ifndef SIMPLE_SET_H
#define SIMPLE_SET_H 1

#define SIMPLE_SET_MULT_FACTOR 4

#include "util.h"       // for next_prime()

#include <cstddef>      // for std::size_t
#include <functional>   // for std::hash

//#include <iostream>

// TODO: play with different multiplicative factors
inline constexpr std::size_t allocate_size_for_capacity_1(std::size_t capacity) {
  return capacity * SIMPLE_SET_MULT_FACTOR;
}

// for collisions, it is best if this returns a prime number
inline constexpr std::size_t allocate_size_for_capacity_2(std::size_t capacity) {
  return next_prime(capacity * SIMPLE_SET_MULT_FACTOR);
}

inline constexpr std::size_t allocate_size_for_capacity(std::size_t capacity) {
  return allocate_size_for_capacity_2(capacity);
}

/** A simple set that does no dynamic allocations except at creation.
 *
 * It uses an open addressing approach to conflict resolution.  Since this
 * set is so simple, you can only insert, check if an element is contained,
 * and clear the set.  That means you cannot even remove elements unless
 * you remove all of them.
 *
 * If you want to use the data pointer directly, an empty location is the
 * same as the zero-initialized value of Key.  That means you cannot insert
 * an element that is equal to the zero-initialization value.
 */
template <class Key,
          std::size_t Capacity=25000,
          class Hash = std::hash<Key>,
          class Pred = std::equal_to<Key>
          >
class SimpleSet {
public:
  SimpleSet() noexcept
    : _size(0)
    , _internal_capacity(allocate_size_for_capacity(Capacity))
    , _collisions(0)
    , _hash()
    , _is_equal()
    , _data{} // zero initialize array
  {}

  /** Getters */
  inline int size() const noexcept { return _size; }
  inline int capacity() const noexcept { return Capacity; }
  inline long collisions() const noexcept { return _collisions; }

  /** Returns the raw internal data pointer. Use with care */
  inline const Key* data() const noexcept { return _data; }
  inline size_t data_size() const noexcept { return _internal_capacity; }

  /** Inserts an element into the set.
   *
   * Returns true if the element was not in the set (the opposite of
   * contains).
   */
  inline bool insert(const Key& elem) noexcept {
    auto idx = _hash(elem) % _internal_capacity;
    bool is_contained = false;
    for (auto current_idx = idx;
         current_idx != prev_idx(idx);
         current_idx = next_idx(current_idx))
    {
      //std::cout << "  insert(" << elem << ")" << std::endl;
      if (_is_equal(_data[current_idx], Key())) {
        _data[current_idx] = elem;
        break;
      } else if (_is_equal(_data[current_idx], elem)) {
        is_contained = true;
        break;
      }
      _collisions += 1;
    }
    return !is_contained;
  }

  /** Returns true if elem is in the set */
  inline bool contains(const Key& elem) const noexcept {
    auto idx = _hash(elem) % _internal_capacity;
    bool is_contained = false;
    for (auto current_idx = idx;
         current_idx != prev_idx(idx);
         current_idx = next_idx(current_idx))
    {
      //std::cout << "  contains(" << elem << ")" << std::endl;
      if (_is_equal(_data[current_idx], Key())) {
        is_contained = false;
        break;
      } else if (_is_equal(_data[current_idx], elem)) {
        is_contained = true;
        break;
      }
      _collisions += 1;
    }
    return is_contained;
  }

  /** Clears out the set.  It will be empty after this call */
  inline void clear() noexcept {
    for (int i = 0; i < _internal_capacity; i++) {
      _data[i] = Key();
    }
  }

private:
  /// Treats data as a circular buffer.
  inline size_t next_idx(size_t idx) const noexcept {
    return (idx + 1) % _internal_capacity;
  }

  /// Again treated as a circular buffer
  inline size_t prev_idx(size_t idx) const noexcept {
    return (idx + _internal_capacity - 1) % _internal_capacity;
  }

private:
  size_t _size;
  size_t _internal_capacity;
  mutable size_t _collisions;
  Hash _hash;
  Pred _is_equal;
  Key _data[allocate_size_for_capacity(Capacity)];
};

#endif // SIMPLE_SET_H
