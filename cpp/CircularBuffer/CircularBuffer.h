#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include "AtIterator.h"

#include <vector>    // for std::vector<>
#include <stdexcept> // for std::out_of_range
#include <iterator>  // for std::foward_iterator_tag

#include <cstdlib>   // for std::size_t

/// A circular buffer that reuses memory by circling around the end.
class CircularBuffer {
public:
  using Iterator = AtIterator<CircularBuffer, int>;
  using ConstIterator = AtIterator<const CircularBuffer, const int>;

public:
  /// Create a circular buffer with a preallocated capacity of size
  explicit CircularBuffer(size_t size) : m_capacity(size), m_data(), m_start(0) {
    m_data.reserve(m_capacity);
  }

  /** Write a new value to the end of the circular buffer
   *
   * If the buffer is full, then overwrite the oldest value with this new val
   *
   * Calls to this method invalidates any existing iterators
   */
  void write(int val) {
    // Note: this only works because we never remove from the buffer, only overwrite
    // If we want to convert this to a circular queue, we will need to implement
    // this differently.
    if (!is_full()) {
      m_data.emplace_back(val);
    } else {
      m_data[m_start] = val;
      m_start = index(m_start + 1);
    }
  }

  /// Return element at location i
  int&       operator[](size_t i)       { return m_data[index(m_start + i)]; }
  const int& operator[](size_t i) const { return m_data[index(m_start + i)]; }

  /// Same as operator[] but will throw if i >= size()
  int&       at(size_t i)       { bounds_check(i); return (*this)[i]; }
  const int& at(size_t i) const { bounds_check(i); return (*this)[i]; }

  // iterator support
  auto begin()        { return Iterator(this, 0);           }
  auto end()          { return Iterator(this, size());      }
  auto cbegin() const { return ConstIterator(this, 0);      }
  auto cend()   const { return ConstIterator(this, size()); }
  auto begin()  const { return cbegin();                    }
  auto end()    const { return cend();                      }

  /// Return the capacity of the buffer
  size_t capacity() const { return m_capacity; }

  /// Return the number of elements stored in the buffer
  size_t size() const { return m_data.size(); }

  /// Return true if the buffer size is equal to its capacity
  bool is_full() const { return size() == capacity(); }

  /// Return true if the buffer's size is zero
  bool is_empty() const { return m_data.empty(); }

  /// Return all values in a std::vector<int> from oldest to newest
  auto allVals() const {
    if (!is_full()) { return m_data; }
    return std::vector<int>(cbegin(), cend());
  }

private:

  /// Compute the index with wrapping around the capacity
  size_t index(size_t i) const { return i % capacity(); }

  /// Throw std::out_of_range if i >= size()
  void bounds_check(size_t i) const {
    if (i >= size()) {
      throw std::out_of_range("CircularBuffer::at() out of range");
    }
  }

private:

  size_t m_capacity;       ///< Max capacity of the buffer
  std::vector<int> m_data; ///< Storage
  size_t m_start;          ///< Location of the oldest buffer element

}; // end of class CircularBuffer

#endif // CIRCULAR_BUFFER_H
