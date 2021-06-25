#ifndef Array_h
#define Array_h

#include "move.h"

// similar to std::array<> (but simpler)
template<typename T, int _N>
class Array {
public:
  static int N() { return _N; }

  // direct getters
  T* data() { return _data; }
  const T* data() const { return _data; }
  int size() const { return _size; }
  int capacity() const { return _N; }

  // helper status functions
  bool is_full() const { return _size == _N; }
  bool is_empty() const { return _size == 0; }

  // index without bounds checking
  T& operator[](int pos) { return _data[pos]; }
  const T& operator[](int pos) const { return _data[pos]; }

  // first and last elements (with no bounds checking)
  T& front() { return _data[0]; }
  const T& front() const { return _data[0]; }
  T& back() { return _data[_size - 1]; }
  const T& back() const { return _data[size - 1]; }

  // use plain pointers as iterators
  T* begin() { return _data; }
  T* end() { return _data + size; }
  const T* cbegin() const { return _data; }
  const T* cend() const { return _data + size; }

  //
  // functions to mutate the array
  //

  // mark cells as empty, but don't do anything to the data
  void clear() { _size = 0; }

  // add to the array.  If it's already full, returns false and changes nothing
  bool append(T val) {
    if (this->is_full()) {
      return false;
    }
    _data[_size] = move(val);
    _size++;
    return true;
  }
  bool push(T val) { return append(move(val)); }

  // returns the value at the top and removes it (by simply decrementing size,
  // it is still there on the array)
  // If the array is already empty, it will not modify the array and will
  // return a default initialized value of T.
  T pop() {
    if (this->is_empty()) {
      return T{};
    }
    size--;
    return _data[size];
  }

private:
  T _data[_N] {};
  int _size = 0;
};

#endif // Array_h
