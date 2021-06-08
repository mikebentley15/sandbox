#ifndef STATIC_STRING_H
#define STATIC_STRING_H

#include "StringTracker.h"

/** A statically-sized string class */
template <uint32_t N>
class StaticString {
public:
  /// Constructors
  StaticString () : _impl(_data, N) {}
  StaticString (const char *str, uint32_t size) : _impl(_data, N) {
    _impl.extend(str, size);
  }
  StaticString (const char *str) : _impl(_data, N) {
    _impl.extend(str);
  }
  template <uint32_t N2>
  StaticString (const StaticString<N2> &other) : _impl(_data, N) {
    _impl.extend(other._impl);
  }
  StaticString (const StringTracker &str) : _impl(_data, N) {
    _impl.extend(str);
  }

  /// Interface follows the StringTracker class
  char*       data()           { return _data;        }
  const char* data()     const { return _data;        }
  uint32_t    capacity() const { return N;            }
  uint32_t    size()     const { return _impl.size(); }
  void clear() { _impl.clear(); }
  bool isEmpty() { return _impl.isEmpty(); }
  uint32_t truncate (uint32_t n) { return _impl.truncate(n); }
  bool append (const char ch) { return _impl.append(ch); }
  uint32_t extend (const char* str, const uint32_t n) { return _impl.extend(str, n); }
  uint32_t extend (const char* str) { return _impl.extend(str); }
  uint32_t extend (const StringTracker &str) { return _impl.extend(str); }
  template <uint32_t N2>
  uint32_t extend (const StaticString<N2> &other) { return _impl.extend(other._impl); }

private:
  char _data[N];
  StringTracker _impl;
};

#endif // STATIC_STRING_H
