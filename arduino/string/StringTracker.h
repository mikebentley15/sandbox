#ifndef STRING_TRACKER_H
#define STRING_TRACKER_H

class StringTracker {
public:
  StringTracker (char *buffer, uint32_t capacity)
    : _buffer(buffer), _capacity(capacity), _size(0)
  {}

  char*       data()           { return _buffer;   }
  const char* data()     const { return _buffer;   }
  uint32_t    capacity() const { return _capacity; }
  uint32_t    size()     const { return _size;     }

  /// Empties the string
  void clear() { _buffer[0] = '\0'; _size = 0; }

  bool isEmpty() { return _size == 0; }

  /** Truncate to n characters.  If it's already less than that, then no change.
   *
   * Returns the size of the string after the operation.
   */
  uint32_t truncate (uint32_t n) {
    if (_size > n) {
      _size = n;
      _buffer[_size] = '\0';
    }
    return _size;
  }

  /// Appends a character.  Returns true if there was room for it.
  bool append (const char ch) {
    if (_size >= _capacity - 1) {
      return false;
    }
    _buffer[_size++] = ch;
    _buffer[_size]   = '\0';
    return true;
  }

  /** append all characters from str (of length n) to this string
   *
   * The given string does not need to be null terminated.  It will simply copy
   * the n characters over, regardless of their value.
   *
   * Returns the number of characters added, which will be less than n if this
   * string becomes full.
   */
  uint32_t extend (const char* str, const uint32_t n) {
    uint32_t count = 0;
    for (; count < n && _size < _capacity - 1; ++count) {
      _buffer[_size++] = str[count];
    }
    _buffer[_size] = '\0';
    return count;
  }

  /// extend a null-terminated string.  Returns the number of added characters
  uint32_t extend (const char* str) {
    uint32_t count = 0;
    for (; str[count] != '\0' && _size < _capacity - 1; ++count) {
      _buffer[_size++] = str[count];
    }
    return count;
  }

  /// convenience method for extending another StringTracker object
  uint32_t extend (const StringTracker &str) { return extend(str._buffer, str._size); }

private:
  char *_buffer = nullptr; // string storage (not owned by this class)
  uint32_t _capacity = 0;  // space provided by buffer (including null termination)
  uint32_t _size = 0;      // size of contents in buffer
                           // size will be between 0 and capacity-1
};

#endif // STRING_TRACKER_H
