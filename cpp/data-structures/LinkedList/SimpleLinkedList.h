#ifndef SIMPLE_LINKED_LIST_H
#define SIMPLE_LINKED_LIST_H

#include <iostream>  // for ostream

#include <cstddef>   // for size_t

template <class T>
class SimpleLinkedList {
public:
  /// Construct an empty linked list
  SimpleLinkedList() {
    
  }

  /// Return the number of elements in the list
  size_t size() const {
    return 0;
  }

  /// Returns the element at a particular location
  T& at(size_t idx) {
    return T();
  }
  const T& at(size_t idx) const {
    return T();
  }

  /// Insert the element at a particular location
  void insert(const T& value, size_t idx) {
    
  }
  /// Same, but move instead of copy
  void insert(T&& value, size_t idx) {
    
  }

  /// Remove an element
  void remove(size_t idx) {
    
  }

  void push_back(const T& value) {
    
  }
  void push_back(T&& value) {
    
  }

  void push_front(const T& value) {
    
  }
  void push_front(T&& value) {
    
  }

  T& pop() {
    
  }

  template <class InputIterator>
  void extend(InputIterator first, InputIterator last) {
    
  }

  /// Returns a new linked list with elements of this followed by other copied in.
  /// This is called when you do "a + b"
  SimpleLinkedList operator+ (const SimpleLinkedList& other) {
    return SimpleLinkedList();
  }

  /// Checks for contains using operator==()
  bool contains(const T& val) {
    return false;
  }
  /// Checks for contains using a custom eq(x,y) function
  template <class EqualsFunction>
  bool contains(const T& val, EqualsFunction eq) {
    return false;
  }

  bool operator==(const SimpleLinkedList& other) {
    return false;
  }

  bool operator!=(const SimpleLinkedList& other) {
    return false;
  }

private:

};

// Output the list to the stream in the form of a string.
// You can probably safely assume that type T already has operator<<()
// implemented.
template <class T>
std::ostream& operator<< (std::ostream& out, const SimpleLinkedList<T>& list) {
  return out;
}

#endif // SIMPLE_LINKED_LIST_H
