#ifndef LINKED_LIST_H
#define LINKED_LIST_H

#include <initializer_list>
#include <iterator>

#include <cstddef>   // for size_t

/// A doubly-linked list class
/// Actually, implementing a singly-linked list
template <class T>
class LinkedList {
private:  // =private helper functions and helper classes
  
  
  
public:   // =type aliases
  using value_type = T;

  // skip the allocator_type, we don't need to go there.  It's too much effort
  //using allocator_type = 

  using reference = value_type&;
  using const_reference = const reference;
  using pointer = value_type*;
  using const_pointer = const pointer;

  // you will need to define these:
  // See: http://www.cplusplus.com/reference/iterator/

  // See: http://www.cplusplus.com/reference/iterator/BidirectionalIterator/
  //using iterator = ...
  //using const_iterator = ...

  // See: http://www.cplusplus.com/reference/iterator/reverse_iterator/
  //using reverse_iterator = ...
  //using const_reverse_iterator = ...

  // Once iterator is defined, you can uncomment this line
  //using difference_type = std::iterator_traits<iterator>::difference_types

  using size_type = std::size_t;

public:   // =public functions

  /// Empty Constructor - creates an empty LinkedList
  LinkedList();

  /// Fill Constructor - creates a LinkedList populated with n copies of val
  explicit LinkedList(size_type n, const_reference val = value_type());

  /** Range Constructor - copies elements from the iterators to populate this
   *
   * Copies the elements [first, last) -- meaning including the first, and up
   * to but not including the last.
   */
  template <class InputIterator>
  LinkedList (InputIterator first, InputIterator last);
  
  /// Copy Constructor - copies elements from another LinkedList
  LinkedList (const LinkedList& other);

  /// Move Constructor - moves all elements to this new LinkedList
  LinkedList (LinkedList&& other);

  /** Initializer List Constructor - creates a LinkedList from initializer list
   *
   * The initializer list format looks like the following:
   *
   *   LinkedList<int> ll = {1, 2, 4, 5}
   *
   * That linked list will be initially populated with the values [1, 2, 4, 5],
   * in that order.
   */
  LinkedList (std::initializer_list<value_type> il);

  /// Destructor
  ~LinkedList() noexcept;

  /// Uncomment these after you've made your iterator classes
  /// Returns an iterator to the beginning of the list
  //iterator begin() noexcept;
  //const_iterator begin() const noexcept;

  /// Returns an iterator to end of the list (right after the last element)
  //iterator end() noexcept;
  //const_iterator end() const noexcept;

  /// Returns a reverse iterator starting from the end of the list
  //reverse_iterator rbegin() noexcept;
  //const_reverse_iterator rbegin() const noexcept;

  /// Returns a reverse iterator for just before the beginning element
  //reverse_iterator rend() noexcept;
  //const_reverse_iterator rend() const noexcept;

  /// Returns a constant iterator to be beginning of the list
  //const_iterator cbegin() const noexcept;

  /** Returns a constant iterator to the end of the list
   *
   * (right after the last element)
   */
  //const_iterator cend() const noexcept;

  /// Returns a constant reverse iterator starting from the end of the list
  //const_reverse_iterator crbegin() const noexcept;

  /// Returns a constant reverse iterator for just before the beginning element
  //const_reverse_iterator crend() const noexcept;

  /// Returns true if this->size() == 0.
  bool empty() const noexcept;

  /// Returns the number of elements in the list
  size_type size() const noexcept;

  /** Returns the maximum number of elements that the list can hold.
   * 
   * Just return a very large number.  There are no guarantees on this number.
   */
  size_type max_size() const noexcept;

  /** Returns a reference to the first element in the list
   *
   * Calling this function on an empty container causes undefined behavior.
   * That means you don't need to implement or test that case :)
   */
  reference front();
  const_reference front() const;

  /** Returns a reference to the last element in the list
   *
   * Calling this function on an empty container causes undefined behavior.
   * That means you don't need to implement or test that case :)
   */
  reference back();
  const_reference back() const;

  /// Overwrite elements with new elements
  template <class InputIterator>
  void assign (InputIterator frist, InputIterator last); // Range version
  void assign (size_type n, const_reference value);      // Fill version
  void assign (std::initializer_list<value_type> il);    // Initializer list

  /// Add an element to the front, calling the constructor with these arguments
  template <class... Args>
  void emplace_front (Args&&... args);

  /// Add an element to the front, copying it or moving it
  void push_front (const_reference val);  // copy version
  void push_front (value_type&& val);     // move version

  /** Remove the front element.
   *
   * If it is empty already, then it is undefined behavior.
   */
  void pop_front();

  /// Add an element to the back, calling the constructor with these arguments
  template <class... Args>
  void emplace_back (Args&&... args);

  /// Add an element to the back, copying it or moving it
  void push_back (const_reference val);  // copy version
  void push_back (value_type&& val);     // move version

  /** Remove the back element.
   *
   * If it is empty already, then it is undefined behavior.
   */
  // commented out because only reasonable to do with a doubly-linked list
  //void pop_back();

  /** Add a new element at the given position, passing args to the constructor
   *
   * Returns an iterator to the newly added element
   */
  //template <class... Args>
  //iterator emplace (const_iterator position, Args&&... args);

  /// Add a new element at the given position
  //iterator insert (const_iterator position, const_reference val); // Single element
  //iterator insert (const_iterator position, size_type n, const_reference val);  // Fill
  //template <class InputIterator>
  //iterator insert (const_iterator position, InputIterator first, InputIterator last); // Range
  //iterator insert (const_iterator position, value_type&& val); // move
  //iterator insert (const_iterator position, std::initializer_list<value_type> il); // Initializer list

  /** Remove element(s).
   *
   * If the positions are not valid removable elements, then undefined behavior.
   *
   * Returns an iterator to the element after the last erased element.
   */
  //iterator erase (const_iterator position); // single element
  //iterator erase (const_iterator first, const_iterator last);  // range (not including last)

  /// Swap contents between two lists
  void swap (LinkedList& other);

  /** Resizes the list so that it contains n elements
   *
   * If n is smaller than size(), then remove all those after n elements.
   * If n is greater than size(), then add elements to the end (with val if
   * specified, otherwise default constructed objects).
   */
  void resize (size_type n);
  void resize (size_type n, const_reference val);

  /// Remove all elements in the list
  void clear() noexcept;

  /// Transfer elements from another list into this one
  //void splice (const_iterator position, LinkedList& other);  // move all
  //void splice (const_iterator position, LinkedList&& other); // move all
  //void splice (const_iterator position, LinkedList& other, const_iterator i); // single element
  //void splice (const_iterator position, LinkedList&& other, const_iterator i); // single element
  //void splice (const_iterator position, LinkedList& other,
  //             const_iterator first, const_iterator last); // range
  //void splice (const_iterator position, LinkedList&& other,
  //             const_iterator first, const_iterator last); // range

  /// Remove all elements with this specific value
  void remove (const_reference val);

  /// Removes from the container all elements for which pred(x) returns true.
  template <class Predicate>
  void remove_if (Predicate pred);

  /// Removes duplicate values
  void unique();  // Using operator==()
  template <class BinaryPredicate>
  void unique (BinaryPredicate binary_pred); // Using binary_pred(x, y)

  /// Merges two already sorted lists together -- moves from other into this one
  void merge (LinkedList& other);  // using operator<()
  void merge (LinkedList&& other); // using operator<()
  template <class Compare>
  void merge (LinkedList& other, Compare comp);  // with custom comp(x,y) function
  template <class Compare>
  void merge (LinkedList&& other, Compare comp); // with custom comp(x,y) function

  /// Sorts elements in the container
  void sort();  // using operator<()
  template <class Compare>
  void sort (Compare comp);  // with custom comp(x,y) function

  // Reverse the order of elements
  void reverse() noexcept;


private:  // =private variables
  
  
  
};

/** Function definitions **/


template <class T>
LinkedList<T>::LinkedList() {
  
}

template <class T>
LinkedList<T>::~LinkedList() noexcept {
  
}


#endif // LINKED_LIST_H
