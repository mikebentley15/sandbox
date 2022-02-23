#include <memory>
#include <iostream>
#include <stdexcept>

namespace {

struct A {
  int x;
  int y;

  // std::uninitialized_copy() works with even explicit constructors
  explicit A(int _x, int _y = 0) : x(_x), y(_y) {
    std::cout << "Constructing A(x: " << x << ", y: " << y <<", addr: " << this << ")\n";
  }

  ~A() {
    std::cout << "Destructing A at " << this << "\n";
  }
};

template <typename T>
struct Arr {
  T *data;
  size_t size;

  template <typename Iterator>
  Arr(Iterator first, size_t n) {
    size = n;
    // allocate
    data = static_cast<T*>(std::aligned_alloc(alignof(T), sizeof(T) * size));
    if (!data) {
      throw std::bad_alloc();
    }
    std::uninitialized_copy_n(first, size, data);
  }

  template <typename Iterator>
  Arr(Iterator first, Iterator last)
    : Arr(first, std::distance(first, last)) {}

  ~Arr() {
    std::destroy(data, data+size); // call destructor
    std::free(data); // free allocated memory
  }
};

std::ostream& operator<<(std::ostream &out, const A &a) {
  out << "A(x: " << a.x << ", y: " << a.y << ", addr: " << &a << ")";
  return out;
}

template <typename T>
std::ostream& operator<<(std::ostream &out, const Arr<T> &a) {
  out << "Arr(data: " << a.data << ", size: " << a.size << "):\n";
  for (auto it = a.data, end = a.data + a.size; it != end; ++it) {
    out << "  " << *it << "\n";
  }
  return out;
}

} // end of unnamed namespace

int main() {
  int xs[] = {1, 2, 3, 4};
  auto sz = std::size(xs);
  A *pbuf = static_cast<A*>(std::aligned_alloc(alignof(A), sizeof(A) * sz));
  if (!pbuf) {
    std::cerr << "Could not allocate buffer\n";
    return 1;
  }

  auto first = pbuf;
  auto last = std::uninitialized_copy(xs, xs+sz, first);

  for (auto it = first; it != last; ++it) {
    std::cout << *it << "\n";
  }

  { // used within a class using the constructor with size
    Arr<A> arr(xs, sz);
    std::cout << arr << "\n";
  }

  { // used within a class using the constructor with iterators
    Arr<A> arr(xs, xs+sz);
    std::cout << arr << "\n";
  }

  { // used within a class using the copy constructor
    Arr<A> arr(first, last);
    std::cout << arr << "\n";
  }

  std::destroy(first, last); // call destructor
  std::free(pbuf); // free allocated memory

  return 0;
}
