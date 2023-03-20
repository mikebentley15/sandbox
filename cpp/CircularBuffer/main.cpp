#include "CircularBuffer.h"

#include <iostream>

namespace {

template <typename Container>
void print_container(std::ostream &out, const Container &container) {
  out << "[";
  bool first = true;
  for (auto &val : container) {
    if (!first) { out << ", "; }
    first = false;
    out << val;
  }
  out << "]";
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T> &vec) {
  print_container(out, vec);
  return out;
}

std::ostream& operator<<(std::ostream& out, const CircularBuffer &buf) {
  out << "CircularBuffer("
         "capacity: " << buf.capacity() << ", "
         "size: " << buf.size() << ") ";
  print_container(out, buf);
  return out;
}

} // end of unnamed namespace

int main(void) {
  CircularBuffer buf(10);
  std::cout << "empty:  " << buf << "\n";
  for (int i = 10; i <= 50; ++i) {
    buf.write(i);
    std::cout << "after writing " << i << ": " << buf.allVals() << "\n";
  }
  std::cout << std::flush;

  return 0;
}
