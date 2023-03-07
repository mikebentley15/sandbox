#ifndef HEAPQ_H
#define HEAPQ_H

#include <functional>  // for std::less
#include <stdexcept>   // for std::out_of_range
#include <utility>     // for std::swap
#include <vector>      // for std::vector

#include <cstdio>      // for std::puts & std::printf

namespace heapq {

template <typename T, typename Less = std::less<T>>
void heapify_down(std::vector<T> &values, size_t i, Less &lessfunc) {
  std::printf("  heapify_down(%lu)\n", i);
  const size_t N = values.size();
  if (i >= N) {
    throw std::out_of_range("heapify_down asked to work on an element past the end");
  }
  size_t smallest = i;
  const size_t left = 2 * i + 1;
  const size_t right = left + 1;
  
  if (left < N && lessfunc(values[left], values[smallest])) {
    smallest = left;
  }
  if (right < N && lessfunc(values[right], values[smallest])) {
    smallest = right;
  }
  if (i != smallest) {
    std::swap(values[i], values[smallest]);   // put smallest on this heap's top
    heapify_down(values, smallest, lessfunc); // fix sub heap property after swap
  }
}

template <typename T, typename Less = std::less<T>>
void heapify_up(std::vector<T> &values, size_t i, Less &lessfunc) {
  std::printf("  heapify_up(%lu)\n", i);
  if (i >= values.size()) {
    throw std::out_of_range("heapify_up asked to work on an element past the end");
  }
  const size_t parent = (i-1) / 2;
  if (lessfunc(values[i], values[parent])) {
    std::swap(values[i], values[parent]);
    heapify_up(values, parent, lessfunc);
  }
}

template <typename T, typename Less = std::less<T>>
void heapify(std::vector<T> &values, Less &lessfunc) {
  std::puts("  heapify()");
  for (size_t i = values.size()/2 - 1; ; --i) {
    std::printf("    heapify(): calling heapify_down(%lu)\n", i);
    heapify_down(values, i, lessfunc);
    if (i == 0) { break; }
  }
}

template <typename T, typename Less = std::less<T>>
T heappop(std::vector<T> &values, Less &lessfunc) {
  std::puts("  heappop()");
  if (values.size() == 0) {
    throw std::out_of_range("Cannot pop from an empty heap");
  }

  // Swap first and last elements
  // Extract the minimum element, using move if possible
  T minval = std::move(values.front());
  if (values.size() == 1) {
    values.pop_back();
  } else {
    values.front() = std::move(values.back());
    values.pop_back();
    // Fix the heap invariant by bubbling down the new top element [O(log(n))]
    heapify_down(values, 0, lessfunc);
  }

  // Return this minimum value
  return minval;
}

template <typename T, typename Less = std::less<T>>
void heappush(std::vector<T> &values, T val, Less &lessfunc) {
  std::puts("  heappush()");
  // Push this new value to the vector
  values.emplace_back(std::move(val));

  // Bubble this value up until heap property is restored
  heapify_up(values, values.size()-1, lessfunc);
}

template <typename T, typename Less = std::less<T>>
T heappushpop(std::vector<T> &values, T val, Less &lessfunc) {
  std::puts("  heappushpop()");
  const size_t N = values.size();
  if (N == 0) { return val; } // simplest case

  if (less(values.front(), val)) {
    // Swap the inserted value and the min, then heapify down
    std::swap(values.front(), val);
    heapify_down(values, 0, lessfunc);
  }
  // else val is already the min

  return val;
}

template <typename T, typename Less = std::less<T>>
T heapreplace(std::vector<T> &values, T val, Less &lessfunc) {
  std::puts("  heapreplace()");
  const size_t N = values.size();
  if (N == 0) {
    throw std::out_of_range("Cannot pop from an empty heap");
  }
  std::swap(values.front(), val);
  heapify_down(values, 0, lessfunc);
  return val;
}

} // end of namespace heapq

#endif // HEAPQ_H
