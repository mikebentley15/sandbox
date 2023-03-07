#ifndef MIN_HEAP_H
#define MIN_HEAP_H

#include "heapq.h"

#include <vector>

template <typename T, typename Less = std::less<T>>
class MinHeap {
public:
  MinHeap(Less lessfunc = Less()) : m_lessfunc(lessfunc) {}
  MinHeap(std::vector<T> &&values, Less lessfunc = Less())
    : m_lessfunc(lessfunc)
    , m_heap(std::move(values))
  { heapq::heapify(m_heap, m_lessfunc); }
  MinHeap(const std::vector<T> &values, Less lessfunc = Less())
    : m_lessfunc(lessfunc)
    , m_heap(values)
  { heapq::heapify(m_heap, m_lessfunc); }

  const T& peek() const { return m_heap.at(0); }
  T pop() { return heapq::heappop(m_heap, m_lessfunc); }
  void push(T val) { heapq::heappush(m_heap, val, m_lessfunc); }
  T pushpop(T val) { return heapq::heappushpop(m_heap, val, m_lessfunc); }
  T replace(T val) { return heapq::heapreplace(m_heap, val, m_lessfunc); }

  const std::vector<T>& data() const { return m_heap; }

private:
  Less m_lessfunc;
  std::vector<T> m_heap;
};

#endif  // MIN_HEAP_H
