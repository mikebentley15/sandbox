#ifndef RADIX_SORT_H
#define RADIX_SORT_H

#include <utility>
#include <iostream>
#include <ios>
#include <iomanip>
#include <memory>

#include <cstring>

namespace {

template <int bits = 1>
void radix_sort_impl(int* arr, int* buf, size_t n) {
  int* from = arr;
  int* to = buf;

  int passes = 8 * sizeof(int) / bits;
  int bins = (1 << bits);

  // Do one pass to see how many of each mask we have.
  int bin_counts[passes][bins];
  memset(bin_counts,0,sizeof(int)*passes*bins);
  for (int i = 0; i < n; i++) {
    uint mask = bins - 1;
    for (int pass = 0; mask; pass++) {
      bin_counts[pass][(arr[i] & mask) >> (bits * pass)]++;
      mask <<= bits;
    }
  }

  uint mask = bins - 1;
  for (int pass = 0; mask; pass++) {
    // Calculate the beginning locations of each bin
    int substep = 1;
    int* bin_ptrs[bins];
    bin_ptrs[0] = to;
    for (int bin = 1; bin < bins; bin++) {
      bin_ptrs[bin] = bin_ptrs[bin-1] + bin_counts[pass][bin-1];
    }

    // Reading in from array, place them in to array in their sections
    int* from_end = from + n;
    for (int *p = from; p != from_end; p++) {
      int dest_bin = ((*p & mask) >> (pass * bits));
      *(bin_ptrs[dest_bin]++) = *p;
    }

    // Swap the from and to arrays
    std::swap(from, to);
    mask <<= bits;
  }

  // If the final destination array is not arr, then memcpy
  if (from != arr) {
    memcpy(arr, from, sizeof(int)*n);
  }
}

} // end of unnamed namespace

template <int bits = 1>
void radix_sort(int* arr, size_t n) {
  std::unique_ptr<int[]> buffer (new int[n]);
  radix_sort_impl<bits>(arr, buffer.get(), n);
}

#endif // RADIX_SORT_H
