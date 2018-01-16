#ifndef AMERICAN_FLAG_SORT_H
#define AMERICAN_FLAG_SORT_H

#include <cstddef>

namespace {

template <uint bits = 1>
void american_flag_sort_rec(int* arr, size_t n, int pass) {
  std::cout << "american_flag_sort_rec("
            << arr << ", "
            << n << ", "
            << pass << ")\n";
  if (pass < 0) {
    return;
  }
  static_assert(false, "Not yet implemented");

  constexpr int tot_passes = 8 * sizeof(int) / bits;
  constexpr size_t bins = (1 << bits);
  size_t counts[bins];
  memset(counts, 0, sizeof(int) * (1 << bits));
  const uint mask = ((1 << bits) - 1) << (bits * pass);
  auto calc_bin = [mask, pass] (int x) {
    return (x & mask) >> (bits * pass);
  };
  for (int i = 0; i < n; ) {
    counts[calc_bin(arr[i])]++;
  }
  int* ptrs[bins];
  ptrs[0] = arr;
  for (size_t bin = 1; bin < bins; bin++) {
    ptrs[bin] = ptrs[bin-1] + counts[bin-1];
  }
  int* offsets[bins];
  memcpy(offsets, ptrs, bins*sizeof(int*));
  size_t current_bin = 0;
  for (int i = 0; i < n;) {
    while (arr+i < offsets[current_bin]) { current_bin++; }

    size_t bin = calc_bin(arr[i]);
    if (bin != current_bin) {
      *ptrs[bin]++ = arr[i];
    } else {
      i++;
    }
  }

  // tail recursion
  for (size_t bin = 0; bin < bins; bin++) {
    american_flag_sort_rec(offsets[bin], counts[bin], pass-1);
  }
}

} // end of unnamed namespace

// pseudo-code below:
//   american_flag_sort(Array, Radix)
//     for each digit D:
//       # first pass: compute counts
//       Counts <- zeros(Radix)
//       for object X in Array:
//         Counts[digit D of object X in base Radix] += 1
//       # compute bucket offsets
//       Offsets <- [ sum(Counts[0..i]) for i in 1..Radix]
//       # swap objects into place
//       for object X in Array:
//         swap X to the bucket starting at Offsets[digit D of X in base Radix]
//       for each Bucket:
//         american_flag_sort(Bucket, Radix)  
template <int bits = 1>
void american_flag_sort(int* arr, size_t n) {
  constexpr int tot_passes = 8 * sizeof(int) / bits;
  american_flag_sort_rec<bits>(arr, n, tot_passes - 1);
}

#endif // AMERICAN_FLAG_SORT_H


