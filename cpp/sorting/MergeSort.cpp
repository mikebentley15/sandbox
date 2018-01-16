#include <utility>
#include <memory>
#include <iostream>
#include <ostream>
#include <vector>

#include <cstring>

namespace {

std::ostream& operator<<(std::ostream& out, std::vector<int> v) {
  for (auto x : v) {
    out << "      " << x << std::endl;
  }
  return out;
}

void merge(int* dest, int* arr1, size_t n1, int* arr2, size_t n2) {
  int* end1 = arr1 + n1;
  int* end2 = arr2 + n2;

  while (arr1 != end1 && arr2 != end2) {
    if (*arr1 <= *arr2) {
      *dest++ = *arr1++;
    } else {
      *dest++ = *arr2++;
    }
  }
  if (arr1 != end1) {
    memcpy(dest, arr1, sizeof(int) * (end1 - arr1));
  }
  if (arr2 != end2) {
    memcpy(dest, arr2, sizeof(int) * (end2 - arr2));
  }
}

void merge_sort_rec(int* arr, int* buffer, size_t n) {
  // Base cases
  if (n <= 1) {
    return;
  }

  // Recursively sort the left and right halves
  size_t left_size = n / 2;
  size_t right_size = n - left_size;
  merge_sort_rec(arr, buffer, left_size);
  merge_sort_rec(arr + left_size, buffer + left_size, right_size);

  // Merge the two sorted halves
  merge(buffer, arr, left_size, arr + left_size, right_size);

  // Copy over from the buffer
  // TODO: instead of copying over, invert arr and buffer
  memcpy(arr, buffer, n * sizeof(int));
}

} // end of unnamed namespace

void merge_sort(int* arr, size_t n) {
  std::unique_ptr<int[]> buffer (new int[n]);
  merge_sort_rec(arr, buffer.get(), n);
}
