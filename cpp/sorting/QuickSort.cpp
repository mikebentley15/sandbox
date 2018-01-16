#include <iostream>
#include <utility>

#include <cstddef>

namespace {
size_t median(int* arr, size_t a, size_t b, size_t c) {
  if (a <= b) { 
    if (b <= c) {        // a <= b <= c
      return b;
    } else if (a <= c) { // a <= c < b
      return c;
    } else {             // c < a <= b
      return a;
    }
  } else {               // b < a
    if (a <= c) {        // b < a <= c
      return a;
    } else if (b <= c) { // b <= c < a
      return c;
    } else {             // c < b < a
      return b;
    }
  }
}
size_t choose_pivot(int* arr, size_t n) {
  size_t median_value = median(arr, 0, (n-1)/2, n-1);
  return median_value;
}
} // end of unnamed namespace

void quick_sort(int* arr, size_t n) {
  // Base case
  if (n <= 1) {
    return;
  }
  if (n == 2) {
    if (arr[1] < arr[0]) {
      std::swap(arr[0], arr[1]);
    }
    return;
  }

  // Find the pivot and put it at the beginning
  size_t pivot = choose_pivot(arr, n);
  if (pivot != 0) {
    std::swap(arr[0], arr[pivot]);
  }

  // Separate the array into before pivot and after pivot
  int* p = arr+1;
  int* end = arr + n;
  while (p != end) {
    if (*p <= *arr) {
      p++;
    } else {
      end--;
      std::swap(*p, *end);
    }
  }

  // Put the pivot in the middle between the two sections
  if (p != arr + 1) {
    std::swap(*arr, *(p-1));
  }

  // Recursively sort the two halves
  size_t left_size = p - arr - 1;
  size_t right_size = n - left_size - 1;
  quick_sort(arr, left_size);
  quick_sort(p, right_size);
}

