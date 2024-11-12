#include <vector>
#include <iostream>

#include <cassert>
#include <cstddef>


class Solution {
  template <typename T>
  using vector = std::vector<T>;

 public:
  vector<int> sortArray1(vector<int> &nums) {
    auto answer = vector<int>(nums.size());

    auto *from = &answer;
    auto *to = &nums;

    auto swap = [](auto &a, auto &b) {
      auto tmp = a;
      a = b;
      b = tmp;
    };

    auto merge = [&from, &to](size_t start, size_t mid, size_t end) {
      auto astart = start;
      auto i = start;
      auto j = mid;
      while (i < mid && j < end) {
        to->at(astart++) = (from->at(i) < from->at(j)) ? from->at(i++) : from->at(j++);
      }
      assert(i == mid || j == end);
      while (i < mid) {
        to->at(astart++) = from->at(i++);
      }
      while (j < end) {
        to->at(astart++) = from->at(j++);
      }
    };

    auto min = [](auto a, auto b) { return (a < b) ? a : b; };

    size_t N = nums.size();

    for (size_t merge_size = 1; merge_size < N; merge_size *= 2) {
      swap(to, from);  // this is why they were initialized backwards
      for (size_t start = 0; start < N; start += merge_size * 2) {
        size_t mid = min(start + merge_size, N);
        size_t end = min(mid + merge_size, N);
        merge(start, mid, end);
      }
    }
    if (to != &answer) {
      swap(to, from);
      merge(0, N, N); // copy to answer
    }
    return answer;
  }

  vector<int> sortArray2(vector<int> &nums) {
    size_t N = nums.size();
    if (N <= 1) { return nums; }

    int *beginning = nums.data();
    int *ending = beginning + N;
    for (size_t merge_size = 1; merge_size < N; merge_size *= 2) {
      int *end = beginning;
      for (int *start = beginning; start + merge_size < ending; start = end) {
        int *mid = minval(start + merge_size, ending);
        end = minval(mid + merge_size, ending);
        inplace_merge(start, mid, end);
      }
    }
    return nums;
  }

 private:
  static void swapint(int &a, int &b) {
    a ^= b;
    b ^= a;
    a ^= b;
  }

  template<typename T>
  static T minval(T a, T b) {
    return (a < b) ? a : b;
  }

  static void inplace_merge(int *start, int *mid, int *end) {
    int *a = start;
    int *awrap = start;

    while (start < mid && mid < end) {
      // a is pointing to min value in left side [start, mid)
      // mid is pointing to min value in right side [mid, end)
      if (*a < *mid) {
        if (a != start) { swapint(*a, *start); }
        ++a;
        ++start;
        if (a == mid) { a = awrap; }
      } else {
        swapint(*mid, *start);
        if (a == start) {
          a = mid;
          awrap = a;
        }
        if (a > awrap) {
          int *b = mid;
          while (b > a) { // bubble it down
            swapint(*b, *(b-1));
            --b;
          }
          ++a;
        }
        ++mid;
        ++start;
      }
    }
    assert(start == mid || mid ==end);
    while (start != mid && a != start) {
      swapint(*(start++), *(a++));
      if (a == mid) { a = awrap; } // wrap a between [awrap, mid)
    }
  }
};
