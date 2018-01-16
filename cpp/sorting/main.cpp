#include "timeFunction.h"
#include "sorts.h"

#include <functional>
#include <algorithm>
#include <map>
#include <iostream>
#include <memory>
#include <random>

#include <cstring>
#include <cstddef>

template <typename T>
using SortFunction = std::function<void(T*,size_t)>;

void populate_dataset(int* arr, size_t n);
bool is_sorted(int* const arr, size_t n);
void experiment(const std::map<std::string, SortFunction<int>> &sort_map, size_t n);
void timesort_csv(const std::map<std::string, SortFunction<int>> &sort_map,
                  std::vector<size_t> sizes);

int main() {
  std::map<std::string, SortFunction<int>> sort_map = {
    //{"americanflagsort< 1>", american_flag_sort<1>},
    //{"americanflagsort< 2>", american_flag_sort<2>},
    //{"americanflagsort< 4>", american_flag_sort<4>},
    //{"americanflagsort< 8>", american_flag_sort<8>},
    {"mergesort", merge_sort},
    {"quicksort", quick_sort},
    {"radixsort< 1>", radix_sort<1>},
    {"radixsort< 2>", radix_sort<2>},
    {"radixsort< 4>", radix_sort<4>},
    {"radixsort< 8>", radix_sort<8>},
    {"radixsort<16>", radix_sort<16>},
    {"std::sort", [](int* arr, size_t n) { return std::sort(arr, arr+n); }},
  };

  experiment(sort_map, 20);

  //std::vector<size_t> sizes;
  //for (int i = 0; i < 22; i++) {
  //  sizes.push_back(1 << i);
  //}

  //timesort_csv(sort_map, sizes);

  return 0;
}

void populate_dataset(int* arr, size_t n) {
  std::mt19937 gen;
  std::uniform_int_distribution<int> dist;
  for (int i = 0; i < n; i++) {
    arr[i] = dist(gen);
  }
}

bool is_sorted(int* const arr, size_t n) {
  for (int i = 1; i < n; i++) {
    if (arr[i] < arr[i-1]) {
      return false;
    }
  }
  return true;
}

void experiment(const std::map<std::string, SortFunction<int>> &sort_map, size_t n) {
  using flit::time_function_autoloop;

  // Create a dataset
  std::unique_ptr<int[]> data_orig (new int[n]);
  std::unique_ptr<int[]> data_copy (new int[n]);
  populate_dataset(data_orig.get(), n);

  // Check that they all sort properly
  for (auto &kv : sort_map) {
    auto &k = kv.first;
    auto &v = kv.second;
    memcpy(data_copy.get(), data_orig.get(), sizeof(int)*n);
    v(data_copy.get(), n);
    if (!is_sorted(data_copy.get(), n)) {
      std::cout << "ERROR: " << k << " did not sort correctly\n";
      for (int i = 0; i < n; i++) {
        std::cout << "  " << data_copy[i] << std::endl;
      }
      return;
    }
  }

  auto to_time_empty = [n, &data_orig, &data_copy]() {
    // copy memory
    memcpy(data_copy.get(), data_orig.get(), sizeof(int)*n);
  };
  std::cout << "# elements = " << n << std::endl;
  auto empty_timing = time_function_autoloop(to_time_empty);
  std::cout << "Timing for empty timing = " << empty_timing << " nanosecs\n";
  for (auto &kv : sort_map) {
    auto &k = kv.first;
    auto &v = kv.second;
    auto to_time = [v, n, &data_orig, &data_copy]() {
      // copy memory
      memcpy(data_copy.get(), data_orig.get(), sizeof(int)*n);
      // sort
      v(data_copy.get(), n);
    };
    auto full_timing = time_function_autoloop(to_time);
    std::cout << "Timing for " << k << " = " << full_timing - empty_timing
              << " nanosecs\n";
  }
}

void timesort_csv(const std::map<std::string, SortFunction<int>> &sort_map,
                  std::vector<size_t> sizes)
{
  using flit::time_function_autoloop;
  int repeats = 3;

  // Output the csv header
  std::cout << "n";
  for (auto &kv : sort_map) {
    std::cout << "," << kv.first;
  }
  std::cout << std::endl;

  for (size_t n : sizes) {
    std::unique_ptr<int[]> data_orig (new int[n]);
    std::unique_ptr<int[]> data_copy (new int[n]);
    // Create a dataset
    populate_dataset(data_orig.get(), n);
    auto to_time_empty = [n, &data_orig, &data_copy]() {
      // copy memory
      memcpy(data_copy.get(), data_orig.get(), sizeof(int)*n);
    };
    auto empty_timing = time_function_autoloop(to_time_empty);
    std::cout << n;
    for (auto &kv : sort_map) {
      auto &v = kv.second;
      auto to_time = [v, n, &data_orig, &data_copy]() {
        // copy memory
        memcpy(data_copy.get(), data_orig.get(), sizeof(int)*n);
        // sort
        v(data_copy.get(), n);
      };
      auto full_time = time_function_autoloop(to_time);
      std::cout << "," << full_time - empty_timing;
    }
    std::cout << std::endl;
  }
}
