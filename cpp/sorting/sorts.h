#ifndef SORTS_H
#define SORTS_H

#include <cstddef>

void quick_sort(int* arr, size_t n);
void merge_sort(int* arr, size_t n);
//void radix_sort(int* arr, size_t n);

template <int bits = 1>
void radix_sort(int* arr, size_t n);
#include "RadixSort.h"

template <int bits = 1>
void american_flag_sort(int* arr, size_t n);
#include "AmericanFlagSort.h"

#endif // QUICK_SORT_H
