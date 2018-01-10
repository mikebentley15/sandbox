#ifndef UTIL_H
#define UTIL_H 1

#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#include <ctime>
#endif

#include <cstdint>      // for uint64_t

/**
 *
 * Function taken from StackOverflow:
 *   http://stackoverflow.com/questions/1861294/how-to-calculate-execution-time-of-a-code-snippet-in-c
 */
inline uint64_t get_time() {
#ifdef _WIN32
  /* Windows */
  FILETIME ft;
  LARGE_INTEGER li;

  // Get the amount of 100 nano seconds intervals elapsed since January 1, 1601
  // (UTC) and copy it to a LARGE_INTEGER structure.
  GetSystemTimeAsFileTime(&ft);
  li.LowPart = ft.dwLowDateTime;
  li.HighPart = ft.dwHighDateTime;

  uint64_t ret = li.QuadPart;
  ret -= 116444736000000000LL; // Convert from file time to UNIX epoch time.
  ret /= 10000;                // From 100 nano (10^-7) to 1 milli (10^-3) intervals

  return ret;
#else
  /* Linux */
  struct timeval tv;

  gettimeofday(&tv, NULL);

  uint64_t ret = tv.tv_usec;
  // Convert from micro seconds (10^-6) to milliseconds (10^-3)
  ret /= 1000;

  // Adds the seconds (10^0) after converting them to milliseconds (10^-3)
  ret += (tv.tv_sec * 1000);

  return ret;
#endif
}

/** Calculates the next prime number
 *
 * Code taken from StackOverflow:
 *   http://stackoverflow.com/questions/30052316/find-next-prime-number-algorithm
 */
inline constexpr bool is_prime(int number) {
  if (number == 2 || number == 3) {
    return true;
  }

  if (number % 2 == 0 || number % 3 == 0) {
    return false;
  }

  int divisor = 6;
  while (divisor * divisor - 2 * divisor + 1 <= number) {
    if (number % (divisor - 1) == 0) {
      return false;
    }
    if (number % (divisor + 1) == 0) {
      return false;
    }
    divisor += 6;
  }

  return true;
}

inline constexpr int next_prime(int a) {
  while (!is_prime(++a)) {}
  return a;
}

#endif // UTIL_H
