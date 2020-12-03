#ifndef TIME_FUNCTION_CALL_H
#define TIME_FUNCTION_CALL_H

#include "returns_void.h"

#include <chrono>
#include <type_traits>
#include <utility>

/// time a single invocation of f(), throwing away its return value
template <typename Func>
float time_function_call(Func &&f) {
  auto before = std::chrono::steady_clock::now();
  f();
  auto after = std::chrono::steady_clock::now();
  auto nanosec = std::chrono::duration_cast<
      std::chrono::nanoseconds>(after - before).count();
  return nanosec / 1.0e9f; // convert to seconds as a float
}

/** use this variant if you want the return value from f()
 *
 * A lot of effort went into making this usable if the function f() does not
 * have a return type (i.e. void).
 */
template <typename Func, std::enable_if<returns_void_v<Func>, int> = 0>
void time_function_call(Func &&f, float &time_out) {
  time_out = time_function_call(f);
}

template <typename Func, std::enable_if<returns_value_v<Func>, int> = 0>
auto time_function_call(Func &&f, float &time_out) {
  auto before = std::chrono::steady_clock::now();
  auto val = f();
  auto after = std::chrono::steady_clock::now();
  auto nanosec = std::chrono::duration_cast<
      std::chrono::nanoseconds>(after - before).count();
  time_out = nanosec / 1.0e9f; // convert to seconds as a float
  return val.val();
}

#endif // TIME_FUNCTION_CALL_H
