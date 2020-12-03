#ifndef UTIL_FUNCTION_TIMER_H
#define UTIL_FUNCTION_TIMER_H

#include "time_function_call.h"

#include <type_traits>
#include <utility>
#include <vector>

class FunctionTimer {
public:
  FunctionTimer() = default;

  // disable copy
  FunctionTimer(const FunctionTimer &other) = delete;
  FunctionTimer& operator= (const FunctionTimer &other) = delete;

  // default move
  FunctionTimer(FunctionTimer &&other) = default;
  FunctionTimer& operator= (FunctionTimer &&other) = default;

  void clear() { _times.clear(); }
  const std::vector<float>& get_times() const { return _times; }

  /// Time f().  This variant is used if f() returns void
  template <typename Func, std::enable_if_t<returns_void_v<Func>, int> = 0>
  auto time(Func &&f) {
    this->_times.emplace_back(time_function_call(f));
  }

  /// Time f() and return its return value
  template <typename Func, std::enable_if_t<returns_value_v<Func>, int> = 0>
  auto time(Func &&f) {
    float timing;
    auto val = time_function_call(f, timing);
    this->_times.emplace_back(timing);
    return val;
  }
  
private:
  std::vector<float> _times;
}; // end of class FunctionTimer

#endif // UTIL_FUNCTION_TIMER_H
