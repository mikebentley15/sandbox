#include <chrono>
#include <ostream>
#include <vector>

template <typename Clock = std::chrono::steady_clock>
class Timer {
public:
  using clock      = Clock;
  using time_point = typename clock::time_point;
  using duration   = typename clock::duration;

public:
  void start() { start_time = clock::now(); }
  void stop()  { stop_time = clock::now();  }
  void append_measurement() { durations.emplace_back(stop_time - start_time); }
  const auto& measurements() const { return durations; }

private:
  time_point start_time;
  time_point stop_time;
  std::vector<duration> durations;
}; // end of class Timer

template <typename Clock>
std::ostream& operator<<(std::ostream &out, const Timer<Clock> &t) {
  out << '[';
  bool first = true;
  for (const auto &m : t.measurements()) {
    if (!first) { out << ", "; }
    first = false;
    out << std::chrono::duration_cast<std::chrono::nanoseconds>(m).count() << " ns";
  }
  out << ']';
  return out;
}
