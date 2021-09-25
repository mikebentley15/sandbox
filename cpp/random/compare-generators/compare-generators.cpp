/**
 * Author:       Michael Bentley
 * Date:         25 Sept 2021
 * Description:  Compares performance of many pseudo-random number generators
 * 
 * See Also:
 * - https://thompsonsed.co.uk/random-number-generators-for-c-performance-tested
 * - https://nullprogram.com/blog/2017/09/21/
 * - https://medium.com/@odarbelaeze/how-competitive-are-c-standard-random-number-generators-f3de98d973f0
 * - https://arvid.io/2018/07/02/better-cxx-prng/
 * - https://www.random.org/randomness/
 * - https://en.cppreference.com/w/cpp/numeric/random
 */

#include "Timer.h"
#include "ScopedTiming.h"

#include <iostream>
#include <chrono>
#include <random>
#include <string>

#include <cstdlib>


template<typename GeneratorType, typename ClockType = std::chrono::steady_clock>
class Random {
public:
  using Generator = GeneratorType;
  using Clock = ClockType;
  using Distribution = std::uniform_int_distribution<int>;

public:
  Random(int begin, int end)
    : generator(Clock::now().time_since_epoch().count())
    , distribution(begin, end - 1)
  { }

  int next() { return distribution(generator); }

private:
  Generator generator;
  Distribution distribution;
};

template<typename RandType>
class RandomProfiler {
public:
  using Rand = RandType;

public:
  RandomProfiler(const std::string &name, Rand &rand, int n)
    : _name(name), _rand(rand), _n(n) {}

  const Timer<>& timer() const { return _timer; }

  void profile() {
    std::cout << "profiling " << _name << ", " << _n << " iterations" << std::endl;
    ScopedTiming stopwatch(_timer);
    (void)stopwatch; // suppresses unused warning
    for (int i = 0; i < _n; ++i) {
      _rand.next();
    }
  }

  std::ostream& print_report(std::ostream &out = std::cout) const {
    out << _name << ":  [";
    bool first = true;
    for (auto &m : _timer.measurements()) {
      if (!first) { out << ", "; }
      first = false;
      auto nanosecs = std::chrono::duration_cast<std::chrono::nanoseconds>(m).count();
      out << double(nanosecs) / double(_n) << " ns/call";
    }
    out << "]" << std::endl;
    return out;
  }

private:
  std::string _name;
  Rand &_rand;
  Timer<> _timer;
  int _n;
};

template <typename RandType>
RandomProfiler<RandType> gen_profiler(const std::string &name, RandType &r) {
  return RandomProfiler<RandType>(name, r);
}


int main() {
  constexpr int begin =  0;
  constexpr int end   = 10;
  constexpr int n = 300'000'000;
  constexpr int reps = 3;

// creates variables r and p
#define GEN_PROFILER(r, p, type) \
  Random<type> r(begin, end); \
  RandomProfiler<Random<type>> p(#type, r, n);
  
  GEN_PROFILER(r0,    p_r0,    std::minstd_rand0         );
  GEN_PROFILER(r,     p_r,     std::minstd_rand          );
  GEN_PROFILER(rc,    p_rc,    std::linear_congruential_engine<std::uint_fast32_t, 1664525, 1013904223, std::numeric_limits<std::unit_fast32_t>::max()>
  GEN_PROFILER(mt,    p_mt,    std::mt19937              );
  GEN_PROFILER(mt64,  p_mt64,  std::mt19937_64           );
  GEN_PROFILER(rl24,  p_rl24,  std::ranlux24_base        );
  GEN_PROFILER(rl48,  p_rl48,  std::ranlux48_base        );
  GEN_PROFILER(kb,    p_kb,    std::knuth_b              );
  GEN_PROFILER(def,   p_def,   std::default_random_engine);

#undef GEN_PROFILER

  std::cout << "sizeof(int): " << 8*sizeof(int) << " bits" << std::endl;

  for (int i = 0; i < reps; ++i) {
    std::cout << "\n  rep " << i+1 << "\n" << std::endl;
    p_r0  .profile();
    p_r   .profile();
    p_rc  .profile();
    p_mt  .profile();
    p_mt64.profile();
    p_rl24.profile();
    p_rl48.profile();
    p_kb  .profile();
    p_def .profile();
  }

  std::cout << std::endl;
  p_r0  .print_report(std::cout);
  p_r   .print_report(std::cout);
  p_rc  .print_report(std::cout);
  p_mt  .print_report(std::cout);
  p_mt64.print_report(std::cout);
  p_rl24.print_report(std::cout);
  p_rl48.print_report(std::cout);
  p_kb  .print_report(std::cout);
  p_def .print_report(std::cout);
  std::cout << std::endl;

  return 0;
}
