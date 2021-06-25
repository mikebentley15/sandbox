#ifndef TimeReporter_h
#define TimeReporter_h

#include "Array.h"

/** Report on multiple timings in micro seconds but only every so often
 *
 * @param N: number of timers
 * @param BufSize: buffer size of number of timing measurements per timer.
 *    Default value of 1000, so reporting only every 1000 timing samples (at
 *    least of the most frequent timer event).
 * 
 * Example usage:
 *
 *   enum class TimerType {
 *     TIMER_PIN_READ,
 *     TIMER_PIN_WRITE,
 *     TIMER_SENSOR_READ,
 *     TIMER_COUNT
 *   };
 *   const int TIMER_SAMPLE_SIZE = 10000; // report this often
 *   TimeReporter<TIMER_COUNT, TIMER_SAMPLE_SIZE> stopwatch;
 *
 *   void loop() {
 *     stopwatch.start(TIMER_PIN_READ);
 *     digitalRead(INPUT_PIN);
 *     stopwatch.stop(TIMER_PIN_READ);
 *
 *     out = (out == HIGH) ? LOW : HIGH;  // toggle
 *     stopwatch.start(TIMER_PIN_WRITE);
 *     digitalWrite(OUTPUT_PIN, out);
 *     stopwatch.stop(TIMER_PIN_WRITE);
 *
 *     // can also give a callable, like a function pointer, lambda expression,
 *     // or any class that implements operator()().
 *     int sensor_val;
 *     stopwatch.time(TIMER_SENSOR_READ, [&sensor_val]() {
 *         sensor_val = sensor.read();
 *       });
 *   }
 * 
 * Once any timer tries to exceed this limit, statistics will be generated and
 * output to the Serial channel, clearing all timer buffers.   This is done at
 * the stop() function when the time is added to the list of times.
 */
template <int N, int BufSize = 20>
class TimeReporter {
public:
  TimeReporter() {
    // make sure all names are initialized as nullptr, meaning not specified
    for (int idx = 0; idx < N; ++idx) { _names[idx] = nullptr; }
  }

  TimeReporter(const TimeReporter &other) = delete;            // disable copy
  TimeReporter(TimeReporter &&other) = default;                // enable move
  TimeReporter& operator=(const TimeReporter &other) = delete; // disable copy
  TimeReporter& operator=(TimeReporter &&other) = default;     // enable move
  ~TimeReporter() = default;

  // sets the name for the given timer.  bounds checking is not performed.
  void set_name(int idx, const char *name) {
    _names[idx] = name;
  }

  /** (re)starts the given timer.
   *
   * Bounds checking is not performed.
   *
   * Does not check if it's currently stopped.
   */
  void start(int idx) { _starts[idx] = micros(); }

  /** stops the given timer.
   *
   * Bounds checking is not performed.
   *
   * Does not check if it's currently stopped.  In that case, it will instead
   * record whatever micros() returns in the end.
   *
   * If the given timer is already full, then calculate stats for all timers,
   * report using the Serial stream, and then clear all timer buffers.
   * Note: if this reporting is performed, it may effect timings of other
   * timers that are currently going.  For that reason, it is recommended not
   * to overlap timing sections.
   */
  void stop(int idx) {
    auto end = micros();
    auto delta = end - _starts[idx];

    // if full, then generate and print the report
    if (_times[idx].is_full()) {
      auto before_report = end;
      Serial.print("TimeReporter: buffer index ");
      Serial.print(idx);
      Serial.println(" is full, generating report");
      this->report();
      this->clear();
      Serial.print("Reporting and clearing took ");
      auto report_delta = micros() - before_report;
      Serial.print(report_delta);
      Serial.print(" microseconds");
      Serial.println();
      Serial.println();
    }

    // add the time to the buffer
    _times[idx].append(delta);
  }

  /// a convenience way of passing in a callable, like a lambda
  template <typename Func>
  void time(int idx, const Func &f) {
    start(idx);
    f();
    stop(idx);
  }

  /// generates and sends the statistical report to Serial
  void report() {
    Serial.println("Time Report:");
    Serial.print("  # timers:   ");
    Serial.println(N);
    Serial.println("  times reported as <mean> (<min> - <max>) of <count>");


    for (int timer_idx = 0; timer_idx < N; ++timer_idx) {
      auto &current = _times[timer_idx];

      // print timer number and name
      Serial.print("  Timer ");
      Serial.print(timer_idx);
      if (_names[timer_idx] != nullptr) {
        Serial.print(" '");
        Serial.print(_names[timer_idx]);
        Serial.print("'");
      }
      Serial.print(":     ");

      if (current.is_empty()) {
        Serial.println("empty");
      } else {
        // calculate statistics
        unsigned long mymin, mymax, mysum;
        mymin = mymax = mysum = current[0];
        for (int idx = 1; idx < current.size(); ++idx) {
          mysum += current[idx];
          if (current[idx] > mymax) {
            mymax = current[idx];
          } else if (current[idx] < mymin) {
            mymin = current[idx];
          }
        }

        Serial.print(mysum / current.size());
        Serial.print(" (");
        Serial.print(mymin);
        Serial.print(" - ");
        Serial.print(mymax);
        Serial.print(") of ");
        Serial.print(current.size());
        Serial.println();
      }
    }

    Serial.println();
  }

  /// clear all timing buffers (but not the start times of running timers)
  void clear() { for (auto &buffer : _times) { buffer.clear(); } }

private:
  Array<unsigned long, BufSize> _times[N] {};
  unsigned long _starts[N] {};
  const char* _names[N] {};
};

#endif // TimeReporter_h
