/** Automatically starts and stops timer based on scope
 *
 * TimerClass expected to have the following interface:
 * - start()
 * - stop()
 * - append_measurement()
 *
 * Example:
 *
 *   Timer t;
 *   {
 *     ScopedTiming st(t);  // timer starts here
 *     // code to be timed
 *     // ...
 *
 *   } // timer stops here
 */
template<typename TimerClass>
class ScopedTiming {
public:
  using Timer = TimerClass;

public:
  ScopedTiming(Timer &t) : timer(t) { timer.start(); }
  ~ScopedTiming() { timer.stop(); timer.append_measurement(); }

private:
  Timer &timer;
}; // end of class ScopedTiming
