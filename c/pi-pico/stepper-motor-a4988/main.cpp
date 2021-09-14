#include "A4988.h"

#include <pico/stdlib.h>
#include <pico/time.h>

#include <functional>
#include <iostream>
#include <vector>

using SpinCallback = std::function<void(int)>;

namespace {

// ================
// Global Constants
// ================

const uint dir_pin  = 16;
const uint step_pin = 17;

const uint ms1_pin  = 20;
const uint ms2_pin  = 19;
const uint ms3_pin  = 18;

const uint led_pin  = 25; 

const int full_steps_per_rotation = 200;
const float degrees_per_full_step = 360.0f / full_steps_per_rotation;


// ================
// Helper Functions
// ================

void init_outpin(const uint pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_OUT);
}

// spin the motor, sleeping in between.  But 
template<typename StepperType>
void spin_motor(StepperType &stepper,
                const int steps_per_sec,
                const int secs,
                SpinCallback &after_step_callback) {
  uint64_t us_per_step = 1000000u / uint64_t(steps_per_sec);
  auto next_step = get_absolute_time();
  for (int i = 0; i < steps_per_sec * secs; ++i) {
    stepper.full_step();

    after_step_callback(i);

    next_step = delayed_by_us(next_step, us_per_step);
    if (time_us_64() + 100 <= to_us_since_boot(next_step)) {
      sleep_until(next_step);
    }
  }
}

void blink(uint interval_ms, int num = 1) {
  for (int i = 0; i < num; ++i) {
    gpio_put(led_pin, true);
    sleep_ms(interval_ms / 2);
    gpio_put(led_pin, false);
    sleep_ms(interval_ms / 2);
  }
}

void print_state(uint64_t time, StepMode mode,
                 int steps, int steps_per_sec) // negative if CCW
{
  float angle = float(steps) * degrees_per_full_step / float(mode);
  float angular_velocity =
      float(steps_per_sec) * degrees_per_full_step / float(mode);
  std::cout << time << ','
            << int(mode) << ','
            << angle << ','
            << angular_velocity << '\n';
}

} // end of unnamed namespace


// ====
// Main
// ====

int main() {
  // initialize I/O
  stdio_init_all();

  // initialize output pins
  init_outpin(dir_pin);
  init_outpin(step_pin);
  init_outpin(ms1_pin);
  init_outpin(ms2_pin);
  init_outpin(ms3_pin);
  init_outpin(led_pin);

  A4988<dir_pin, step_pin> stepper;

#ifdef ARMLAB_DEMO // faster demo version
  const int initial_blink_countdown = 5;
  const uint mode_blink_duration = 1; // secs - blinking when changing modes
  const int spin_seconds = 3;
  const int max_iter = 20;
  const int angular_speed_increment = 36;  // degrees per second
  const int print_frequency = 20; // Hz
#else // runtime code
  const int initial_blink_countdown = 20; // secs - blinking before beginning
  const uint mode_blink_duration = 2; // secs - blinking when changing modes
  const int spin_seconds = 10;
  const int max_iter = 20;
  const int angular_speed_increment = 36;  // degrees per second
  const int print_frequency = 20; // Hz
#endif // ARMLAB_DEMO

  // blink LED for a few seconds, doing a countdown.
  for (int i = initial_blink_countdown; i > 0; --i) {
    const uint initial_blink_duration =
        1000u / uint(initial_blink_countdown + 1);
    auto next_step = make_timeout_time_ms(1000u);
    blink(initial_blink_duration, i);
    sleep_until(next_step);
  }

  std::cout << "Beginning of the Experiment\n";

  std::cout << "time_ns,mode,angle,angular_velocity\n";

  std::vector<uint64_t> times;
  times.reserve(2 * print_frequency * spin_seconds);

  for (int iter = 1; iter <= max_iter; iter++) {
    int angular_speed = angular_speed_increment * iter;
    StepMode modes[] = {
      StepMode::FULL_STEP,
      StepMode::HALF_STEP,
      StepMode::QUARTER_STEP,
      StepMode::EIGHTH_STEP,
      StepMode::SIXTEENTH_STEP,
    };

    int mode_num = 0;
    for (auto mode : modes) {
      mode_num++;

      const int steps_per_sec =
          angular_speed * int(mode) * full_steps_per_rotation / 360;
      const int total_steps = steps_per_sec * spin_seconds;
      const int print_step_interval = steps_per_sec / print_frequency;

      SpinCallback capture_time = [&times, print_step_interval](int idx) {
          if (idx % print_step_interval == 0) {
            times.emplace_back(time_us_64());
          }
        };

      // blink out the mode (duration adjusted by # blinks)
      blink(mode_blink_duration / uint(mode_num), mode_num);
      print_state(time_us_64(), mode, 0, steps_per_sec);

      stepper.set_step_mode(mode, ms1_pin, ms2_pin, ms3_pin);

      gpio_put(led_pin, true);
      spin_motor(stepper, steps_per_sec, spin_seconds, capture_time);
      stepper.change_direction();

      gpio_put(led_pin, false);
      spin_motor(stepper, steps_per_sec, spin_seconds, capture_time);
      stepper.change_direction();

      auto end_time = time_us_64();

      for (size_t i = 0; i < times.size() / 2; ++i) {
        print_state(times[i], mode, i * print_step_interval, steps_per_sec);
      }
      for (size_t i = times.size() / 2; i < times.size(); ++i) {
        print_state(times[i], mode, 2 * total_steps - i * print_step_interval,
                    -steps_per_sec);
      }
      print_state(end_time, mode, 0, -steps_per_sec);
      times.clear();
    }
  }

  return 0;
}
