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
void spin_motor(A4988_simple &stepper,
                const int steps_per_sec,
                const int secs,
                SpinCallback &after_step_callback) {
  uint64_t us_per_step = 1000000u / uint64_t(steps_per_sec);
  absolute_time_t next_step = get_absolute_time();
  for (int i = 0; i < steps_per_sec * secs; ++i) {
    stepper.full_step();

    after_step_callback(i);

    next_step = delayed_by_us(next_step, us_per_step);
    sleep_until(next_step);
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

void print_state(absolute_time_t time, StepMode mode,
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

  A4988_simple stepper(dir_pin, step_pin);

  // blink LED for 10 seconds
  blink(1000u, 10);

  std::cout << "Beginning of the Experiment\n";

  const int spin_seconds = 10;
  const int max_iter = 20;
  const int angular_speed_increment = 36;  // degrees per second

  std::cout << "time_ns,mode,angle,angular_velocity\n";

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

      SpinCallback print_forward = [mode, steps_per_sec](int idx) {
          print_state(get_absolute_time(), mode, idx, steps_per_sec);
        };
      SpinCallback print_backward = [mode, steps_per_sec, total_steps](int idx) {
          print_state(get_absolute_time(), mode, total_steps - idx, -steps_per_sec);
        };

      // blink for 2 seconds (duration adjusted by # blinks)
      blink(2000u / uint(mode_num), mode_num);
      print_state(get_absolute_time(), mode, 0, steps_per_sec);

      stepper.set_step_mode(mode, ms1_pin, ms2_pin, ms3_pin);
      spin_motor(stepper, steps_per_sec, spin_seconds, print_forward);
      stepper.change_direction();
      spin_motor(stepper, steps_per_sec, spin_seconds, print_backward);
      stepper.change_direction();

      print_state(get_absolute_time(), mode, 0, -steps_per_sec);
    }
  }

  return 0;
}
