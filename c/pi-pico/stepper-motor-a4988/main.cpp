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
  uint64_t us_per_step = uint64_t(1000000u / steps_per_sec);
  absolute_time_t next_step = get_absolute_time();
  for (int i = 0; i < steps_per_sec * secs; ++i) {
    stepper.full_step();

    after_step_callback(i);

    next_step = delayed_by_us(next_step, us_per_step);
    sleep_until(next_step);
  }
}

void blink(int interval_ms, int num = 1) {
  for (int i = 0; i < num; ++i) {
    gpio_put(led_pin, true);
    sleep_ms(interval_ms / 2);
    gpio_put(led_pin, false);
    sleep_ms(interval_ms / 2);
  }
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
  blink(1000, 10);

  std::cout << "Beginning of the Experiment\n";

  const int spin_seconds = 10;
  const int max_iter = 8;
  const int angular_speed_increment = 36;  // degrees per second
  const int max_steps =
      max_iter * angular_speed_increment * spin_seconds    // degrees traveled
         * int(StepMode::SIXTEENTH_STEP) * full_steps_per_rotation / 360;

  struct MyState {
    absolute_time_t time;
    StepMode mode;
    int steps;         // total steps from the zero mark. negative if CCW
    int steps_per_sec; // degrees/sec. negative if going to CCW

    MyState(absolute_time_t t, StepMode m, int s, int spc)
      : time(t), mode(m), steps(s), steps_per_sec(spc) {}

    void print() const {
      float angle = steps * degrees_per_full_step / float(mode);
      float angular_velocity = steps_per_sec * degrees_per_full_step / float(mode);
      std::cout << time << ','
                << int(mode) << ','
                << angle << ','
                << angular_velocity << '\n';
    }
  };
  std::vector<MyState> state_cache;
  state_cache.reserve(2 * max_steps);

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

      const int steps_per_sec = angular_speed * int(mode) / degrees_per_full_step;
      const int total_steps = steps_per_sec * spin_seconds;

      SpinCallback capture_forward_state =
        [&state_cache, steps_per_sec, mode](int idx) {
          auto t = get_absolute_time();
          state_cache.emplace_back(t, mode, idx, steps_per_sec);
        };
      SpinCallback capture_reverse_state =
        [&state_cache, steps_per_sec, mode, total_steps](int idx) {
          auto t = get_absolute_time();
          state_cache.emplace_back(
              t, mode, total_steps - idx, -steps_per_sec);
        };

      // blink for 2 seconds (duration adjusted by # blinks)
      blink(2000 / mode_num, mode_num);
      MyState{get_absolute_time(), mode, 0, steps_per_sec}.print();

      stepper.set_step_mode(mode, ms1_pin, ms2_pin, ms3_pin);
      spin_motor(stepper, steps_per_sec, spin_seconds, capture_forward_state);
      stepper.change_direction();
      spin_motor(stepper, steps_per_sec, spin_seconds, capture_reverse_state);
      stepper.change_direction();

      // flush all states to the console
      for (auto &state : state_cache) {
        state.print();
      }
      state_cache.clear();
      MyState{get_absolute_time(), mode, 0, -steps_per_sec}.print();
    }

    sleep_ms(1000);
  }

  return 0;
}
