#ifndef A4988_H
#define A4988_H

#include <pico/types.h> // for uint
#include <pico/time.h>
#include <pico/stdlib.h>

enum class StepMode {
  FULL_STEP = 1,
  HALF_STEP = 2,
  QUARTER_STEP = 4,
  EIGHTH_STEP = 8,
  SIXTEENTH_STEP = 16,
};

struct A4988_simple {
public:
  A4988_simple(uint dir_pin, uint step_pin)
    : _dir_pin(dir_pin)
    , _step_pin(step_pin)
    , _is_high(true)
    , _is_forward(true)
  {
    gpio_put(_dir_pin, _is_forward);
    gpio_put(_step_pin, _is_high);
  }

  uint dir_pin() const { return _dir_pin; }
  uint step_pin() const { return _step_pin; }

  void half_step() {
    _is_high = !_is_high;
    gpio_put(_step_pin, _is_high);
  }

  // does two half steps with a delay in between
  void full_step(uint delay_us = 1) {
    half_step();
    sleep_us(delay_us);
    half_step();
  }

  // change direction.  Return true if now moving forward.
  void set_direction(bool forward) {
    _is_forward = forward;
    gpio_put(_dir_pin, _is_forward);
  }
  
  void set_forward() { set_direction(true); }
  void set_backward() { set_direction(false); }

  bool change_direction() {
    set_direction(!_is_forward);
    gpio_put(_dir_pin, _is_forward);
    return _is_forward;
  }

  static void set_step_mode(const StepMode mode,
                            const uint pin_m1,
                            const uint pin_m2,
                            const uint pin_m3)
  {
    auto set_mpins =
      [pin_m1, pin_m2, pin_m3] (bool m1_on, bool m2_on, bool m3_on) {
        gpio_put(pin_m1, m1_on);
        gpio_put(pin_m2, m2_on);
        gpio_put(pin_m3, m3_on);
      };

    switch (mode) {
      case StepMode::FULL_STEP:      set_mpins(false, false, false); break;
      case StepMode::HALF_STEP:      set_mpins(true , false, false); break;
      case StepMode::QUARTER_STEP:   set_mpins(false, true , false); break;
      case StepMode::EIGHTH_STEP:    set_mpins(true , true , false); break;
      case StepMode::SIXTEENTH_STEP: set_mpins(true , true , true ); break;
    }
    sleep_us(100);
  }

protected:
  uint _dir_pin;
  uint _step_pin;

  bool _is_forward;
  bool _is_high;
};

struct A4988 : public A4988_simple {
public:
  A4988(uint dir_pin, uint step_pin)
    : A4988_simple(dir_pin, step_pin)
    , _steps_per_sec(0)
    , _last_step()
  {}

  uint dir_pin() const { return _dir_pin; }
  uint step_pin() const { return _step_pin; }

  void set_speed(int steps_per_sec) {
    _steps_per_sec = steps_per_sec;
    // TODO: check if direction has changed
    // TODO: calculate us per step
    // TODO: calculate next timestamp
  }
  int speed() const { return _steps_per_sec; }

  // Checks if it's time to do another step.  If so, step.
  // Returns true if a step was taken.
  // Must be called at least once per step, but do it as often as possible
  bool update() {
    // TODO: implement: check if current timestamp is later than next step timestamp
    return false;
  }

protected:
  uint _dir_pin;
  uint _step_pin;
  int _steps_per_sec;
  
  absolute_time_t _last_step;
}; // end of class A4988

#endif // A4988_H
