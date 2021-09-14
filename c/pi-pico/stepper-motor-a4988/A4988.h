/** A4988 class
 *
 * Originally developed for the Raspberry Pi Pico, this class can work with
 * both the Raspberry pi pico and Arduino (using ifdef's).  It's a very simple
 * implementation and assumes that you are using the A4988 chip driver to talk
 * with a stepper motor.
 *
 * This uses the DIO2 Arduino library when compiling for Arduino.  It makes
 * communication on GPIO very fast.
 *
 * This library was partially inspired by the AccelStepper library, but made to
 * be simpler and specific to using the A4988 chip.
 */

#ifndef A4988_H
#define A4988_H

#ifdef PICO_BUILD

#  include <pico/types.h> // for uint
#  include <pico/time.h>
#  include <pico/stdlib.h>

#  define  A4988_set_pin(pin, is_high)  gpio_put(pin, is_high)
#  define  A4988_sleep_micros(x)        sleep_us(x)

using A4988_PinType = uint;

#elif defined(ARDUINO)

#  include <Arduino.h>
#  include <DIO2.h>

#  define  A4988_set_pin(pin, is_high)  digitalWrite2f(pin, is_high ? HIGH : LOW)
#  define  A4988_sleep_micros(x)        delay_micros(x)

using A4988_PinType = GPIO_pin_t;

#else
#  error "Does not appear to be the Pi Pico or Arduino"
#endif

enum class StepMode {
  FULL_STEP = 1,
  HALF_STEP = 2,
  QUARTER_STEP = 4,
  EIGHTH_STEP = 8,
  SIXTEENTH_STEP = 16,
};

template <A4988_PinType DirPin, A4988_PinType StepPin>
struct A4988 {
public:
  A4988()
    : _is_forward(true)
  {
    A4988_set_pin(DirPin, _is_forward);
    A4988_set_pin(StepPin, false);
  }

  A4988_PinType dir_pin() const { return DirPin; }
  A4988_PinType step_pin() const { return StepPin; }

  // does two half steps with a delay in between
  void full_step(uint delay_us = 1) {
    A4988_set_pin(StepPin, true);
    A4988_sleep_micros(delay_us);
    A4988_set_pin(StepPin, false);
  }

  // change direction.  Return true if now moving forward.
  void set_direction(bool forward) {
    _is_forward = forward;
    A4988_set_pin(DirPin, _is_forward);
  }
  
  void set_forward() { set_direction(true); }
  void set_backward() { set_direction(false); }

  bool change_direction() {
    set_direction(!_is_forward);
    A4988_set_pin(DirPin, _is_forward);
    return _is_forward;
  }

  static void set_step_mode(const StepMode mode,
                            const uint pin_m1,
                            const uint pin_m2,
                            const uint pin_m3)
  {

#define A4988_SET_MPINS(m1_on, m2_on, m3_on) \
        A4988_set_pin(pin_m1, m1_on); \
        A4988_set_pin(pin_m2, m2_on); \
        A4988_set_pin(pin_m3, m3_on)

    switch (mode) {
      case StepMode::FULL_STEP:      A4988_SET_MPINS(false, false, false); break;
      case StepMode::HALF_STEP:      A4988_SET_MPINS(true , false, false); break;
      case StepMode::QUARTER_STEP:   A4988_SET_MPINS(false, true , false); break;
      case StepMode::EIGHTH_STEP:    A4988_SET_MPINS(true , true , false); break;
      case StepMode::SIXTEENTH_STEP: A4988_SET_MPINS(true , true , true ); break;
    }
    A4988_sleep_micros(100);

#undef A4988_SET_MPINS

  }

protected:
  bool _is_forward;
};

//struct A4988 : public A4988_simple {
//public:
//  A4988(uint dir_pin, uint step_pin)
//    : A4988_simple(dir_pin, step_pin)
//    , _steps_per_sec(0)
//    , _last_step()
//  {}
//
//  void set_speed(int steps_per_sec) {
//    _steps_per_sec = steps_per_sec;
//    // TODO: check if direction has changed
//    // TODO: calculate us per step
//    // TODO: calculate next timestamp
//  }
//  int speed() const { return _steps_per_sec; }
//
//  // Checks if it's time to do another step.  If so, step.
//  // Returns true if a step was taken.
//  // Must be called at least once per step, but do it as often as possible
//  bool update() {
//    // TODO: implement: check if current timestamp is later than next step timestamp
//    return false;
//  }
//
//protected:
//  int _steps_per_sec;
//  
//  absolute_time_t _last_step;
//}; // end of class A4988



// undefine temporary macro variables and functions
#undef A4988_set_pin
#undef A4988_sleep_micros

#endif // A4988_H
