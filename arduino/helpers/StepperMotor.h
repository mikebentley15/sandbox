#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "mockstd.h"

#include <math.h>

struct StepperMotor {
  const uint32_t millidegrees_per_step = 1800;  // 200 steps per revolution

  int step_pin;        // to send pulses for stepping
  int dir_pin;         // to set the direction (high is forward)
  bool forward;        // if we are set to moving forward
  bool pulse_is_high;  // state of the pulse to step_pin

  StepperMotor() {
    step_pin = -1;
    dir_pin = -1;
    forward = false;
    pulse_is_high = false;
  }

  void setup(int step_pin_id, int dir_pin_id) {
    this->step_pin = step_pin_id;
    this->dir_pin  = dir_pin_id;
    pinMode(this->step_pin, OUTPUT);
    pinMode(this->dir_pin,  OUTPUT);
    digitalWrite(this->step_pin, LOW);
    digitalWrite(this->dir_pin,  HIGH);
    forward = true;
    pulse_is_high = false;
  }

  void toggle_pulse() {
    digitalWrite(step_pin, (pulse_is_high ? LOW : HIGH));
    pulse_is_high = !pulse_is_high;
  }

  void pulse_low() {
    digitalWrite(step_pin, LOW);
    pulse_is_high = false;
  }

  void pulse_high() {
    digitalWrite(step_pin, HIGH);
    pulse_is_high = true;
  }

  void toggle_direction() {
    digitalWrite(dir_pin, (forward ? LOW : HIGH));
    forward = !forward;
  }

  void go_forward() {
    digitalWrite(dir_pin, HIGH);
    forward = true;
  }

  void go_backward() {
    digitalWrite(dir_pin, LOW);
    forward = false;
  }

  /** calculate the microseconds per pulse for a velocity
   *
   * @param vel: angular velocity in milli-degrees per second
   * @return closest integer microsecond interval for each pulse to achieve the
   *     given velocity.
   */
  static constexpr uint32_t micros_per_pulse(uint32_t vel) {
    // 1.8e9 = (1e6 us/s) * (1e3 m-deg/deg) * (360 deg/rev) / (200 pulses/rev)
    return uint32_t(1800000000) / vel;
  }

  /** calculate the next pulse interval for an acceleration
   *
   * assumes it is called immediately after calling a pulse with interval
   * micros_per_pulse(vel).
   *
   * performs a square root using double precision, so it's not the most
   * efficient thing...
   *
   * @param vel: current velocity in millidegrees per second
   * @param acc: desired acceleration in millidegrees per second squared
   * @return pulse interval: microseconds until the next pulse (assuming this
   *     is called right after a pulse is sent)
   * @return new velocity: the new velocity at the next pulse in millidegrees
   *     per second.
   */
  static pair<uint32_t, uint32_t> next_pulse(uint32_t vel, uint32_t acc) {
    double dvel(vel);
    double dacc(acc);
    double t = (- dvel + sqrt(dvel * dvel + 4 * dacc * 1.8e9)) / (2 * dacc);
    uint32_t new_vel = uint32_t(dacc * t);
    return pair<uint32_t, uint32_t>(uint32_t(t), new_vel);
  }
};

#endif //STEPPER_MOTOR_H
