#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

struct StepperMotor {
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
};

#endif //STEPPER_MOTOR_H
