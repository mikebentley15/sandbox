/* 
 * by:   Michael Bentley
 * date: 31 May 2021
 *
 * Notes:
 * - EventLoop class created using SoftTimer as a reference
 *   https://github.com/yehnan/SoftTimer
 *   - Modified to be simpler (only the needed functionality)
 *   - uses microseconds instead of milliseconds
 *   - note: microseconds overflow every ~70 minutes, but it works anyway
 * - HX711: Originally uses the HX711 arduino library by Bogdan Necula and
 *   Andreas Motl.
 *   - This allows for reading every 86ms
 *   - If you ask for two readings in a row, it blocks
 * - HX711_ADC: Intriging.  Keeps a running average for you and uses
 *   non-blocking calls.
 *   - https://github.com/olkal/HX711_ADC
 *   - Docs say the chip can handle either 10 Hz or 80 Hz.  You just need to
 *     set the right voltage to some of the other pins.  (TODO: look into)
 *   - I think I would prefer to do my own running average and rather get the
 *     raw reading from this thing.
 *   - TODO: try out this library instead
 */

#include "EventLoop.h"
#include "StepperMotor.h"

#include <HX711.h>

//
// global constants
//

// defines stepper motor pins
const int LINEAR_STEP_PIN = 2;
const int LINEAR_DIR_PIN  = 3;
const int ROTARY_STEP_PIN = 5;
const int ROTARY_DIR_PIN  = 6;

// defines loadcell pins
const int LOADCELL_DOUT_PIN = 11;
const int LOADCELL_SCK_PIN  = 12;

// found using calibrate-force-sensor sketch
const float calibration_factor = 418110.f;

// how much to step rotary velocity
const float motor_increment = 1.f; // Hz
const int steps_per_rotation = 200;
const uint32_t freq_delta = uint32_t(2 * steps_per_rotation * motor_increment);


//
// global non-const variables
//

HX711 loadcell;
StepperMotor rotary_motor;
StepperMotor linear_motor;
EventLoop<10> eventloop;

RegisteredEventBase *read_force_event = nullptr;
RegisteredEventBase *rotary_motor_event = nullptr;
RegisteredEventBase *linear_motor_event = nullptr;
RegisteredEventBase *read_serial_event = nullptr;


//
// function forward declarations
//

void setup();
void loop();
bool read_force(RegisteredEventBase *event);
bool rotary_motor_half_step(RegisteredEventBase *event);
bool linear_motor_half_step(RegisteredEventBase *event);
bool read_serial(RegisteredEventBase *event);


//
// function definitions
//

void setup() {
  // Setup the serial port and print instructions
  Serial.begin(9600);
  Serial.println("Manual motor control with force readings");
  Serial.println();
  Serial.println("- press 'a' to decrease linear velocity");
  Serial.println("- press 's' to stop linear motor");
  Serial.println("- press 'd' to increase linear velocity");
  Serial.println("- press 'z' to decrease rotary velocity");
  Serial.println("- press 'x' to stop rotary motor");
  Serial.println("- press 'c' to increase linear velocity");
  Serial.println();
  Serial.println("Note: decreasing past zero makes it go in reverse");

  // Setup the stepper motor pins as Outputs
  rotary_motor.setup(ROTARY_STEP_PIN, ROTARY_DIR_PIN);
  linear_motor.setup(LINEAR_STEP_PIN, LINEAR_DIR_PIN);

  // Setup the force sensor
  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  loadcell.set_scale(calibration_factor);
  loadcell.tare(); // blocking call

  // Setup the callback events
  read_force_event = eventloop.schedule(86000, read_force);
  read_serial_event = eventloop.schedule(1, read_serial);

  Serial.println();
  Serial.println("Setup complete");
  Serial.println();
}

void loop() {
  eventloop.update();
}
  //digitalWrite(linear_dir_pin, HIGH); // Enables the motor to move in a particular direction
  //digitalWrite(rotary_dir_pin, HIGH); // Enables the motor to move in a particular direction

  //int microwait = 1000;

  //// Makes 200 pulses for making one full cycle rotation
  //for(int x = 0; x < 400; x++) {
  //  digitalWrite(linear_step_pin, HIGH);
  //  digitalWrite(rotary_step_pin, HIGH);
  //  delayMicroseconds(microwait);
  //  digitalWrite(linear_step_pin, LOW);
  //  digitalWrite(rotary_step_pin, LOW);
  //  delayMicroseconds(microwait);
  //}

  //// Makes 400 pulses for making two full cycle rotation
  //digitalWrite(linear_dir_pin, LOW); // Enables the motor to move in a particular direction
  //digitalWrite(rotary_dir_pin, LOW); // Enables the motor to move in a particular direction
  //for(int x = 0; x < 400; x++) {
  //  digitalWrite(linear_step_pin, HIGH);
  //  digitalWrite(rotary_step_pin, HIGH);
  //  delayMicroseconds(microwait);
  //  digitalWrite(linear_step_pin, LOW);
  //  digitalWrite(rotary_step_pin, LOW);
  //  delayMicroseconds(microwait);
  //}
  //delay(1000);
//}

bool read_force(RegisteredEventBase *event) {
  Serial.print("< force = ");
  Serial.print(loadcell.get_units());
  Serial.println(" >");
  return false;
}

bool rotary_motor_half_step(RegisteredEventBase *event) {
  rotary_motor.toggle_pulse();
  return false;
}

bool linear_motor_half_step(RegisteredEventBase *event) {
  linear_motor.toggle_pulse();
  return false;
}

bool read_serial(RegisteredEventBase *event) {
  uint32_t frequency;

  if (Serial.available()) {
    char from_user = Serial.read();

    if (from_user == 'a') { // move linear faster backward
      if (linear_motor_event == nullptr) { // at 0Hz
        frequency = freq_delta;
        linear_motor_event = eventloop.schedule_frequency(
            frequency, linear_motor_half_step);
        linear_motor.go_backward();
      } else {
        frequency = 1000000 / linear_motor_event->period_micros;
        frequency += (linear_motor.forward ? -freq_delta : freq_delta);
        if (frequency == 0) {
          eventloop.remove_event(linear_motor_event);
          linear_motor_event = nullptr;
        } else {
          linear_motor_event->period_micros = 1000000 / frequency;
        }
      }
      Serial.print("< linear motor velocity = ");
      Serial.print(float(linear_motor.forward ? frequency : -frequency)
                   / (2 * steps_per_rotation));
      Serial.println(" Hz >");

    } else if (from_user == 's') { // stop linear motor
      if (linear_motor_event != nullptr) {
        eventloop.remove_event(linear_motor_event);
        linear_motor_event = nullptr;
      }
      Serial.println("< linear motor velocity = 0 Hz >");

    } else if (from_user == 'd') { // move linear faster forward
      if (linear_motor_event == nullptr) { // at 0Hz
        frequency = freq_delta;
        linear_motor_event = eventloop.schedule_frequency(
            frequency, linear_motor_half_step);
        linear_motor.go_forward();
      } else {
        frequency = 1000000 / linear_motor_event->period_micros;
        frequency += (linear_motor.forward ? freq_delta : -freq_delta);
        if (frequency == 0) {
          eventloop.remove_event(linear_motor_event);
          linear_motor_event = nullptr;
        } else {
          linear_motor_event->period_micros = 1000000 / frequency;
        }
      }
      Serial.print("< linear motor velocity = ");
      Serial.print(float(linear_motor.forward ? frequency : -frequency)
                   / (2 * steps_per_rotation));
      Serial.println(" Hz >");

    } else if (from_user == 'z') { // move rotary faster backward
      if (rotary_motor_event == nullptr) { // at 0Hz
        frequency = freq_delta;
        rotary_motor_event = eventloop.schedule_frequency(
            frequency, rotary_motor_half_step);
        rotary_motor.go_backward();
      } else {
        frequency = 1000000 / rotary_motor_event->period_micros;
        frequency += (rotary_motor.forward ? -freq_delta : freq_delta);
        if (frequency == 0) {
          eventloop.remove_event(rotary_motor_event);
          rotary_motor_event = nullptr;
        } else {
          rotary_motor_event->period_micros = 1000000 / frequency;
        }
      }
      Serial.print("< rotary motor velocity = ");
      Serial.print(float(rotary_motor.forward ? frequency : -frequency)
                   / (2 * steps_per_rotation));
      Serial.println(" Hz >");


    } else if (from_user == 'x') { // stop rotary motor
      if (rotary_motor_event != nullptr) {
        eventloop.remove_event(rotary_motor_event);
        rotary_motor_event = nullptr;
      }
      Serial.println("< rotary motor velocity = 0 Hz >");

    } else if (from_user == 'c') { // move rotary faster forward
      if (rotary_motor_event == nullptr) { // at 0Hz
        frequency = freq_delta;
        rotary_motor_event = eventloop.schedule_frequency(
            frequency, rotary_motor_half_step);
        rotary_motor.go_forward();
      } else {
        frequency = 1000000 / rotary_motor_event->period_micros;
        frequency += (rotary_motor.forward ? freq_delta : -freq_delta);
        if (frequency == 0) {
          eventloop.remove_event(rotary_motor_event);
          rotary_motor_event = nullptr;
        } else {
          rotary_motor_event->period_micros = 1000000 / frequency;
        }
      }
      Serial.print("< rotary motor velocity = ");
      Serial.print(float(rotary_motor.forward ? frequency : -frequency)
                   / (2 * steps_per_rotation));
      Serial.println(" Hz >");

    }
  }
  return false;
}
