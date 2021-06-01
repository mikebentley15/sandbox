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
const uint32_t motor_increment = 1; // Hz
const uint32_t steps_per_rotation = 200;


//
// global non-const variables
//

HX711 loadcell;
StepperMotor rotary_motor;
StepperMotor linear_motor;
EventLoop<10> eventloop;

RegisteredEventBase *rotary_motor_event = nullptr;
RegisteredEventBase *linear_motor_event = nullptr;
//RegisteredEventBase *read_force_event = nullptr;
//RegisteredEventBase *read_serial_event = nullptr;

int32_t linear_velocity = 0; // Hz (rotations per second)
int32_t rotary_velocity = 0; // Hz (rotations per second)

uint32_t force_events  = 0;
uint32_t rotary_events = 0;
uint32_t linear_events = 0;
uint32_t event_status_period = 1000000;


//
// function forward declarations
//

void setup();
void loop();
bool event_status(RegisteredEventBase *event);
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
  eventloop.schedule(86000, read_force);
  eventloop.schedule(1, read_serial);
  eventloop.schedule(event_status_period, event_status);

  Serial.println();
  Serial.println("Setup complete");
  Serial.println();
  Serial.println("force-events,rotary-events,linear-events");
}

void loop() {
  eventloop.update();
}

bool event_status(RegisteredEventBase *event) {
  uint32_t force_evt_freq  = force_events  * 1000000 / event_status_period;
  uint32_t rotary_evt_freq = rotary_events * 1000000 / event_status_period;
  uint32_t linear_evt_freq = linear_events * 1000000 / event_status_period;
  // reset the event counts
  force_events  = 0;
  rotary_events = 0;
  linear_events = 0;
  // print the event frequencies
  Serial.print(force_evt_freq);
  Serial.print("\t");
  Serial.print(rotary_evt_freq);
  Serial.print("\t");
  Serial.print(linear_evt_freq);
  Serial.println();
  return false;
}

bool read_force(RegisteredEventBase *event) {
  force_events++;
  //Serial.print("< force = ");
  //Serial.print(loadcell.get_units() * 1000);
  //Serial.println(" g >");
  return false;
}

bool rotary_motor_half_step(RegisteredEventBase *event) {
  rotary_events++;
  rotary_motor.toggle_pulse();
  return false;
}

bool linear_motor_half_step(RegisteredEventBase *event) {
  linear_events++;
  linear_motor.toggle_pulse();
  return false;
}

bool read_serial(RegisteredEventBase *event) {
  enum UpdateType {
    UT_NONE,
    UT_LINEAR,
    UT_ROTARY,
  };

  UpdateType utype = UT_NONE;
  if (Serial.available()) {
    char from_user = Serial.read();

    switch (from_user) {
      case 'a': utype = UT_LINEAR; linear_velocity -= motor_increment; break;
      case 's': utype = UT_LINEAR; linear_velocity  = 0;               break;
      case 'd': utype = UT_LINEAR; linear_velocity += motor_increment; break;
      case 'z': utype = UT_ROTARY; rotary_velocity -= motor_increment; break;
      case 'x': utype = UT_ROTARY; rotary_velocity  = 0;               break;
      case 'c': utype = UT_ROTARY; rotary_velocity += motor_increment; break;
    }

    uint32_t call_freq;
    if (utype == UT_LINEAR) {
      uint32_t call_freq = abs(linear_velocity) * 2 * steps_per_rotation;
      Serial.print("< linear motor velocity = ");
      Serial.print(linear_velocity);
      Serial.println(" Hz >");
      if (linear_motor_event == nullptr) {
        if (linear_velocity != 0) {
          linear_motor_event = eventloop.schedule_frequency(
              call_freq, linear_motor_half_step);
        }
      } else if (linear_velocity == 0) {
        eventloop.remove_event(linear_motor_event);
        linear_motor_event = nullptr;
      } else {
        linear_motor_event->period_micros = 1000000 / call_freq;
        if (linear_velocity < 0) {
          linear_motor.go_backward();
        } else {
          linear_motor.go_forward();
        }
      }
    }

    if (utype == UT_ROTARY) {
      uint32_t call_freq = abs(rotary_velocity) * 2 * steps_per_rotation;
      Serial.print("< rotary motor velocity = ");
      Serial.print(rotary_velocity);
      Serial.println(" Hz >");
      if (rotary_motor_event == nullptr) {
        if (rotary_velocity != 0) {
          rotary_motor_event = eventloop.schedule_frequency(
              call_freq, rotary_motor_half_step);
        }
      } else if (rotary_velocity == 0) {
        eventloop.remove_event(rotary_motor_event);
        rotary_motor_event = nullptr;
      } else {
        rotary_motor_event->period_micros = 1000000 / call_freq;
        if (rotary_velocity < 0) {
          rotary_motor.go_backward();
        } else {
          rotary_motor.go_forward();
        }
      }
    }
  }

  return false;
}
