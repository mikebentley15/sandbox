/* 
 * by:   Michael Bentley
 * date: 25 June 2021
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

#include "MessageParser.h"
#include "MessageSender.h"
#include "EventLoop.h"
#include "StepperMotor.h"

// ignore warnings when include HX711.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wexpansion-to-defined"
#include <HX711.h>
#pragma GCC diagnostic pop

#ifndef UNUSED_VAR
#define UNUSED_VAR(x) (void)x
#endif


template <typename T>
struct Ratio {
  T a; // numerator
  T b; // denominator
  constexpr operator T() { return T(a / b); }
  constexpr Ratio operator*(T val) { return Ratio{a * val, b}; }
  constexpr Ratio operator/(T val) { return Ratio{a, b * val}; }
  constexpr Ratio operator*(const Ratio &r) { return Ratio{a * r.a, b * r.b}; }
  constexpr Ratio operator/(const Ratio &r) { return Ratio{a * r.b, b * r.a}; }
};


//
// global constants
//

//const unsigned long BAUD = 9600;  // bits per second
//const unsigned long BAUD = 115200;
const unsigned long BAUD = 230400;
//const unsigned long BAUD = 500000;

const uint32_t linear_pitch = 5000; // micrometers / rev
const uint32_t steps_per_revolution = 200; // steps / rev
const uint32_t microns_per_step = linear_pitch / steps_per_revolution;
const uint32_t angle_per_step = 360000 / steps_per_revolution;
const uint32_t linear_to_angular = angle_per_step / microns_per_step;

const uint16_t max_events = 10;
const uint32_t force_read_interval = 100000; // ~10 Hz

// defines stepper motor pins
const int LINEAR_STEP_PIN = 2;
const int LINEAR_DIR_PIN  = 3;
const int ROTARY_STEP_PIN = 5;
const int ROTARY_DIR_PIN  = 6;

// defines loadcell pins
const int LOADCELL_DOUT_PIN = 11;
const int LOADCELL_SCK_PIN  = 12;

// found using calibrate-force-sensor sketch
//const float calibration_factor = 418110.f;  // 1 / grams
const Ratio<int32_t> calibration_ratio {5973, 140}; // 1 / micro newtons (uN)
const float calibration_factor = float(calibration_ratio.a) / calibration_ratio.b;

const int32_t max_angular_velocity     = 5 * 360000; // 5 Hz
const int32_t max_angular_acceleration = 2 * 360000; // 2 Hz / sec


//
// global variables
//

HX711 loadcell;
StepperMotor rotary_motor;
StepperMotor linear_motor;

MessageParser parser;
MessageSender sender(Serial);
EventLoop<max_events> eventloop;

Event stream_state_event;
Event force_read_event;
Event rotary_motor_event;
Event linear_motor_event;

// current state
int32_t linear_abs = 0; // micrometers
int32_t rotary_abs = 0; // milli-degrees
int32_t linear_vel = 0; // micrometers / sec
int32_t rotary_vel = 0; // milli-degrees / sec
int32_t raw_force  = 0; // raw force sensor value
int32_t force = 0;      // micro-Newtons

// desired state
int32_t des_linear_abs = 0; // micrometers
int32_t des_rotary_abs = 0; // milli-degrees
int32_t des_linear_vel = 0; // micrometers / sec
int32_t des_rotary_vel = 0; // milli-degrees / sec

bool stream_force = false;

//
// helper functions
//

namespace {

void send_state() {
  sender.sendCurrentState(linear_abs, rotary_abs,
                          linear_vel, rotary_vel,
                          force);
}

} // end of unnamed namespace


//
// function definitions
//

void setup() {

  //
  // hardware setup
  //

  Serial.begin(BAUD);

  // Setup the force sensor
  Serial.print("Taring force sensor... ");
  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  loadcell.set_scale(calibration_factor);
  loadcell.tare(10); // number of times to average.  blocking call
  Serial.println("done");

  // Setup the stepper motor pins as outputs
  rotary_motor.setup(ROTARY_STEP_PIN, ROTARY_DIR_PIN);
  linear_motor.setup(LINEAR_STEP_PIN, LINEAR_DIR_PIN);

  //
  // timed events
  //

  // setup events for when we want to activate them

  stream_state_event.callback = [](Event*) { send_state(); };

  force_read_event.callback = [](Event*) {
    if (loadcell.is_ready()) {
      raw_force = loadcell.read();
      // floating-point computation
      //force = uint32_t((raw_force - loadcell.get_offset())
      //                 / loadcell.get_scale());
      // fixed-point computation rather than floating-point
      force = ((raw_force - loadcell.get_offset()) * calibration_ratio.b)
              / calibration_ratio.a;
      if (stream_force) {
        sender.sendForce(force);
      }
    }
  };

  rotary_motor_event.callback = [](Event*) {
    rotary_motor.toggle_pulse();

    // update the state
    if (!rotary_motor.pulse_is_high()) { // then it moved
      if (rotary_motor.forward) {
        rotary_abs += angle_per_step;
      } else {
        rotary_abs -= angle_per_step;
      }
    }
  };

  linear_motor_event.callback = [](Event*) {
    linear_motor.toggle_pulse();

    // update the state
    if (!linear_motor.pulse_is_high()) { // then it moved
      if (linear_motor.forward) {
        linear_abs += microns_per_step;
      } else {
        linear_abs -= microns_per_step;
      }
    }
  };

  // register events that are to always go
  force_read_event.period_micros = force_read_interval;
  force_read_event.last_trigger = micros(); // now
  eventloop.add_event(&force_read_event);


  //
  // callbacks for commands to the serial port
  //

  // user sends <help>
  parser.setHelpCallback([]() {
    sender.sendHelpCommand("help", "Send descriptions of each supported command");
    sender.sendHelpCommand("settings", "Send each setting value");
    sender.sendHelpCommand("state",
        "Send the current state\r\n"
        " - linear absolute position in micrometers\r\n"
        " - rotary absolute position in milli-degrees\r\n"
        " - linear velocity in micrometers per second\r\n"
        " - rotary velocity in milli-degrees per second\r\n"
        " - force sensor reading in micro-Newtons");
    sender.sendHelpCommand("send-binary", "Set binary to on or off\r\n"
        " @param on|off: turn binary output on or off");
    sender.sendHelpCommand("stream-force",
        "Set the auto-streaming of force measurements\r\n"
        " @param on|off: turn streaming on or off");
    sender.sendHelpCommand("stream-state-on",
        "Turn on the auto-streaming of states\r\n"
        " @param interval (32-bit unsigned): how often to print the state");
    sender.sendHelpCommand("stream-state-off",
        "Turn off the auto-streaming of states");
    sender.sendHelpCommand("tare",
        "Tare the loadcell force sensor to zero.  This blocks everything\r\n"
        " (including motors and force readings) for about 500 ms.  At the\r\n"
        " beginning of tare, a <tare-starting> will be sent.  At the end\r\n"
        " of tare, a <tare-finished> will be sent.");
  });

  // user sends <settings>
  parser.setSettingsCallback([]() {
    sender.sendSetting("baud rate", BAUD);
    sender.sendSetting("binary output", sender.is_binary());
    sender.sendSetting("linear pitch", linear_pitch);
    sender.sendSetting("stream force", stream_force);
    sender.sendSetting("force sensor interval", force_read_event.period_micros);
    sender.sendSetting("linear step pin", LINEAR_STEP_PIN);
    sender.sendSetting("linear dir pin", LINEAR_DIR_PIN);
    sender.sendSetting("rotary step pin", ROTARY_STEP_PIN);
    sender.sendSetting("rotary dir pin", ROTARY_DIR_PIN);
    sender.sendSetting("loadcell dout pin", LOADCELL_DOUT_PIN);
    sender.sendSetting("loadcell sck pin", LOADCELL_SCK_PIN);
    sender.sendSetting("loadcell calibration factor", calibration_factor);
    sender.sendSetting("max angular velocity", max_angular_velocity);
    sender.sendSetting("max angular acceleration", max_angular_acceleration);

    bool is_streaming_state = eventloop.contains(&stream_state_event);
    sender.sendSetting("stream state", is_streaming_state);
    if (is_streaming_state) {
      sender.sendSetting("stream state interval",
                         stream_state_event.period_micros);
    }
  });

  // user sends <state>
  parser.setStateCallback(send_state);

  // user sends <send-binary/[on|off]>
  parser.setSendBinaryCallback([](bool on) {
    sender.set_binary(on);
  });

  // user sends <stream-force/[on|off]>
  parser.setStreamForceCallback([](bool on) {
    stream_force = on;
  });

  // user sends <stream-state-on/{interval}>
  parser.setStreamStateOnCallback([](uint32_t interval) {
    stream_state_event.period_micros = interval;
    if (!eventloop.contains(&stream_state_event)) {
      stream_state_event.last_trigger = micros(); // now
      eventloop.add_event(&stream_state_event);
    }
  });

  // user sends <stream-state-off>
  parser.setStreamStateOffCallback([]() {
    eventloop.remove_event(&stream_state_event);
  });

  // user sends <tare>
  parser.setTareCallback([]() {
    sender.sendTareStarting();
    loadcell.tare(5);
    sender.sendTareFinished();
    eventloop.reset();
  });


  //
  // temporary debug values
  //

  // temporary debug values for the five state variables
  auto set_byte_vals = [](int32_t &val, uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    val = (int32_t(a) << 24)
        + (int32_t(b) << 16)
        + (int32_t(c) <<  8)
        +  int32_t(d);
  };

  set_byte_vals(linear_abs, ' ', 'H', 'e', 'l');
  set_byte_vals(rotary_abs, 'l', 'o', ' ', 'W');
  set_byte_vals(linear_vel, 'o', 'r', 'l', 'd');
  set_byte_vals(rotary_vel, ',', ' ', 'f', 'r');
  set_byte_vals(force,      'i', 'e', 'n', 'd');

  Serial.println("setup() finished");
}

void loop() {
  eventloop.update();
}

// This is a special function that is automatically called at the end of
// loop(), but only if there is data to read from the serial port.
void serialEvent() {
  while (Serial.available()) {
    parser.append(Serial.read());
  }
}

