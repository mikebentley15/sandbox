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

#include <HX711.h>

#ifndef UNUSED_VAR
#define UNUSED_VAR(x) (void)x
#endif


using Evt = RegisteredEventBase;


//
// global constants
//

const unsigned long BAUD = 9600;  // bits per second
//const unsigned long BAUD = 115200;
//const unsigned long BAUD = 230400;
//const unsigned long BAUD = 500000;

const uint32_t linear_pitch = 5000; // micrometers


//
// global variables
//

MessageParser parser;
MessageSender sender(Serial);
EventLoop<10> eventloop;

int32_t linear_abs = 0; // micrometers
int32_t rotary_abs = 0; // milli-degrees
int32_t linear_vel = 0; // micrometers / sec
int32_t rotary_vel = 0; // milli-degrees / sec
int32_t force = 0;      // micro-Newtons

bool stream_force = false;
uint32_t force_read_interval = 90000; // microseconds between, ~11 Hz

//
// function declarations
//



//
// function definitions
//

void setup() {
  Serial.begin(BAUD);

  //
  // callbacks for commands to the serial port
  //

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
        " @param on|off: turn binary output on or off.");
    sender.sendHelpCommand("stream-force",
        "Set the auto-streaming of force measurements\r\n"
        " @param on|off: turn streaming on or off.");
  });

  parser.setSettingsCallback([]() {
    sender.sendSetting("baud rate", BAUD);
    sender.sendSetting("binary output", sender.is_binary());
    sender.sendSetting("linear pitch", linear_pitch);
    sender.sendSetting("stream force", stream_force);
    sender.sendSetting("force sensor interval", force_read_interval);
  });

  parser.setStateCallback([]() {
    sender.sendCurrentState(linear_abs, rotary_abs,
                            linear_vel, rotary_vel,
                            force);
  });

  parser.setSendBinaryCallback([](bool on) {
    sender.set_binary(on);
  });

  parser.setStreamForceCallback([](bool on) {
    stream_force = on;
  });


  //
  // timed events
  //

  // read force
  eventloop.schedule(force_read_interval, [](Evt*) -> bool {
    // TODO: read the force sensor
    if (stream_force) {
      sender.sendForce(force);
    }
    return false;
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
    Serial.println(val);
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

