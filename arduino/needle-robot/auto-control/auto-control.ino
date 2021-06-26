#include "MessageParser.h"
#include "MessageSender.h"


//
// global constants
//

const unsigned long BAUD = 9600;


//
// global variables
//

MessageParser parser;
MessageSender sender(Serial);

int32_t linear_abs = 0;
int32_t rotary_abs = 0;
int32_t linear_vel = 0;
int32_t rotary_vel = 0;
int32_t force = 0;

//
// function declarations
//

void send_help();
void send_settings();
void send_state();


//
// function definitions
//

void setup() {
  Serial.begin(BAUD);

  // callbacks for commands to the serial port
  parser.setHelpCallback(send_help);
  parser.setSettingsCallback(send_settings);
  parser.setStateCallback(send_state);

  Serial.println("setup() finished");
}

void loop() {

}

// This is a special function that is automatically called at the end of
// loop(), but only if there is data to read from the serial port.
void serialEvent() {
  while (Serial.available()) {
    parser.append(Serial.read());
  }
}


void send_help() {
  sender.sendHelpCommand("help", "Send descriptions of each supported command");
  sender.sendHelpCommand("settings", "Send each setting value");
  sender.sendHelpCommand("state",
      "Send the current state\r\n"
      " - linear absolute position in micrometers\r\n"
      " - rotary absolute position in milli-degrees\r\n"
      " - linear velocity in micrometers per second\r\n"
      " - rotary velocity in milli-degrees per second\r\n"
      " - force sensor reading in micro-Newtons");
}

void send_settings() {
  sender.sendSetting("baud rate", BAUD);
}

void send_state() {
  sender.sendCurrentState(linear_abs, rotary_abs,
                          linear_vel, rotary_vel,
                          force);
}
