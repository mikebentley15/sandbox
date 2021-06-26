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
  });

  parser.setSettingsCallback([]() {
    sender.sendSetting("baud rate", BAUD);
    sender.sendSetting("binary output", sender.is_binary());
  });

  parser.setStateCallback([]() {
    sender.sendCurrentState(linear_abs, rotary_abs,
                            linear_vel, rotary_vel,
                            force);
  });

  parser.setSendBinaryCallback([](bool turn_on) {
    sender.set_binary(turn_on);
  });


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

}

// This is a special function that is automatically called at the end of
// loop(), but only if there is data to read from the serial port.
void serialEvent() {
  while (Serial.available()) {
    parser.append(Serial.read());
  }
}

