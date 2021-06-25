#include "MessageParser.h"

//
// global constants
//

const unsigned long BAUD = 9600;


//
// global variables
//

MessageParser parser;


//
// function declarations
//


//
// function definitions
//

void setup() {
  Serial.begin(BAUD);

  parser.setHelpCallback([]() { Serial.println("<help> called"); });
  parser.setSettingsCallback([]() { Serial.println("<settings> called"); });
  parser.setStateCallback([]() { Serial.println("<state> called"); });

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
