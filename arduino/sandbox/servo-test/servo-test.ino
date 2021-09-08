/*
  Arduino Starter Kit example
  Project 5 - Servo Mood Indicator

  This sketch is written to accompany Project 5 in the Arduino Starter Kit

  Parts required:
  - servo motor
  - 10 kilohm potentiometer
  - two 100 uF electrolytic capacitors

  created 13 Sep 2012
  by Scott Fitzgerald

  https://store.arduino.cc/genuino-starter-kit

  This example code is part of the public domain.
*/

// include the Servo library
#include <Servo.h>

Servo myServo;  // create a servo object
const int servo_pin = 5;

void setup() {
  myServo.attach(servo_pin);
}

void loop() {
  for (int i = 0; i < 180; i++) {
    myServo.write(i);
    delay(10);
  }
  for (int i = 180; i > 0; i--) {
    myServo.write(i-1);
    delay(10);
  }
  delay(1000);
}
