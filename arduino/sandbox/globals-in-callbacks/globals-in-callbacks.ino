#include "Caller.h"

int32_t value = 0;
Caller caller;

void callback_func() {
  caller.print(value);
}

void setup() {
  Serial.begin(230400);
  caller.callback = callback_func;
}

void loop() {
  delay(1);
}

void serialEvent() {
  if (Serial.available()) {
    Serial.read();
    caller.call();
  }
}
