// Author:       Michael Bentley
// Date:         08 June 2021
// Description:  Test a simple string class with a maximum size

#include "StaticString.h"

StaticString<100> str;

void setup() {
  str.extend("hello world");
  Serial.println(str.data());
}

void loop() { }
