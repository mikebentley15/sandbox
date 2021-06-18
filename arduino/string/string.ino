// Author:       Michael Bentley
// Date:         08 June 2021
// Description:  Test a simple string class with a maximum size

#include "StaticString.h"

StaticString<100> str;

template <typename StringGeneratorFunc>
void print_func(StringGeneratorFunc f) {
  Serial.println(f());
}

void setup() {
  str.extend("hello world");
  Serial.println(str.data());
  auto L = []() -> const char* {
    static StaticString<100> s;
    s.extend("hi there");
    return s.data();
  };
}

void loop() {
  delay(1000);
}
