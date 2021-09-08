#ifndef CALLER_H
#define CALLER_H

using Callback = void();

struct Caller {
  Callback *callback = nullptr;
  void call();
  void print(int32_t value) {
    Serial.print(value);
  }
};

#endif // CALLER_H
