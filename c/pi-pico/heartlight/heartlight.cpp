#include <pico/stdlib.h>

namespace {


void init_outpin(const uint pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_OUT);
}

class LED {
public:
  LED(uint pin) : pin_(pin) {
    init_outpin(this->pin_);
  }

  // getters
  bool is_on() const { return this->is_on_; }
  operator bool() const { return this->is_on_; }
  uint pin() const { return this->pin_; }

  // setters
  bool toggle() {
    this->set(!this->is_on_);
    return this->is_on();
  }

  void set(bool on) {
    this->is_on_ = on;
    gpio_put(this->pin_, uint(this->is_on_));
  }

private:
  uint pin_;
  bool is_on_{false};
};

} // unnamed namespace

int main() {
  static constexpr uint led_pin_1 = 1;
  static constexpr uint led_pin_2 = 20;
  static constexpr beat_time_ms = 250;
  LED first{led_pin_1};
  LED second{led_pin_2};
  for (;;) {
    first.set(true);
    second.set(false);
    sleep_ms(beat_time_ms);

    first.set(false);
    second.set(true);
    sleep_ms(beat_time_ms*2);

    first.set(false);
    second.set(false);
    sleep_ms(beat_time_ms);
  }
  return 0;
}
