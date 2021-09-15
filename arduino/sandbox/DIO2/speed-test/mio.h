#ifndef MIO_H_
#define MIO_H_

#include "DIO2.h"

namespace mio {

extern "C" {

constexpr inline GPIO_pin_t arduino_to_gpio_pin(uint8_t pin) {
  return
      pin ==  0 ? DP0  :
      pin ==  1 ? DP1  :
      pin ==  2 ? DP2  :
      pin ==  3 ? DP3  :
      pin ==  4 ? DP4  :
      pin ==  5 ? DP5  :
      pin ==  6 ? DP6  :
      pin ==  7 ? DP7  :
      pin ==  8 ? DP8  :
      pin ==  9 ? DP9  :
      pin == 10 ? DP10 :
      pin == 11 ? DP11 :
      pin == 12 ? DP12 :
      pin == 13 ? DP13 :
      pin == 14 ? DP14 :
      pin == 15 ? DP15 :
      pin == 16 ? DP16 :
      pin == 17 ? DP17 :
      pin == 18 ? DP18 :
      pin == 19 ? DP19 :
      pin == 20 ? DP20 :
      pin == 21 ? DP21 :
      pin == 22 ? DP22 :
      pin == 23 ? DP23 :
      pin == 24 ? DP24 :
      pin == 25 ? DP25 :
      pin == 26 ? DP26 :
      pin == 27 ? DP27 :
      pin == 28 ? DP28 :
      pin == 29 ? DP29 :
      pin == 30 ? DP30 :
      pin == 31 ? DP31 :
      pin == 32 ? DP32 :
      pin == 33 ? DP33 :
      pin == 34 ? DP34 :
      pin == 35 ? DP35 :
      pin == 36 ? DP36 :
      pin == 37 ? DP37 :
      pin == 38 ? DP38 :
      pin == 39 ? DP39 :
      pin == 40 ? DP40 :
      pin == 41 ? DP41 :
      pin == 42 ? DP42 :
      pin == 43 ? DP43 :
      pin == 44 ? DP44 :
      pin == 45 ? DP45 :
      pin == 46 ? DP46 :
      pin == 47 ? DP47 :
      pin == 48 ? DP48 :
      pin == 49 ? DP49 :
      pin == 50 ? DP50 :
      pin == 51 ? DP51 :
      pin == 52 ? DP52 :
      pin == 53 ? DP53 :
      pin == 54 ? DP54 :
      pin == 55 ? DP55 :
      pin == 56 ? DP56 :
      pin == 57 ? DP57 :
      pin == 58 ? DP58 :
      pin == 59 ? DP59 :
      pin == 60 ? DP60 :
      pin == 61 ? DP61 :
      pin == 62 ? DP62 :
      pin == 63 ? DP63 :
      pin == 64 ? DP64 :
      pin == 65 ? DP65 :
      pin == 66 ? DP66 :
      pin == 67 ? DP67 :
      pin == 68 ? DP68 :
      pin == 69 ? DP69 :
                  DP_INVALID;
}

constexpr inline uint16_t get_register_address(GPIO_pin_t gpin) {
  return ((gpin & 0x0080) == 0) ?
         (gpin & 0x00FF) : \
			   (gpin & 0x007F) | 0x0100;
}

} // end of extern "C"

template <uint8_t num>
struct Pin {

  //
  // compile-time constant values
  //

  //static constexpr uint8_t id = num;
  //static constexpr auto gpio = arduino_to_gpio_pin(num); //gpio_pins_progmem[num];
  //static constexpr uint8_t register_address_value = get_register_address(gpio);
  static constexpr uint16_t register_address_value =
      get_register_address(arduino_to_gpio_pin(num));
  static constexpr uint8_t mask = GPIO_PIN_MASK(arduino_to_gpio_pin(num));
  static constexpr uint8_t neg_mask = ~mask;

  //
  // static functions
  //

  static volatile uint8_t* register_address() {
    return reinterpret_cast<uint8_t*>(register_address_value);
  }

  static volatile uint8_t& register_reference() {
    return *register_address();
  }

  static void write_low() {
    GPIO2_OPTIONAL_ATOMIC_BEGIN
    *register_address() &= neg_mask;
    GPIO2_OPTIONAL_ATOMIC_END
  }

  static void write_high() {
    GPIO2_OPTIONAL_ATOMIC_BEGIN
    *register_address() |= mask;
    GPIO2_OPTIONAL_ATOMIC_END
  }

  static void write(uint8_t value) {
    if (value == 0) {
      write_low();
    } else {
      write_high();
    }
  }
};

extern "C" {

struct Pin_t {
  volatile uint8_t* register_address;
  uint8_t mask;
};

inline void Pin_init(Pin_t &p, uint8_t num) {
  auto gpio = gpio_pins_progmem[num];
  p.register_address = GET_PORT_REG_ADR(gpio);
  p.mask = GPIO_PIN_MASK(gpio);
}

inline void Pin_write_high(Pin_t &p) __attribute__((always_inline, unused));
inline void Pin_write_high(Pin_t &p) {
  GPIO2_OPTIONAL_ATOMIC_BEGIN
  *p.register_address |= p.mask;
  GPIO2_OPTIONAL_ATOMIC_END
}

} // end of extern "C"

template <uint8_t pin>
inline void pin_write_high()  __attribute__((always_inline, unused));
template <uint8_t pin>
inline void pin_write_high() {
  static constexpr auto gpio = arduino_to_gpio_pin(pin);
  static constexpr uint16_t register_address_value = get_register_address(gpio);
  static constexpr uint8_t mask = GPIO_PIN_MASK(gpio);

  GPIO2_OPTIONAL_ATOMIC_BEGIN
  *((uint8_t*)register_address_value) |= mask;
  GPIO2_OPTIONAL_ATOMIC_END
}

} // end of namespace mio

#endif // MIO_H_
