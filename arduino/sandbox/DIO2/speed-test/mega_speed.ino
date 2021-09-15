/*
Speed test of fast digital I/O for Arduino Mega

Un-comment the version ï¿½f digitalWrite you wish to test in the
for loop below.
The program will print time in micro seconds it takes to perform 100-times
digitalWrite().

24.2.2014
*/

// Set to 1 to have fast but (maybe) bigger program (inline digital I/O functions)
// set 0 to have slower and (sometimes) smaller program 
#define GPIO2_PREFER_SPEED 1
#include "DIO2.h"  // include the fast I/O 2 functions

// rollout explicitly
#define REPEAT_TEN_TIMES(x) x;x;x;x;x;x;x;x;x;x;
#define REPEAT_ONE_HUNDRED_TIMES(x) REPEAT_TEN_TIMES(REPEAT_TEN_TIMES(x))
#define REPEAT_ONE_THOUSAND_TIMES(x) REPEAT_ONE_HUNDRED_TIMES(REPEAT_TEN_TIMES(x))

#include "nonconstants.h"
#include "mio.h"

// declared in nonconstants.h
//uint8_t pin = 7;
//uint8_t arduino_led_pin = 13;
//GPIO_pin_t my_pin = DP7;
//GPIO_pin_t led_pin = DP13;

mio::Pin<7> dpin;
mio::Pin_t tpin;

void setup() {
  // put your setup code here, to run once:
  pinMode2(arduino_led_pin, OUTPUT);
  pinMode2(pin, OUTPUT);
  mio::Pin_init(tpin, pin);
  Serial.begin(9600);
  Serial.print("  pin 7 gpio: 0x");
  Serial.print(DP7, HEX);
  Serial.println();
  Serial.print("  pin 7 register address: 0x");
  Serial.print(dpin.register_address_value, HEX);
  Serial.println();
  Serial.print("  pin 7 mask: 0x");
  Serial.print(dpin.mask, HEX);
  Serial.println();
}

static inline void manual_set_7_high()  __attribute__((always_inline, unused));
static inline void manual_set_7_high() {
  GPIO2_OPTIONAL_ATOMIC_BEGIN
  *((volatile uint8_t*)0x102) |= 0x10;
  GPIO2_OPTIONAL_ATOMIC_END
}

void loop() {
  // put your main code here, to run repeatedly: 
  uint32_t start, end;
  //uint8_t cnt;

  // Blink the LED to see if the program is running
  digitalWrite2(13, HIGH );    // version with pin number conversion
  delay(100);
  digitalWrite2(13, LOW );
  delay(100);

  Serial.print("testing: ");
  start = micros();
  REPEAT_ONE_THOUSAND_TIMES(
    // Standard Arduino function
    digitalWrite(pin, HIGH)     // 6.8 us; 3.40 us with timer-pin check disabled 
          
    // New functions
    // Arduino compatible version using pin as a simple integer
    //digitalWrite2(pin, HIGH)    // 1.71 us  (2.41 us with GPIO2_PREFER_SPEED = 0)
    //digitalWrite2(7, HIGH)      // 1.71 us  
          
    // Fast version using pin code as a variable
    //digitalWrite2f(my_pin, HIGH) // 1.40 us (2.30 us with GPIO2_PREFER_SPEED = 0)
    //digitalWrite2f(DP7, HIGH)   // 0.504 us

    //dpin.write_high() // 1.22 us
    //mio::Pin<7>::write_high() // 1.22 us

    //mio::Pin_write_high(tpin) // 1.90 us
    //mio::pin_write_high<7>()  // // 1.27 us

    //manual_set_7_high()
  )
  end = micros();

  Serial.print("1000 digitalWrites took (microseconds): ");
  Serial.print(end - start);
  Serial.println();
  delay(500);
}

