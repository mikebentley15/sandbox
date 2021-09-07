#include "pico/stdlib.h"

#include <stdio.h>

#define PIN 25

#define RUN_TEN_TIMES(x) x;x;x;x;x;x;x;x;x;x;
#define RUN_ONE_HUNDRED_TIMES(x) RUN_TEN_TIMES(RUN_TEN_TIMES(x))
#define RUN_TEN_THOUSAND_TIMES(x) RUN_ONE_HUNDRED_TIMES(RUN_ONE_HUNDRED_TIMES(x))

int main() {
  // initialize I/O
  stdio_init_all();

  // initialize gpio
  gpio_init(PIN);
  gpio_set_dir(PIN, GPIO_OUT);

  int count = 1000000;

  while (true) {
    // about 0.22 us per call
    uint32_t start = time_us_32();
    RUN_TEN_THOUSAND_TIMES(gpio_put(PIN, 1));
    uint32_t timing_high = time_us_32() - start;
    sleep_ms(200);

    // about 0.22 us per call
    start = time_us_32();
    RUN_TEN_THOUSAND_TIMES(gpio_put(PIN, 0));
    uint32_t timing_low = time_us_32() - start;
    sleep_ms(200);

    // about 0.22 us per call
    bool is_high = false;
    start = time_us_32();
    for (int i = 0; i < 500; ++i) {
      RUN_TEN_THOUSAND_TIMES(gpio_put(PIN, (is_high = !is_high ? 0 : 1)));
    }
    uint32_t timing_toggle = time_us_32() - start;
    sleep_ms(200);

    printf("Time for\n"
           "- 10,000 sets to high: %d us\n"
           "- 10,000 sets to low:  %d us\n"
           "- 500,000 toggles:     %d us\n",
           timing_high, timing_low, timing_toggle);

    sleep_ms(500);
  }

  return 0;
}
