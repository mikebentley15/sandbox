#ifndef serial_assert_h
#define serial_assert_h

#include <stdlib.h>

// uncomment this to turn off asserts
//#define NDEBUG

#ifdef NDEBUG
# define serial_assert(expression, message)
#else
# define serial_assert(expression, message)              \
  if (!(expression)) {                                   \
    Serial.println("Error: assert failed " #expression); \
    Serial.print("  ");                                  \
    Serial.println(message);                             \
    Serial.flush();                                      \
    abort();                                             \
  }
#endif


#endif // serial_assert_h
