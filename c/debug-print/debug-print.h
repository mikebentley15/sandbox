#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

#include <stdio.h>
#include <stdarg.h>

#ifndef mdebug
# ifdef DEBUG
#  define mdebug(...) printf(__VA_ARGS__)
# else // ndef DEBUG
#  define mdebug(...)
# endif // ndef DEBUG
#endif // def mdebug

static inline
void debug(const char* format, ...) {
#ifdef DEBUG
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);
#endif
}

#endif // DEBUG_PRINT_H
