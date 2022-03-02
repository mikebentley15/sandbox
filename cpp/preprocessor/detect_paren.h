#ifndef DETECT_PAREN_H
#define DETECT_PAREN_H

#define _CHECK_N(x, n, ...) n
#define _CHECK(...) _CHECK_N(__VA_ARGS__, 0,)
#define _PROBE(x) x, 1

#define IS_PAREN(x) _CHECK(_IS_PAREN_PROBE x)
#define _IS_PAREN_PROBE(...) _PROBE(~)

#endif // DETECT_PAREN_H
