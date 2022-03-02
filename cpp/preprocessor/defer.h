#ifndef DEFER_H
#define DEFER_H

#define _EMPTY(...)
#define DEFER(...) __VA_ARGS__ _EMPTY()
#define DEFER_N(N, ...) DEFER ## N(__VA_ARGS__)
#define DEFER1(...) DEFER(__VA_ARGS__)
#define DEFER2(...) __VA_ARGS__ DEFER1(_EMPTY) ()
#define DEFER3(...) __VA_ARGS__ DEFER2(_EMPTY) ()
#define DEFER4(...) __VA_ARGS__ DEFER3(_EMPTY) ()
#define DEFER5(...) __VA_ARGS__ DEFER4(_EMPTY) ()

#endif // DEFER_H
