#ifndef FOO_H
#define FOO_H

namespace {
inline int unnamed_namespace_inline_hfunc() { return 8; }
int unnamed_namespace_hfunc() { return 9; }
}

static inline int static_inline_hfunc() { return 10; }
static int static_hfunc() { return 11; }
inline int inline_hfunc() { return 12; }
int func();


#endif // FOO_H
