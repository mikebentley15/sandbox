#ifndef FOO_H
#define FOO_H

namespace {
inline int unnamed_namespace_inline_hfunc() { return 13; }
int unnamed_namespace_hfunc() { return 14; }
}

static inline int static_inline_hfunc() { return 15; }
static int static_hfunc() { return 16; }
inline int inline_hfunc() { return 17; }
int func();


#endif // FOO_H
