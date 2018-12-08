// This is the next one that caused problems.
// Why in the world would you put a macro within an enum?
// And what are we supposed to do about it?

namespace Architecture
{
  enum Type {
    Generic = 0x0,
    SSE = 0x1,
    AltiVec = 0x2,
    VSX = 0x3,
    NEON = 0x4,

    Target = SSE
#define hello
  };
}

