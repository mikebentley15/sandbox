// input that caused it to fail, not yet sure why

extern __inline __m512d
__attribute__ ((__gnu_inline__, __always_inline__, __artificial__))
_mm512_set1_pd (double __A)
{
  return (__m512d) __builtin_ia32_broadcastsd512 (__extension__
        (__v2df) { __A, },
        (__v8df)
        _mm512_undefined_pd (),
        (__mmask8) -1);
}


