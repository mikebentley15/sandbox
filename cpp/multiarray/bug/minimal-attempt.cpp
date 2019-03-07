// This code compiles fine under
//   g++ -O0 <filename>
// but when compiled under any of these:
//   g++ -O1 <filename>
//   g++ -O2 <filename>
//   g++ -O3 <filename>
// it gives a segmentation fault.
//
// This is the case for GCC versions 7 and up.  I was testing and minimizing
// the source code for GCC version 8.2.1 (from 20181127)

template <class T> void f (T *x, int n) {
  // All of these variants cause the problem:
  //
  //   T (*a)[n][n] = (T(*)[n][n]) x;
  //   T (*a)[n][n] = reinterpret_cast<T(*)[n][n]>(x);
  //   auto a = (T(*)[n][n])x;
  //   auto a = reinterpret_cast<T(*)[n][n]>(x);
  //
  // The problem goes away if we set the incoming integer n as const
  //
  // Of the four different types of declarations, the only ones that
  // successfully compile without error in Clang are those with the auto
  // declaration.
  auto a = (T(*)[n][n]) x;
  a[0][0][0] = 0;
}

int main() {
  float x[200];
  f(x, 10);
  return 0;
}
