template <class T> void f(T *x, const int n) {
  // Note: if we change this to 
  //     T (*a)[n] = (T(*)[n]) x;  // i.e. 2 dimensional
  //   or change this to
  //     T (*a)[n][n] = reinterpret_cast<T(*)[n][n]>(x);
  //   then the compiler does just fine
  //T (*a)[n][n] = (T(*)[n][n]) x;
  //T (*a)[n][n] = reinterpret_cast<T(*)[n][n]>(x);
  auto &a = reinterpret_cast<T(*&)[n][n]>(x);
  a[0][0][0] = 0;
}

int main() {
  float x[200];
  f(x, 10);
  return 0;
}
