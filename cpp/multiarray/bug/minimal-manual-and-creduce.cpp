template <class a> void b(a, int c) {
  a(*d)[c + 1][c + 1] = (a(*)[c + 1][c + 1]) d[0];
}
double e;
main() { b(e, 0); }
