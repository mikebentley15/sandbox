int a[16];
extern int b[16];

int get_a(int i) {
  return a[i];
}

int get_b(int i) {
  return b[i];
}

int set_a(int i, int v) {
  a[i] = v;
}

int set_b(int i, int v) {
  b[i] = v;
}
