#include <cstdlib>
#include <cstdio>

void func() {
  exit(2);
}

void my_at_exit() {
  printf("called my_at_exit()\n");
  // causes immediate termination with a core dump
  //throw 3.14;
}

int main(void) {
  std::atexit(my_at_exit);
  try {
    func();
  } catch (double d) {
    printf("Caught a double: %lf\n", d);
  }
  return 0;
}
