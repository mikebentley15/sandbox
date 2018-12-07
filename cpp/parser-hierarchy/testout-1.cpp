#include <cstdio>
#define max(x, y) (x > y ? x : y)
int main(int argc, char** argv) {
 bool extranewline = true;
 for (int i = 0; i < 3; i++) {
  printf("Hello world! ; ");
 }
 if (extranewline) printf("\n");
 return 0;
}
