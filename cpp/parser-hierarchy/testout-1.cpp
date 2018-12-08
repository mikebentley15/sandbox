#include < cstdio >
extern"C++" {
#ifndef __MAC__
#define max(x, y) (x > y ? x : y)
#endif
 int main(int argc, char ** argv) {
  ;
  bool extranewline = true;
  for(int i = 0; i < 3; i ++) {
   if(i % 2 == 0) printf("Hello world! ; ");
  }
  char ch1 = '\x7f';
  char ch2 = L'\x7f';
  char ch3 = '\'';
  char * s1 = "";
  char * s2 = "\"";
  char * s3 = "\\\"\"\"\\";
  if(extranewline) printf("\n");
  return 0;
 }
}
