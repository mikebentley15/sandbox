#include <cstdio>

extern "C++" {
#ifndef __MAC__
#define max(x, y) \
  (x > y ? \
   x : y)
#endif // end of #ifndef __MAC__

// main function
int
main(
					int argc,
					char** argv
)
{
  bool extranewline = true;
  // print three hello worlds in one line
  for (int i = 0; i < 3; i++)
    {
      if (i % 2 == 0)
        printf("Hello world! ; ");
    }

  // test that our parser can handle some weird characters
  char ch1 = '\x7f';
  char ch2=
    L'\x7f'
    ;
  char ch3 = '\'';

  // test that our parser can handle some weird strings
  char* s1 = "";
  char* s2 = "\"";
  char* s3 = "\\\"\"\"\\";

  // end in a newline
  if (extranewline)
    printf("\n");

  // success!
  return 0;
}

} // end of extern "C++"
