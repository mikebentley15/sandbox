#include <cstdio>

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
      printf("Hello world! ; ");
    }

  // end in a newline
  if (extranewline)
    printf("\n");

  // success!
  return 0;
}
