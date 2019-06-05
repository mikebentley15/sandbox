#include <cstdio>

extern "C" {

//
// Successfully stubs, but causes immediate termination and a core dump since
// it throws.
// This is not what we want.
//
//[[ noreturn ]] void exit (int exit_code) {
//  printf("From stubbed exit(%d)\n", exit_code);
//  throw 3;
//}

}
