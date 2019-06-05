#include <cstdlib>
#define exit exit_stub

extern "C" {

void exit_stub (int exit_code);

}
