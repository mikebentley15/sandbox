#include <cstdio>

int main(int argCount, char* argList[]) {
    if (argCount == 0) {
        std::puts("Zero arguments received"); // LCOV_EXCL_LINE
    } else if (argCount <= 1) {
        std::puts("One argument received");
    } else {
        std::puts("More than one argument received");
    }
    return 0;
}
