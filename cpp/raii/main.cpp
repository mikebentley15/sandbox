#include "ScopeCleaner.hpp"

#include <cstdio>

// Prints:
// second
// fourth
// third
// fifth
// first
int main() {
    ScopeCleaner first{[](){ std::puts("first"); }};
    {
        ScopeCleaner second{[](){ std::puts("second"); }};
    }
    {
        ScopeCleaner third{[](){ std::puts("third"); }};
        {
            ScopeCleaner fourth{[](){ std::puts("fourth"); }};
        }
    }
    {
        ScopeCleaner fifth{[](){ std::puts("fifth"); }};
    }
}
