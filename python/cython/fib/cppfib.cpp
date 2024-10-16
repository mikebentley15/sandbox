#include "cppfib.hpp"

#include<tuple>
#include<utility>

double cppfib(int n) {
    double a = 0.0;
    double b = 1.0;
    for (int i = 0; i < n; ++i) {
        std::tie(a, b) = std::make_pair(a + b, a);
    }
    return a;
}
