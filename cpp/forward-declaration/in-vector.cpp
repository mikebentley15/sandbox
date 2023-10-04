#include <vector>
#include <utility>

class ForwardDeclared;

struct A {
    A() = default;
    A(std::vector<ForwardDeclared> inVec); // compiles
    // A(std::vector<ForwardDeclared> inVec) : vec{std::move(inVec)}; // doesn't
    std::vector<ForwardDeclared> vec; // this DOES compile
    // std::vector<ForwardDeclared> vec2{}; // this does NOT compile
};

int main() {
    // std::vector<ForwardDeclared> vec; // doesn't compile
    // A a; // doesn't compile
    // A a{}; // coesn't compile
    return 0;
}
