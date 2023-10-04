#include <memory>

class ForwardDeclared;

struct A {
    A() = default;

    A(std::shared_ptr<ForwardDeclared> f) : ptr(std::move(f)) {}
    // A(ForwardDeclared *f) : ptr{f} {} // doesn't compile

    std::shared_ptr<ForwardDeclared> ptr{};
};

int main() {
    ForwardDeclared *f = nullptr;
    std::shared_ptr<ForwardDeclared> ptr1;
    std::shared_ptr<ForwardDeclared> ptr2{};
    std::shared_ptr<ForwardDeclared> ptr3{nullptr};
    // std::shared_ptr<ForwardDeclared> ptr4{f}; // doesn't compile
    A a0{};
    A a1{ptr1};
    A a2{ptr2};
    A a3{ptr3};
    A a4{nullptr};
    return 0;
}
