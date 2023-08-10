#include <iostream>
#include <tuple>
#include <vector>
#include <ios>

template <typename T>
concept HasNodeType = requires { typename T::Node; };

template <typename T, typename Node>
concept CanProcessNodeA = requires (T &&processor, Node &&node) {
    { processor.process(node) } -> std::same_as<std::tuple<bool, Node, std::vector<Node>>>;
};

template <typename T>
concept CanProcessNodeB = requires(T &&processor, typename T::Node &&node) {
    { processor.process(node) } -> std::same_as<std::tuple<bool, typename T::Node, std::vector<typename T::Node>>>;
};

template <typename T>
concept NodeProcessorA = CanProcessNodeA<T, typename T::Node>;

template <typename T>
concept NodeProcessorB = HasNodeType<T> && CanProcessNodeA<T, typename T::Node>;

struct A {
    struct Node { int a; };
};

struct B {
    using Node = A::Node;
};

struct C {
    struct Node {};
    void process(Node&);
};

struct D {
    void process();
};

struct E {
    struct Node {};
    std::tuple<bool, Node, std::vector<Node>> process();
};

struct F {
    struct Node {};
    std::tuple<bool, Node, std::vector<Node>> process(Node&);
};

template <typename T>
void testType(const char* name) {
    std::cout << std::boolalpha
        << name << ":\n"
        << "  HasNodeType:      " << HasNodeType<T> << "\n"
        << "  CanProcessNodeB:  " << CanProcessNodeB<T> << "\n"
        << "  NodeProcessorA:   " << NodeProcessorA<T> << "\n"
        << "  NodeProcessorB:   " << NodeProcessorB<T> << "\n"
        << std::endl;
}

int main(void) {
#define TEST_TYPE(x) testType<x>(#x)
    TEST_TYPE(A);
    TEST_TYPE(B);
    TEST_TYPE(C);
    TEST_TYPE(D);
    TEST_TYPE(E);
    TEST_TYPE(F);
#undef TEST_TYPE
}
