#include "DirectSignal.hpp"
#include "SignalBlocker.hpp"
#include "SignalMapper.hpp"
#include "SignalConnector.hpp"

#include <string>
#include <string_view>
#include <stdexcept>
#include <iostream>

using namespace std::literals::string_literals;

class A {
public:
    DirectSignal<void()> shutdownSignal;
    DirectSignal<void(const std::string&)> messageSignal;

public:
    ~A() noexcept {
        try {
            shutdownSignal.emit();
        } catch(const std::exception &ex){
            std::cerr << ex.what() << std::endl;
        }
    }
    void constructMessage(std::string_view pre, std::string_view post) {
        messageSignal.emit(std::string(pre) + "::"s + std::string(post));
    }
};

class B {
public:
    DirectSignal<void(std::string_view, std::string_view)> messageReadyToCreateSignal;
    auto shutdownSignal() { return SignalConnector{shutdownSignal_}; }

public:
    ~B() noexcept {
        try {
            shutdownSignal_.emit();
        } catch(const std::exception &ex){
            std::cerr << ex.what() << std::endl;
        }
    }

    void prepareMessage() {
        messageReadyToCreateSignal.emit("Hello", "World!");
    }

private:
    DirectSignal<void()> shutdownSignal_;
};

int main() {
    // create objects
    A a;
    B b;
    SignalMapper<std::string_view, void()> destroyMap;

    // connect signals
    a.shutdownSignal.connect([]() { std::cout << "a.~A()\n"; });
    b.shutdownSignal().connect([]() { std::cout << "b.~B()\n"; });
    a.messageSignal.connect([](const std::string& message) { std::cout << "a.messageSignal: " << message << "\n"; });
    b.messageReadyToCreateSignal.connect([](std::string_view pre, std::string_view post) {
        std::cout << "b.messageReadyToCreateSignal: {pre: " << pre << ", post: " << post << "}\n";
    });
    b.messageReadyToCreateSignal.connect(std::bind_front(&A::constructMessage, &a));
    destroyMap.map("from a", a.shutdownSignal);
    destroyMap.map("from b", b.shutdownSignal());
    destroyMap.mapped.connect([](std::string_view key) {
        std::cout << "destroyMap.mapping: " << key <<"\n";
    });

    // create some events
    b.prepareMessage();
    a.constructMessage("Michael", "Bentley");

    return 0;
}
