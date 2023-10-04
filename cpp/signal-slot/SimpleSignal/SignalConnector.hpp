#pragma once

/// Thin wrapper around a signal that only provides connect() for public consumption
/// Possible usage pattern:
///
///   class A {
///   public:
///       auto myImportantEvent() { return SignalConnector{myImportantSignal_}; }
///   private:
///       DirectSignal<void()> myImportantSignal_;
///   };
///
/// Then from the caller
///
///   A a;
///   a.myImportantEventSignal().connect([]() {
///       std::cout << "myImportantEvent happened";
///   });
///
/// Alternatively:
///
///   class A {
///   public:
///       SignalConnector<DirectSignal<void()>> myImportantEvent;
///       A() : myImportantEvent(myImportantEventSignal_) {}
///   private:
///       DirectSignal<void()> myImportantEventSignal_{};
///   };
template <class Signal>
class SignalConnector {
public:
    using FunctionSignature = typename Signal::FunctionSignature;
    using FunctionType = typename Signal::FunctionType;

public:
    SignalConnector() = delete;

    SignalConnector(Signal &signal) : signal_(signal) {}
    SignalConnector(const SignalConnector&) = default;
    SignalConnector(SignalConnector&&) = default;
    SignalConnector& operator=(const SignalConnector&) = default;
    SignalConnector& operator=(SignalConnector&&) = default;
    ~SignalConnector() noexcept = default;

    void connect(FunctionType slot) {
        signal_.get().connect(std::move(slot));
    }

private:
    std::reference_wrapper<Signal> signal_;
};
