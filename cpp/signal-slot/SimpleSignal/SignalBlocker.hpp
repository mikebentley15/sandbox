#pragma once

#include <functional>

/// RAII class for setBlocked(true)..setBlocked(false) on a signal
template<class Signal>
class SignalBlocker {
public:
    SignalBlocker(Signal& signal)
        : signal_(std::ref(signal))
        , originalState_(signal_.isBlocked())
    {
        reblock();
    }

    ~SignalBlocker() noexcept { signal_.get().setBlocked(originalState_); }
    void unblock() { signal_.get().setBlocked(false); }
    void reblock() { signal_.get().setBlocked(true); }

private:
    std::reference_wrapper<Signal> signal_;
    bool originalState_;
};
