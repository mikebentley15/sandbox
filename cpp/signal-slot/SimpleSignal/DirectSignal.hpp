#pragma once

#include <functional>
#include <vector>
#include <utility>

template<class FunctionSignature = void()>
class DirectSignal;

template<class... Args>
class DirectSignal<void(Args...)> {
public:
    using FunctionSignature = void(Args...);
    using FunctionType = std::function<FunctionSignature>;

public:
    DirectSignal() = default;
    DirectSignal(DirectSignal&&) = default;
    DirectSignal& operator=(DirectSignal&&) = default;
    ~DirectSignal() noexcept = default;

    DirectSignal(const DirectSignal&) = delete;
    DirectSignal& operator=(const DirectSignal&) = delete;

    /// TODO: enable connection to functions with fewer arguments
    /// TODO: enable connection to functions with convertible arguments
    void connect(FunctionType slot) {
        connections_.push_back(std::move(slot));
    }

    template<class... EmitArgs>
    requires std::is_invocable_v<FunctionType, EmitArgs...>
    void emit(EmitArgs&&... args) const {
        if (blocked_) {
            return;
        }
        for (const auto &connection : connections_) {
            if (blocked_) { // in case one of the connections blocked the signal
                return;
            }
            std::invoke(connection, std::forward<EmitArgs>(args)...);
        }
    }

    bool isBlocked() const { return blocked_; }
    void setBlocked(bool blocked = true) { blocked_ = blocked; }

private:
    std::vector<FunctionType> connections_;
    bool blocked_{false};
};
