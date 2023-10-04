#pragma once

#include "DirectSignal.hpp"

#include <functional>
#include <unordered_set>
#include <utility>

template <class Key = int, class FunctionSignature = void()>
class SignalMapper;

/// Class to map multiple signals to a single signal with an extra key argument
/// The extra key argument uniquely identifies the signal source.
template <class Key, class ...Args>
class SignalMapper<Key, void(Args...)> {
public: // types
    using FunctionSignature = void(Args...);
    using FunctionType = std::function<FunctionSignature>;

public: // signals
    /// Connect to this for the mapped signal
    DirectSignal<void(Key, Args...)> mapped;

public: // methods
    SignalMapper() = default;

    // disable moves because that will invalidate the mappings
    SignalMapper(const SignalMapper&) = delete;
    SignalMapper(SignalMapper&&) = delete;
    SignalMapper& operator=(const SignalMapper&) = delete;
    SignalMapper& operator=(SignalMapper&&) = delete;

    bool isMapped(const Key &key) const { return mappedKeys_.contains(key); }

    /// Map a signal with the given key.
    /// @return true if the key was available and the mapping was made.  False otherwise.
    template <class Signal>
    bool map(Key key, Signal &&signal)
    {
        auto [iter, wasInserted] = mappedKeys_.insert(std::move(key));
        if (!wasInserted) {
            return false;
        }
        signal.connect([this, key = *iter](Args&&... args) {
            mapped.emit(key, std::forward<Args>(args)...);
        });
        return true;
    }

private:
    std::unordered_set<Key> mappedKeys_;
};
