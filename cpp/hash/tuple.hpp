#pragma once

#include <functional> // for std::hash
#include <tuple>
#include <iostream>

class HashCombiner
{
public:
    HashCombiner(size_t hashValue)
        : hashValue_{hashValue}
    {}
    operator size_t() const { return hashValue_; }

private:
    size_t hashValue_;
};

HashCombiner operator+(HashCombiner a, HashCombiner b)
{
    std::cout << "Combining hashes (" << a << ", " << b << ")\n";
    return {size_t{a} ^ (size_t{b} + size_t{0x9e3779b9} + (size_t{a} << 6) + (size_t{a} >> 2))};
}

template<class FirstType, class... RemainingTypes>
struct std::hash<std::tuple<FirstType, RemainingTypes...>>
{
    using ValueType = std::tuple<FirstType, RemainingTypes...>;

    size_t operator()(const ValueType& value) const
    {
        auto combine = [](auto&&... args) -> std::size_t {
            return (... + HashCombiner(std::hash<std::remove_cv_t<std::remove_reference_t<decltype(args)>>>()(args)));
        };
        return std::apply(combine, value);
    }
};
