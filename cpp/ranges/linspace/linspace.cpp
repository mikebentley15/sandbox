#include <iostream>
#include <ranges>
#include <utility>
#include <stdexcept>
#include <concepts>

namespace view {

/// Returns count evenly spaced samples, calculated over the interval [start, stop]
/// Returned as a std::view
auto linspace(size_t start, size_t stop, size_t count) {
    const auto range = stop - start;
    return std::views::iota(size_t{0}, count)
         | std::views::transform([start, range, count](size_t index) {
               return start + index * range / (count - 1);
           });
}

template <class Range>
class PairWise : std::ranges::view_interface<std::pair<std::ranges::range_value_t<Range>, std::ranges::range_value_t<Range>>>
{
public:
    template <class ItType> class Iterator;

    template <class ItType>
    struct Sentinel {
        ItType end;
    };

    template <class ItType>
    class Iterator {
    public:
        Iterator(const Iterator &other) = default;
        Iterator(Iterator &&other) = default;

        bool operator==(const Iterator &other) const { return first_ == other.first_ && second_ == other.second_; }

        template <typename ItOther>
        bool operator==(const Sentinel<ItOther> &sentinel) const { return first_ == sentinel.end || second_ == sentinel.end; }

        auto operator*() { return std::make_pair(*first_, *second_); }
        void operator++() { ++first_; ++second_; }

        Iterator(ItType first) : first_(first), second_(first) {
            ++second_;
        }

    private:
    private:
        ItType first_;
        ItType second_;
    };

public:
    PairWise(Range range) : range_(std::move(range)) {}
    
    auto begin() const
    {
        return Iterator(std::ranges::begin(range_));
    }

    auto end() const
    {
        return Sentinel<decltype(std::ranges::end(range_))>(std::ranges::end(range_));
    }
private:
    Range range_;
};

/// pairwise
auto pairwise(std::ranges::viewable_range auto&& r) {
    auto beginning = std::ranges::begin(r);
    auto ending = std::ranges::end(r);
    using Value = decltype(*beginning);
    if (beginning == ending) {
        throw std::out_of_range("Cannot pairwise an empty range");
    }
    struct Transformer {
        mutable Value prev;
        auto operator()(const Value &next) {
           auto newpair = std::make_pair(prev, next);
           this->prev = next;
           return newpair;
        }
    };
    return r
         | std::views::drop(1)
         | std::views::transform(Transformer{*beginning});
}

}

namespace {

template <class A, class B, class C>
concept either = std::same_as<A, B> || std::same_as<A, C>;

template <class Range>
    requires either<std::ranges::range_value_t<Range>, std::size_t, std::pair<std::size_t, std::size_t>>
std::ostream& operator<<(std::ostream &out, const Range &range)
{
    out << "[ ";
    bool first = true;
    for (const auto &val : range) {
        if (!first) {
            out << ", ";
        }
        first = false;
        out << val;
    }
    out << " ]";
    return out;
}

template <class A, class B>
std::ostream& operator<<(std::ostream &out, const std::pair<A, B> &pair) {
    return out << "[ " << pair.first << ", " << pair.second << " ]";
}

}

#define PRINT_RANGE(x) { auto val = x; std::cout << #x << ": " << val << std::endl; }

int main() {
    auto linspace1 = view::linspace(10, 20, 4);
    auto linspace2 = view::linspace(10, 20, 5);
    PRINT_RANGE(view::linspace(10, 20, 4));
    PRINT_RANGE(view::linspace(10, 20, 5));
    auto tmp = view::PairWise(linspace1);
    for (auto val : tmp) {
        std::cout << val;
    }
    std::cout << std::endl;
    //PRINT_RANGE(tmp);
    //PRINT_RANGE(view::pairwise(linspace1));
    //PRINT_RANGE(view::pairwise(linspace2));
    return 0;
}
