#pragma once

#include <ranges>
#include <utility>

namespace view {

template <class Range>
class PairWise : std::ranges::view_interface<PairWise<Range>> {
public:
  template <class ItType> class Iterator;

  template <class ItType> struct Sentinel {
    ItType end;
  };

  template <class ItType> class Iterator {
  public:
    Iterator(const Iterator &other) = default;
    Iterator(Iterator &&other) = default;

    bool operator==(const Iterator &other) const {
      return first_ == other.first_ && second_ == other.second_;
    }

    template <typename ItOther>
    bool operator==(const Sentinel<ItOther> &sentinel) const {
      return first_ == sentinel.end || second_ == sentinel.end;
    }

    auto operator*() { return std::make_pair(*first_, *second_); }
    void operator++() {
      ++first_;
      ++second_;
    }

    Iterator(ItType first) : first_(first), second_(first) { ++second_; }

  private:
    ItType first_;
    ItType second_;
  };

public:
  PairWise(Range range) : range_(std::move(range)) {}

  auto begin() const { return Iterator(std::ranges::begin(range_)); }

  auto end() const {
    return Sentinel<decltype(std::ranges::end(range_))>(
        std::ranges::end(range_));
  }

private:
  Range range_;
};

/// pairwise
auto pairwise(std::ranges::viewable_range auto &&r) {
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
  return r | std::views::drop(1) |
         std::views::transform(Transformer{*beginning});
}

} // namespace view
