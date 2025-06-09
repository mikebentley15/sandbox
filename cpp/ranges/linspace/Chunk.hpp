#pragma once

#include <ranges>
#include <tuple>
#include <utility>

namespace view {

template <std::ranges::viewable_range Range>
class Chunk : std::ranges::view_interface<Chunk<Range>> {
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
      return iter_ == sentinel.end;
    }

    auto operator*() {
      return std::ranges::subrange {*first_, *second_);
      }
      void operator++() { std::advance(iter_, chunkSize_); }

      Iterator(ItType beginIter, ItType endIter, const std::size_t chunkSize)
          : beginIter_{iter}, chunkSize_{chunkSize} {}

    private:
      ItType beginIter_;
      ItType endIter_;
    };

  public:
    Chunk(Range range, const std::size_t chunkSize)
        : range_{std::move(range)}, chunkSize_{chunkSize} {}
    auto begin() const {
      return Iterator{std::ranges::begin(range_), chunkSize_};
    }
    auto end() const { return Sentinel{std::ranges::end(range_)}; }

  private:
    Range range_;
    std::size_t chunkSize_;
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
