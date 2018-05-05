#ifndef COUNTER_H
#define COUNTER_H

#include <algorithm>
#include <map>
#include <unordered_map>
#include <initializer_list>

template <typename A, typename MapType>
class BaseCounter {
public:
  using key_type    = typename MapType::key_type;
  using mapped_type = typename MapType::mapped_type;
  using value_type  = typename MapType::value_type;
  using size_type   = typename MapType::size_type;

  template <typename T>
  BaseCounter(T first, T last) {
    std::for_each(first, last, [=](const A &x) { this->add(x); });
  }
  BaseCounter(const BaseCounter<A,MapType>& other) : m(other.m) {}
  BaseCounter(BaseCounter<A,MapType>&& other) : m(std::move(other.m)) {}
  BaseCounter(std::initializer_list<value_type> il) : m(il) {}
  BaseCounter(std::initializer_list<A> il) : BaseCounter(il.begin(), il.end()) {}

  void add(const A &elem, int amt = 1) {
    auto iter = m.find(elem);
    if (iter == m.end()) {
      m.insert({elem, amt});
    } else {
      iter->second += amt;
    }
  }
  const auto begin() const { return m.cbegin(); }
  const auto end() const { return m.cend(); }
  auto begin() { return m.begin(); }
  auto end() { return m.end(); }
  int count(const A &elem) {
    auto iter = m.find(elem);
    if (iter == m.end()) return 0;
    return iter->second;
  }
  int operator[](const A &elem) { return this->count(elem); }

private:
  MapType m;
};

template <typename A>
using UnorderedCounter = BaseCounter<A, std::unordered_map<A,int>>;

template <typename A>
using Counter = BaseCounter<A, std::map<A,int>>;

#endif // COUNTER_H
