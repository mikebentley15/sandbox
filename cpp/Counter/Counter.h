#ifndef COUNTER_H
#define COUNTER_H

#include <algorithm>
#include <map>
#include <unordered_map>
#include <initializer_list>
#include <iostream>

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

// I had this as operator<< and it worked for all sorts of containers.
//
// But the problem was that it matched too many things.  Specifically, it made
// a call to this operator with std::string ambiguous and therefore a compiler
// error.
//
// This pattern is called "template template parameters" where one or more of
// the template parameters also have template parameters.
//
// The simpler way for this template also works having a single template called
// Cont instead of all that are seen here.  However, that also overrides simple
// calls to the operator<<() function such as
//   std::cout << "hello\n";
// that would all of a sudden become ambiguous since range-based for loops work
// with static arrays.
//
// I don't know an easy way to only target container types...  We could require
// that the type contains begin() and end(), but that is also not very helpful
// since it will still conflict with std::string (which technically std::string
// is a container for characters, but it's bad because it already has an
// implementation for operator<<()).
template <typename T, template<class,class...> class Cont, class... Args>
std::ostream& myoutput(std::ostream& out, const Cont<T, Args...> &c) {
  bool first = true;
  out << "[";
  for (const auto &x : c) {
    if (!first) out << ", ";
    first = false;
    out << x;
  }
  out << "]";
  return out;
}

template <typename A, typename B>
std::ostream& operator<<(std::ostream& out, const BaseCounter<A,B> &c) {
  return myoutput(out, c);
}

template <typename A, typename B>
std::ostream& operator<<(std::ostream& out, const std::pair<A,B> &p) {
  return out << "(" << p.first << ", " << p.second << ")";
}

#endif // COUNTER_H
