#include <algorithm>
#include <chrono>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>

#include <cstdint>

std::ostream debug_stream(nullptr);
void debug_enable()  { debug_stream.rdbuf(std::cout.rdbuf()); }

class Timer {
public:
  Timer() : _start(std::chrono::high_resolution_clock::now()) {}
  auto start() const { return _start; }
  auto duration() const {
    using namespace std::chrono;
    decltype(_start) stop = high_resolution_clock::now();
    return duration_cast<std::chrono::duration<double>>(stop - _start);
  }
  auto seconds() const { return this->duration().count(); }
private:
  std::chrono::high_resolution_clock::time_point _start;
};

Timer t;

std::ostream& operator<<(std::ostream& out, const Timer &t) {
  return out << std::to_string(t.seconds());
}

std::ostream& debug() {
  return debug_stream << "debug(" << t << " sec): ";
}

struct Interval {
  size_t a;
  size_t b;
  bool intersects(const Interval &other) const {
    return this->contains(other.a) || this->contains(other.b) ||
           other.contains(this->a) || other.contains(this->b);
  }
  bool contains(size_t x) const {
    return a <= x && x <= b;
  }
  Interval intersection(const Interval &other) const {
    return Interval{std::max(a, other.a), std::min(b, other.b)};
  }
};

std::istream& operator>>(std::istream& in, Interval &i) {
  return in >> i.a >> i.b;
}

std::ostream& operator<<(std::ostream& out, const Interval &i) {
  return out << "[" << i.a << ", " << i.b << "]";
}

bool operator<(const Interval &x, const Interval &y) {
  return x.a < y.a || (x.a == y.a && x.b < y.b);
}

bool operator==(const Interval &x, const Interval &y) {
  return x.a == y.a && x.b == y.b;
}

struct Query {
  Interval i;
  uint64_t val;
};

std::istream& operator>>(std::istream& in, Query &q) {
  return in >> q.i >> q.val;
}

std::ostream& operator<<(std::ostream& out, const Query &q) {
    return out << "Query(" << q.i << ", " << q.val << ")";
}

bool operator<(const Query &x, const Query &y) {
  return x.i < y.i || (x.i == y.i && x.val < y.val);
}

bool operator==(const Query &x, const Query &y) {
  return x.i == y.i && x.val == y.val;
}

uint64_t with_vector(int n, const std::vector<Query> &queries) {
  debug() << "Inside with_vector()\n";
  std::vector<uint64_t> vec(n);
  for (auto &q : queries) {
    for (int i = q.i.a-1; i < q.i.b; i++) {
        vec[i] += q.val;
    }
  }

  auto max_value = *std::max_element(vec.begin(), vec.end());
  debug() << "max value = " << max_value << std::endl;    
  return max_value;
}

// Assumes the queries are already sorted.
uint64_t with_sweep(int n, const std::vector<Query> &queries) {
  auto by_ending = [](Query x, Query y) {
    return x.i.b > y.i.b;
  };
  using by_ending_comparator = std::reference_wrapper<decltype(by_ending)>;
  using PQ = std::priority_queue<Query, std::vector<Query>, by_ending_comparator>;
  PQ active(std::ref(by_ending));

  // Sweep across the array indices, activating intervals as we run into them.
  auto it_q = queries.begin();
  uint64_t max_value = 0;
  uint64_t current_value = 0;
  for (size_t i = 1; i <= n && it_q != queries.end(); i++) {
    // See if anything is activated
    while (it_q != queries.end() && it_q->i.a <= i) {
      current_value += it_q->val;
      active.push(*it_q);
      //debug() << "  activate:   " << *it_q << std::endl;
      it_q++;
    }
    // See what is deactivated
    while (active.size() > 0 && active.top().i.b < i) {
      current_value -= active.top().val;
      //debug() << "  deactivate: " << active.top() << std::endl;
      active.pop();
    }
    if (current_value > max_value) {
      max_value = current_value;
    }
  }
  return max_value;
}

/// Does not assume sorted - sorts in the process
std::vector<Query> condensed_read(int q) {
  std::map<Interval, uint64_t> counts;

  for (int i = 0; i < q; i++) {
    Interval interval;
    uint64_t value;
    std::cin >> interval >> value;
    counts[interval] += value;
  }

  std::vector<Query> condensed;
  for (auto &interval_count : counts) {
    auto &interval = interval_count.first;
    auto &count    = interval_count.second;
    condensed.push_back({interval, count});
  }

  return condensed;
}

int main() {
  //debug_enable();

  int n, q;
  std::cin >> n >> q;
  debug() << "n = " << n << ", q = " << q << std::endl;
  debug() << "Before reading input\n";
  auto queries = condensed_read(q);
  debug() << "Before with_sweep()" << std::endl;
  std::cout << with_sweep(n, queries) << std::endl;
  debug() << "All done" << std::endl;
  return 0;
}

