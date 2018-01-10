#include <algorithm>
#include <ios>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include <cstring>  // for memcpy
#include <cmath>    // for sqrt and atan2

using stringlist  = std::vector<std::string>;
using intlist     = std::vector<int>;
using stringset   = std::unordered_set<std::string>;
using intset      = std::unordered_set<unsigned int>;

//const bool DEBUG = true;
const bool DEBUG = false;
template <typename T>
void debug(const T &item) {
  if (DEBUG) {
    std::cout << item << std::endl;
  }
}

#define my_assert(x) \
  if (!x) { \
    printf("Assertion failed!, line %d, %s", __LINE__, #x); \
  }

stringlist read_input() {
  stringlist inlist;
  while (!std::cin.eof()) {
    std::string line;
    std::getline(std::cin, line);
    if (line.size() > 0) { // ignore empty lines
      inlist.emplace_back(line);
    }
  }
  return inlist;
}

template <typename T>
void print_lines(const T &outlist, bool print = true) {
  if (print) {
    for (auto val : outlist) {
      std::cout << val << std::endl;
    }
  }
}

template <typename T>
std::vector<T> str_split(const std::string &instr) {
  if (instr == "") {
    return {};
  }
  std::vector<T> vals;
  std::istringstream streamer;
  streamer.str(instr);
  while(!streamer.eof()) {
    T val;
    streamer >> val;
    vals.emplace_back(val);
  }
  return vals;
}

template <typename A, typename B>
std::vector<A> map_keys(const std::map<A,B> &m) {
  std::vector<A> keys;
  for (auto &ab : m) {
    keys.emplace_back(ab.first);
  }
  return keys;
}

template <typename A, typename B>
std::vector<A> map_keys(const std::unordered_map<A,B> &m) {
  std::vector<A> keys;
  for (auto &ab : m) {
    keys.emplace_back(ab.first);
  }
  return keys;
}

template <typename A, typename B>
std::vector<B> map_values(const std::map<A,B> &m) {
  std::vector<B> values;
  for (auto &ab : m) {
    values.emplace_back(ab.second);
  }
  return values;
}

template <typename A, typename B>
std::vector<B> map_values(const std::unordered_map<A,B> &m) {
  std::vector<B> values;
  for (auto &ab : m) {
    values.emplace_back(ab.second);
  }
  return values;
}

struct Problem {
  std::string name;
  unsigned int difficulty;
  Problem(const std::string &_n, unsigned int _d) : name(_n), difficulty(_d) {}
};
using ProblemCount = std::unordered_map<Problem, unsigned int>;
using ProblemList = std::vector<Problem>;
using Permutations = std::vector<ProblemList>;

bool operator<(const Problem &a, const Problem &b) {
  return a.name < b.name
         || (a.name == b.name && a.difficulty < b.difficulty);
}

// For unordered_map
bool operator==(const Problem &a, const Problem &b) {
  return a.name == b.name && a.difficulty == b.difficulty;
}

// For unordered_map
namespace std {
  template <> struct hash<Problem>
  {
    size_t operator()(const Problem& x) const
    {
      return hash<std::string>()(x.name) + hash<unsigned int>()(x.difficulty);
    }
  };
}

std::ostream& operator<<(std::ostream& out, const Problem &p) {
  out << "(" << p.name << ", " << p.difficulty << ")";
  return out;
}

std::ostream& operator<<(std::ostream& out, const ProblemList &pl) {
  out << "Problems{";
  for (auto &p : pl) {
    out << p << ", ";
  }
  out << "}";
  return out;
}

template <typename A, typename B>
std::ostream& operator<<(std::ostream& out, const std::pair<A, B> &p) {
  out << "(" << p.first << ", " << p.second << ")";
  return out;
}

ProblemCount count_problems(const stringlist &inlist) {
  ProblemCount counts;

  for (auto it = inlist.begin() + 1; it != inlist.end(); ++it) {
    auto split = str_split<std::string>(*it);
    Problem p(split[0], std::stoi(split[1]));
    counts[p] += 1;
  }

  return counts;
}

Permutations find_permutations_recursive(
    const std::unordered_set<Problem> &pset,
    const stringset &remaining_topics,
    unsigned int current_difficulty,
    unsigned int max_difficulty,
    const ProblemList &chosen)
{
  if (current_difficulty > max_difficulty) {
    return {chosen};
  }

  Permutations permutations;
  for (auto &topic : remaining_topics) {
    if (1 == pset.count({topic, current_difficulty})) {
      stringset new_remaining_topics(remaining_topics);
      new_remaining_topics.erase(topic);
      ProblemList new_chosen(chosen);
      new_chosen.emplace_back(topic, current_difficulty);
      auto sub_perms = find_permutations_recursive(
          pset,
          new_remaining_topics,
          current_difficulty + 1,
          max_difficulty,
          new_chosen
          );
      permutations.insert(permutations.end(),
                          sub_perms.begin(), sub_perms.end());
    }
  }

  return permutations;
}

Permutations find_permutations(const ProblemList &unique_problems,
                               unsigned int max_difficulties) {
  // Grab the topics and difficulties
  using ProblemSet = std::unordered_set<Problem>;
  ProblemSet pset;
  stringset topics;
  intset difficulties;
  for (auto &p : unique_problems) {
    pset.insert(p);
    topics.insert(p.name);
    difficulties.insert(p.difficulty);
  }
  debug("\nTopics:");
  print_lines(topics, DEBUG);
  debug("\nDifficulties:");
  print_lines(difficulties, DEBUG);

  return find_permutations_recursive(
      pset, topics, 0, max_difficulties, {});
}

unsigned int count_combinations(const ProblemCount &counts,
                                const Permutations &permutations)
{
  unsigned int combinations = 0u;

  for (auto &p : permutations) {
    unsigned int p_combos = 1;
    for (auto &prob : p) {
      p_combos *= counts.at(prob);
    }
    combinations += p_combos;
  }

  return combinations;
}

int main() {
  // Read input
  auto inlist = read_input();
  auto config = str_split<int>(inlist[0]);
  auto N = config[0];
  ProblemCount counts = count_problems(inlist);
  debug("Counts:");
  print_lines(counts, DEBUG);

  // Get the permutations
  ProblemList unique_problems = map_keys(counts);
  Permutations permutations = find_permutations(unique_problems, N-1);
  debug("\nPermutations:");
  print_lines(permutations, DEBUG);

  debug("\n# Combinations:");
  auto combinations = count_combinations(counts, permutations);
  std::cout << combinations << std::endl;
}

