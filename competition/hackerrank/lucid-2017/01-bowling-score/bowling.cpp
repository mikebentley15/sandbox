#include <ios>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

using stringlist = std::vector<std::string>;
using intlist  = std::vector<int>;

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
void print_lines(const std::vector<T> outlist) {
  for (auto val : outlist) {
    std::cout << val << std::endl;
  }
}

template <typename T>
bool equal(const std::vector<T>& a, const std::vector<T>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); i++) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

template <typename T>
void assert_equal(const std::vector<T>& expected, const std::vector<T>& actual,
                  std::string name = "test case") {
  if (!equal(expected, actual)) {
    std::cout << "Failed " << name << std::endl;
    std::cout << "Expected:\n"; print_lines(expected);
    std::cout << "Actual:\n";   print_lines(actual);
  } else {
    std::cout << "Passed " << name << std::endl;
  }
}

intlist str_to_ints(const std::string &instr) {
  if (instr == "") {
    return {};
  }
  intlist vals;
  std::istringstream streamer;
  streamer.str(instr);
  while(!streamer.eof()) {
    int val;
    streamer >> val;
    vals.emplace_back(val);
  }
  return vals;
}

intlist bowling_scores(const stringlist& inlist) {
  intlist scores;
  for (int i = 1; i < inlist.size(); i++) {
    auto bowler = inlist[i];
    auto throws = str_to_ints(bowler);
    if (throws.size() == 0) {
      continue;
    }
    intlist frame_scores = {0,0,0,0,0,0,0,0,0,0};
    size_t idx = 0;
    int frame = 0;
    bool in_frame = false;
    for (size_t idx = 0; idx < throws.size(); idx++) {
      if (frame != 9 && throws[idx] == 10) {
        frame_scores[frame] += throws[idx] + throws[idx + 1] + throws[idx + 2];
        frame++;
        in_frame = false;
      } else if (frame != 9 && in_frame) {
        if (frame_scores[frame] + throws[idx] == 10) {
          frame_scores[frame] += throws[idx + 1];
        }
        frame_scores[frame] += throws[idx];
        frame++;
        in_frame = false;
      } else {
        frame_scores[frame] += throws[idx];
        in_frame = true;
      }
    }
    int score = 0;
    for (auto scr : frame_scores) {
      score += scr;
    }
    scores.emplace_back(score);
  }
  return scores;
}

void test_bowling_scores() {
  stringlist testin_1 = {
    "3",
    "10 10 10 10 10 10 10 10 10 10 10 10",
    "5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5",
    "10 0 0 10 0 0 10 0 0 10 0 0 10 0 0",
  };
  intlist expected_1 = {300, 150, 50};
  intlist actual_1 = bowling_scores(testin_1);
  assert_equal(expected_1, actual_1, "test 1");

  stringlist testin_2 = {
    "3",
    "4 5 5 4 6 3 2 7 7 2 3 6 6 3 8 1 1 8 9 0",
    "6 2 5 3 3 7 8 1 10 4 6 8 1 7 3 10 4 6 6",
    "10 0 10 3 3 10 9 1 6 3 3 5 9 1 4 4 10 6 4",
  };
  intlist expected_2 = {90, 146, 137};
  intlist actual_2 = bowling_scores(testin_2);
  assert_equal(expected_2, bowling_scores(testin_2), "test 2");
}

int main() {
  test_bowling_scores();

  //auto inlist = read_input();
  return 0;
}



