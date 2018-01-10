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

intlist bowling_scores(const stringlist &inlist) {
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

int main() {
  auto inlist = read_input();
  auto scores = bowling_scores(inlist);
  print_lines(scores);
}
