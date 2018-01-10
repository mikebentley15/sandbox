#include <algorithm>
#include <ios>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <unordered_map>

#include <cstring>  // for memcpy

//const bool DEBUG = true;
const bool DEBUG = false;

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

void debug(const std::string &msg) {
  if (DEBUG) {
    puts(msg.c_str());
  }
}

// Count two-way trips for all passengers
unsigned int count_trip_floors(unsigned int counts[5001],
                               unsigned int capacity) {
  debug("count_trip_floors");

  // Copy the input
  unsigned int counts_copy[5001];
  memcpy(counts_copy, counts, 5001 * sizeof(unsigned int));

  // Count full trips
  unsigned int floors = 0;
  unsigned int fill = 0;
  unsigned int highest_floor_to_go = -1;
  for (auto i = 5000; i > 0; i--) {
    while (fill + counts_copy[i] >= capacity) {
      counts_copy[i] -= capacity - fill;
      fill = 0;
      if (highest_floor_to_go != -1) {
        floors += 2*highest_floor_to_go;
        highest_floor_to_go = -1;
      } else {
        floors += 2*i;
      }
    }
    fill += counts_copy[i];
    counts_copy[i] = 0;
    if (fill != 0 && highest_floor_to_go == -1) {
      highest_floor_to_go = i;
    }
  }

  // Finish the last partially filled elevator
  if (fill != 0 && highest_floor_to_go != -1) {
    floors += 2*highest_floor_to_go;
  }

  debug("  floors returned: " + std::to_string(floors));

  return floors;
}

unsigned int min_elevator_floors(const stringlist &inlist) {
  debug("min_elevator_floors");
  unsigned int min_floors = 0;

  auto config = str_to_ints(inlist[0]);
  auto capacity = config[0];

  // Count how many need to get to floors.
  auto max_floor = 5000;
  unsigned int positive_counts[max_floor + 1]{};
  unsigned int negative_counts[max_floor + 1]{};
  int max_positive = 0;
  int max_negative = 0;
  for (auto it = inlist.begin() + 1; it != inlist.end(); it++) {
    int floor = std::stoi(*it);
    if (floor > 0) {
      ++positive_counts[floor];
      if (max_positive < floor) {
        max_positive = floor;
      }
    } else if (floor < 0) {
      ++negative_counts[-floor];
      if (max_negative < -floor) {
        max_negative = -floor;
      }
    }
  }

  // count the number of floors traveled
  min_floors += count_trip_floors(positive_counts, capacity);
  min_floors += count_trip_floors(negative_counts, capacity);
  min_floors -= std::max(max_positive, max_negative);

  debug("min_floors: " + std::to_string(min_floors));
  return min_floors;
}

struct Clock {
  unsigned int hour;   // 24-hour format
  unsigned int minute;
  unsigned int second;
};

Clock simplify(const Clock &from_clock) {
  Clock new_time = from_clock;
  
  // Simplify seconds
  new_time.minute += new_time.second / 60;
  new_time.second = new_time.second % 60;

  // Simplify minutes
  new_time.hour += new_time.minute / 60;
  new_time.minute = new_time.minute % 60;

  // Simplify hours
  //auto day = new_time.hour / 24;
  new_time.hour = new_time.hour % 24;

  return new_time;
}

void print_clock(const Clock &time) {
  Clock to_print = simplify(time);
  int print_hour = to_print.hour % 12;
  bool is_am = to_print.hour < 12;
  if (print_hour == 0) {
    print_hour = 12;
  }
  printf("%02d:%02d:%02d %s\n", print_hour, to_print.minute, to_print.second,
         (is_am ? "AM" : "PM"));
}

int main() {
  auto inlist = read_input();
  auto floors = min_elevator_floors(inlist);
  unsigned int seconds = floors * 20;
  Clock time {9, 0, seconds};
  print_clock(time);
}

