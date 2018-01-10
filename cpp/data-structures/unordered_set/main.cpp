#include "SimpleSet.h"
#include "util.h"

#include <iostream>
#include <random>
#include <set>
#include <unordered_set>

#include <cassert>

#ifndef UNUSED_ARG
#define UNUSED_ARG(x) (void)x
#endif // UNUSED_ARG

uint64_t time_function(std::function<void(void)> func) {
  // Do one loop before doing actual timing
  auto before = get_time();
  func();
  auto after = get_time();

  // Now do the actual timing
  auto loopcount = 100;
  before = get_time();
  for (auto i = 0; i < loopcount; i++) {
    func();
  }
  after = get_time();

  // TODO: calculate the loop cost and subtract it.

  //return (after - before) / loopcount;
  return (after - before);
}


int main(int argCount, char* argList[]) {
  UNUSED_ARG(argCount);
  UNUSED_ARG(argList);

  std::mt19937_64 generator;
  std::set<uint64_t> entry_set;
  uint64_t entries[25000];
  for (auto i = 0; i < 25000; i++) {
    //
    // Randomly generate numbers to put into the set
    //

    //auto entry = generator();
    //// Keep generating while entry is not unique.
    //while (entry_set.find(entry) != entry_set.end()) {
    //  entry = generator();
    //}
    //entries[i] = static_cast<uint64_t>(entry);


    //
    // Instead just use the addresses of the entries themselves
    // This is more realistic to the use case we care about
    //

    entries[i] = reinterpret_cast<uint64_t>(entries + i); // store the address
  }

  SimpleSet<uint64_t, 25000> simple_set;

  auto simple_set_func = [&simple_set, &entries]() {
    bool contains = false;
    for (uint64_t i = 0; i < 25000; i++) {
      assert(false == simple_set.contains(entries[i]));
      assert(true  == simple_set.insert(entries[i]));
      assert(true  == simple_set.contains(entries[i]));
    }
    simple_set.clear();
  };

  std::set<uint64_t> std_set;

  auto std_set_func = [&std_set, &entries]() {
    bool contained = false;
    for (uint64_t i = 0; i < 25000; i++) {
      assert(std_set.find(entries[i]) == std_set.end());
      std_set.insert(entries[i]);
      assert(std_set.find(entries[i]) != std_set.end());
    }
    std_set.clear();
  };

  std::unordered_set<uint64_t> std_unordered_set(25000);

  auto std_unordered_set_func = [&std_unordered_set, &entries]() {
    bool contained = false;
    for (uint64_t i = 0; i < 25000; i++) {
      assert(std_unordered_set.find(entries[i]) == std_unordered_set.end());
      std_unordered_set.insert(entries[i]);
      assert(std_unordered_set.find(entries[i]) != std_unordered_set.end());
    }
    std_unordered_set.clear();
  };

  std::cout << "Timing for SimpleSet:           " << time_function(simple_set_func) << " ms" << std::endl;
  std::cout << "Timing for std::set:            " << time_function(std_set_func) << " ms" << std::endl;
  std::cout << "Timing for std::unordered_set:  " << time_function(std_unordered_set_func) << " ms" << std::endl;
  std::cout << "SimpleSet collisions:           " << simple_set.collisions() << std::endl;

  return 0;
}
