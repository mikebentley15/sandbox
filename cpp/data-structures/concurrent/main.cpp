#include "LockFreeQueue.h"

#include <algorithm>
#include <atomic>
#include <thread>
#include <vector>
#include <chrono>
#include <iostream>

#include <cstdio>

namespace {

template <typename Function>
auto start_many_threads(uint32_t num_threads, Function &&f) {
  std::vector<std::thread> threads(num_threads);
  // start threads
  for (auto &thread : threads) {
    thread = std::thread(std::forward<Function>(f));
  }
  return threads;
}

auto wait_many_threads(std::vector<std::thread> &threads) {
  for (auto &thread : threads) {
    thread.join();
  }
}

void print_nanos(std::chrono::nanoseconds elapsed) {
  auto ns = elapsed.count();
  const auto s = ns / 1'000'000'000;
  ns = ns % 1'000'000'000;
  const auto ms = ns / 1'000'000;
  ns = ns % 1'000'000;
  const auto us = ns / 1'000;
  ns = ns % 1'000;

  std::cout
    << s << "s "
    << ms << "ms "
    << us << "us "
    << ns << "ns";
}

template <typename ProducerFunc, typename ConsumerFunc>
void run_producer_consumer(
    uint32_t num_producers, ProducerFunc &&produce,
    uint32_t num_consumers, ConsumerFunc &&consume,
    const std::vector<bool> &seen
    )
{
  auto start_time = std::chrono::steady_clock::now();

  // run producer/consumer experiment
  std::printf("  # producers: %u\n", num_producers);
  std::printf("  # consumers: %u\n", num_consumers);
  auto consumer_threads = start_many_threads(num_consumers, consume);
  auto producer_threads = start_many_threads(num_producers, produce);
  wait_many_threads(producer_threads);
  wait_many_threads(consumer_threads);

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed = end_time - start_time;
  std::cout << "Wall clock: ";
  print_nanos(elapsed);
  std::cout << "\n";

  // ensure all values were seen
  if (!std::all_of(seen.begin(), seen.end(), [](bool val) { return val; })) {
    std::printf("FAILURE: At least one value was not seen\n");
  }
}

} // end of unnamed namespace

int main() {
  LockFreeQueue<uint32_t> q;
  constexpr uint32_t N = 5'000'000;
  std::atomic<uint32_t> produce_count(0);
  std::atomic<uint32_t> consume_count(0);
  std::vector<bool> seen(N);

  auto produce = [&q, &produce_count, N] () {
    std::puts("produce() started");
    uint32_t value;
    while ((value = produce_count.fetch_add(1, std::memory_order_relaxed)) < N) {
      q.push(value);
    }
    std::puts("produce() finished");
  };

  auto consume = [&q, &consume_count, &seen, N] () {
    std::puts("consume() started");
    uint32_t value {};
    while (consume_count.fetch_add(1, std::memory_order_relaxed) < N) {
      while (!q.pop(value)) {}
      if (value >= N) {
        std::printf("FAILURE: Out of bounds value: %u\n", value);
      } else if (seen[value]) {
        std::printf("FAILURE: Duplicate value seen: %u\n", value);
      } else {
        seen[value] = true;
      }
    }
    std::puts("consume() finished");
  };

  auto reset = [&seen, &produce_count, &consume_count] () {
    std::puts("reset()");
    seen.assign(seen.size(), false);
    produce_count.store(0, std::memory_order_relaxed);
    consume_count.store(0, std::memory_order_relaxed);
  };

  std::printf("Now running producer/consumer example with %u pushes and pops.\n", N);
  run_producer_consumer(1, produce, 1, consume, seen); reset();
  run_producer_consumer(1, produce, 1, consume, seen); reset();
  run_producer_consumer(2, produce, 1, consume, seen); reset();
  run_producer_consumer(1, produce, 2, consume, seen); reset();
  run_producer_consumer(2, produce, 2, consume, seen); reset();
  run_producer_consumer(3, produce, 3, consume, seen); reset();
  run_producer_consumer(5, produce, 5, consume, seen); reset();

  return 0;
}
