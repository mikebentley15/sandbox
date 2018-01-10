#include <iostream>
#include <future>

int main() {
  auto max_val = 1000000;
  auto future_val = std::async([max_val](){
    long sum = 0;
    for (int i = 1; i <= max_val; i++)
      sum += i;
    return sum;
  });

  // do some work here before we need the sum.

  auto sum = future_val.get();
  // now use the sum
  std::cout << "Sum from 1 to " << max_val << " = " << sum << std::endl;

  return 0;
}
