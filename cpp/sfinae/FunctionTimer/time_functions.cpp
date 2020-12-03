#include "FunctionTimer.h"
#include "time_function_call.h"

#include <iostream>

#include <unistd.h>  // for usleep()

void test_timer() {
  std::cout << "\n---- test_timer() ----\n";

  FunctionTimer timer;

  std::cout
    << "before call to time()\n"
    << "  timer size: "  << timer.get_times().size() << std::endl;

  timer.time([]() { usleep(20000); });

  std::cout
    << "after first call to time() (void return)\n"
    << "  timer size: "  << timer.get_times().size() << std::endl
    << "  times:      [" << timer.get_times()[0] << "]" << std::endl;

  timer.time([]() { usleep(40000); });

  std::cout
    << "after second call to time() (void return)\n"
    << "  timer size: "  << timer.get_times().size() << std::endl
    << "  times:      [" << timer.get_times()[0] << ", "
                         << timer.get_times()[1] << "]" << std::endl;

  auto val = timer.time([]() { return 42; });

  std::cout
    << "after third call to time() (nonvoid return)\n"
    << "  timer size: "  << timer.get_times().size() << std::endl
    << "  times:      [" << timer.get_times()[0] << ", "
                         << timer.get_times()[1] << ", "
                         << timer.get_times()[2] << "]" << std::endl
    << "  returned:   "  << val << std::endl;
}

void test_time_function_call() {
  std::cout << "\n---- test_time_function_call() ----\n";

  std::cout
    << "version 1 with void return:\n"
    << "  time:   " << time_function_call([]() { usleep(20000); }) << std::endl;

  float time = 0.0f;
  time_function_call([]() { usleep(40000); }, time);
  std::cout
    << "version 2 with void return:\n"
    << "  time:   " << time << std::endl;

  int retval = 0;
  time = time_function_call([&retval]() { retval = 22; return retval; });
  std::cout
    << "version 1 with nonvoid return:\n"
    << "  retval: " << retval << "\n"
    << "  time:   " << time << std::endl;

  retval = 0;
  retval = time_function_call([]() { return 17; }, time);
  std::cout
    << "version 2 with nonvoid return:\n"
    << "  retval: " << retval << "\n"
    << "  time:   " << time << std::endl;
}

int main() {
  test_timer();
  test_time_function_call();
  std::cout << std::endl;
  return 0;
}
