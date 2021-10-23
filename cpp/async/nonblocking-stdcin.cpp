#include <iostream>
#include <chrono>

#include <unistd.h>
#include <poll.h>
//#include <stropts.h> // not available on my platform

template <typename F>
double time(F &&func) {
  auto start = std::chrono::steady_clock::now();
  func();
  auto end = std::chrono::steady_clock::now();
  auto diff = std::chrono::duration<double>(end - start);
  return diff.count();
}

bool strategy_peek() { // doesn't work, blocks when input is not available
  char ch;
  ch = std::cin.peek(); // oh man!  this blocks if there's nothing!
  std::cout << "cin.peek() returned '" << ch << "'\n";
  return true;
}

bool strategy_fseek(FILE* stream) { // doesn't work, always returns true
  long int nchars;
  if (fseek(stream, 0, SEEK_END) != 0) return false;
  if ((nchars = ftell(stream)) < 0)    return false;
  if (fseek(stream,0,SEEK_SET) != 0)   return false;
  return (nchars > 0);
}

bool strategy_poll() { // doesn't work, always returns false
  struct pollfd fds[1];
  fds[0].fd = 0; // 0 is standard in
  fds[0].events = POLLRDBAND;
  int ret = poll(fds, 1, 1);
  return ret > 0;
}

// from https://stackoverflow.com/questions/26948723/checking-the-stdin-buffer-if-its-empty
bool strategy_select() {
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);
  fd_set savefds = readfds;

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  int chr;

  int sel_rv = select(1, &readfds, NULL, NULL, &timeout);
  //if (sel_rv > 0) {
  //  puts("Input:");
  //  while ((chr = getchar()) != EOF) putchar(chr);
  //} else if (sel_rv == -1) {
  //  perror("select failed");
  //}

  readfds = savefds;
  return sel_rv > 0;
}

//bool strategy_ioctl() { // not supported on Linux
//  int n;
//  return ioctl(0, I_NREAD, &n) == 0 && n > 0;
//}

bool does_stdin_have_stuff() {
  //return strategy_peek();  // doesn't work
  //return strategy_fseek(); // doesn't work
  //return strategy_poll();
  //return strategy_ioctl(); // not supported
  return strategy_select();
}

int main(int arg_count, char *arg_list[]) {
  char buffer[10000]; // big buffer
  int buf_size = 0;
  double secs;
  bool readyread = false;
  auto readyread_check = [&readyread, &secs]() {
    auto wrap = [&readyread]() { readyread = does_stdin_have_stuff(); };
    secs = time(wrap);
    return readyread;
  };

  while (true) {
    std::cout << "before calling does_stdin_have_stuff()" << std::endl;
    readyread_check();
    std::cout << "after calling does_stdin_have_stuff() (" << secs << " secs)\n"
              << "  - returned " << (readyread ? "true" : "false") << "\n"
              << "\n";
    if (readyread) {
      while (readyread_check()) {
        std::cin >> buffer[buf_size++];
      }
      std::cout << "  - read " << buf_size << " characters\n"
                << "  - '";
      for (int i = 0; i < buf_size; i++) {
        std::cout.put(buffer[i]);
      }
      std::cout << "'" << std::endl;
      buf_size = 0;
    }
    std::cout << "\n"
              << "Input something now while I wait for 3 seconds\n"
              << std::endl;

    sleep(3);
  }
}
