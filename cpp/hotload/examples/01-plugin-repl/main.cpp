#include <cstdio> // needed before readline

#include <readline/history.h>
#include <readline/readline.h>

#include <string_view>
#include <memory>
#include <iostream>

#include <cstdlib>

// namespace {
// 
// class PluginWrapper {
// public:
//   PluginWrapper(std::string_view path);
// 
//   void tryPrintHelp();
// };
// 
// }

int main() {
  std::puts("Dynamic Load Repl.  Press 'help' to see commands and 'quit' to quit.");
  constexpr auto prompt = ">> ";

  const auto wrappedFree = [](auto* const ptr) { std::free(ptr); };
  for (std::unique_ptr<char, decltype(wrappedFree)> buf{readline(prompt)};
       buf;
       buf.reset(readline(prompt)))
  {
    const std::string_view input{buf.get()};
    if (input.empty()) {
      continue;
    }
    add_history(input.data());
    if (input == "quit") {
      break;
    } else if (input == "help") {
      std::puts("Built-in commands:\n"
                "  help     Print this help message.\n"
                "  quit     Quit the program.");
      // TODO: get commands and descriptions from plugin, or have plugin print its help.
      // TODO: add hotload command
      // TODO: pass them to plugins in alphabetical order.
    } else {
      std::cout << "Error: Unrecognized command: '" << input << "'\n";
    }
  }

  std::puts("Quitting.");
  return 0;
}
