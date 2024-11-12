#include "PluginWrapper.hpp"

#include <cstdio> // needed before readline

#include <readline/history.h>
#include <readline/readline.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <ranges>
#include <string_view>
#include <memory>
#include <iostream>

#include <cstdlib>

namespace bfs = boost::filesystem;

namespace {

auto isPlugin(const bfs::path& file) -> bool {
  return file.extension() == ".so" && file.filename().string().starts_with("lib");
}

auto loadPlugins(const bfs::path& dir) -> std::vector<PluginWrapper>{
  std::vector<bfs::path> sharedLibs;
  auto dirContents = std::ranges::subrange(bfs::directory_iterator(dir),
                                           bfs::directory_iterator());
  std::ranges::copy_if(dirContents, std::back_inserter(sharedLibs), isPlugin);
  std::vector<PluginWrapper> plugins;
  plugins.reserve(sharedLibs.size());
  std::ranges::transform(sharedLibs, std::back_inserter(plugins),
      [](const bfs::path& libpath) { return PluginWrapper{libpath.string()}; });
  return plugins;
}

} // namespace

int main(int argCount, char* argList[]) {
  if (argCount < 1) {
    std::puts("Error: need at least the argument name");
    return 1;
  }

  std::puts("Dynamic Load Repl.  Press 'help' to see commands and 'quit' to quit.");
  constexpr auto prompt = ">> ";
  const bfs::path appPath{argList[0]};
  auto plugins = loadPlugins(appPath.parent_path());

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
      std::puts("\n"
                "Built-in commands:\n"
                "  help       Print this help message.\n"
                "  reload     Reload all plugins.\n"
                "  quit       Quit the program.\n");
      for (const auto& plugin : plugins) { plugin.printHelp(); }
    } else if (input == "reload") {
      for (auto& plugin : plugins) { plugin.reload(); }
    } else if (auto found = std::ranges::find_if(
                   plugins, [&input](const auto& plugin) {
                       return plugin.tryHandleInput(input); });
               found != plugins.end())
    {
      // nothing to do, already handled.

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
