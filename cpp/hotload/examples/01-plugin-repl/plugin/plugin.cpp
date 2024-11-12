#include "PluginFunctions.hpp"

#include <fmt/format.h>

#include <cstdio>
#include <random>
#include <string_view>

using State = PluginFunctions::State;

namespace {

State pluginInit()           { std::puts("- TestPlugin: pluginInit()"); return nullptr; }
void pluginPreReload(State)  { std::puts("- TestPlugin: pluginPreReload()"); }
void pluginPostReload(State) { std::puts("- TestPlugin: pluginPostReload()"); }
void pluginDestroy(State)    { std::puts("- TestPlugin: pluginDestroy()"); }
void printHelp(State) {
  std::puts(
      "TestPlugin:\n"
      "  test          Print a test message.\n"
      "  compliment    Given a name, provide a compliment.\n"
      "  dice          Roll two 'fair' dice.\n"
  );
}
bool tryHandleInput(State, const std::string_view input) {
  std::puts("- TestPlugin: tryHandleInput()");
  if (input == "test") {
    std::puts("    > Printing a test message.");
    return true;
  }
  if (input.starts_with("compliment") && input.size() > 11) {
    fmt::print("    > {}, you look good!\n", input.substr(11));
    return true;
  }
  if (input == "dice") {
    static std::mt19937 gen;
    std::uniform_int_distribution dist{1, 6};
    fmt::print("    > {}, {}\n", dist(gen), dist(gen));
    return true;
  }
  return false;
}

}

__attribute__((constructor)) void handleDlOpen() {
  std::puts("TestPlugin (constructed)");
}

__attribute__((destructor)) void handleDlClose() {
  std::puts("TestPlugin (destructed)");
}

extern "C" {

auto getPluginFunctions() -> PluginFunctions {
  return PluginFunctions{
    .pluginInit = &pluginInit,
    .pluginPreReload = &pluginPreReload,
    .pluginPostReload = &pluginPostReload,
    .pluginDestroy = &pluginDestroy,
    .printHelp = &printHelp,
    .getCommands = nullptr,
    .tryHandleInput = &tryHandleInput,
  };
}

}
