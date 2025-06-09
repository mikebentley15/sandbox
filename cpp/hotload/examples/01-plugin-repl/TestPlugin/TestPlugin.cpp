#include "app/PluginFunctions.hpp"

#include <cstdio>
#include <iostream>
#include <random>
#include <string_view>

namespace {

PluginState pluginInit() {
  // std::puts("- TestPlugin: pluginInit()");
  return nullptr;
}

void pluginPreReload(PluginState) {
  // std::puts("- TestPlugin: pluginPreReload()");
}

void pluginPostReload(PluginState) {
  // std::puts("- TestPlugin: pluginPostReload()");
}

void pluginDestroy(PluginState) {
  // std::puts("- TestPlugin: pluginDestroy()");
}

void printHelp(PluginState) {
  std::puts("TestPlugin:\n"
            "  test          Print a test message.\n"
            "  compliment    Given a name, provide a compliment.\n"
            "  dice          Roll two 'fair' dice.\n");
}
bool tryHandleInput(PluginState, const std::string_view input) {
  // std::puts("- TestPlugin: tryHandleInput()");
  if (input == "test") {
    std::puts("    > Printing a test message.");
    return true;
  }
  if (input.starts_with("compliment") && input.size() > 11) {
    std::cout << "    > " << input.substr(11) << ", you look good!\n";
    return true;
  }
  if (input == "dice") {
    static std::mt19937 gen;
    std::uniform_int_distribution dist{1, 6};
    std::cout << "    > " << dist(gen) << ", " << dist(gen) << "\n";
    return true;
  }
  return false;
}

} // namespace

__attribute__((constructor)) void handleDlOpen() {
  std::puts("TestPlugin (constructed)");
}

__attribute__((destructor)) void handleDlClose() {
  std::puts("TestPlugin (destructed)");
}

extern "C" {

auto getPluginFunctions(PluginFunctions *const outFunctions) -> void {
  if (nullptr == outFunctions) {
    std::fputs("Error: cannot populate plugin functions to a null pointer",
               stderr);
    return;
  }

  *outFunctions = PluginFunctions{
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
