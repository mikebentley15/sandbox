#pragma once

#include <string>
#include <string_view>
#include <vector>

inline constexpr auto getPluginFunctionsName = "getPluginFunctions";

extern "C" struct PluginFunctions;
using GetPluginFunctionsFunc = void(PluginFunctions *);

/// Internal plugin global state, can store anything.
using PluginState = void *;

//
// Functions for Global State Handling
//

/// Initialize the plugin's global state.
using PluginInitFunc = PluginState();

/// Prepare the plugin to be reloaded (maybe unloads some of the state)
using PluginPreReloadFunc = void(PluginState);

/// Initialize the plugin after reloading (restores State to be functional)
using PluginPostReloadFunc = void(PluginState);

/// Called before exiting the application.
using PluginDestroyFunc = void(PluginState);

//
// Functions for working with the repl
//

/// Print help documentation for this plugin
using PrintHelpFunc = void(PluginState);

/// Return the commands this plugin supports.  Just for initial command
/// completion.
using GetCommandsFunc = std::vector<std::string>(PluginState);

/// Handle the input string from the user.
/// @returns true if the string was handled and false otherwise.  If false is
///   returned, we will try the next plugin (if any).  If no plugin can handle
///   the command, then an error is printed.
using TryHandleInputFunc = bool(PluginState, std::string_view);

/// A single structure of raw function pointers.
///
/// A single plugin getter function returns all of the other plugin functions.
extern "C" {
struct PluginFunctions {
  // Actual member variables, raw function pointers.
  // Unimplemented functions can just kept as nullptr.
  PluginInitFunc *pluginInit{nullptr};
  PluginPreReloadFunc *pluginPreReload{nullptr};
  PluginPostReloadFunc *pluginPostReload{nullptr};
  PluginDestroyFunc *pluginDestroy{nullptr};
  PrintHelpFunc *printHelp{nullptr};
  GetCommandsFunc *getCommands{nullptr};
  TryHandleInputFunc *tryHandleInput{nullptr};
};
}
