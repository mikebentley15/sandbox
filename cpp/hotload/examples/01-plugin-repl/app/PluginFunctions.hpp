#pragma once

#include <string>
#include <string_view>
#include <vector>

inline constexpr auto getPluginFunctionsName = "getPluginFunctions";

class PluginFunctions;
using GetPluginFunctionsFunc = PluginFunctions();

/// A single structure of raw function pointers.
/// 
/// A single plugin getter function returns all of the other plugin functions.
struct PluginFunctions {
  /// Internal plugin global state, can store anything.
  using State = void*;

  //
  // Functions for Global State Handling
  //

  /// Initialize the plugin's global state.
  using PluginInitFunc = State();

  /// Prepare the plugin to be reloaded (maybe unloads some of the state)
  using PluginPreReloadFunc = void(State);

  /// Initialize the plugin after reloading (restores State to be functional)
  using PluginPostReloadFunc = void(State);

  /// Called before exiting the application.
  using PluginDestroyFunc = void(State);

  //
  // Functions for working with the repl
  //

  /// Print help documentation for this plugin
  using PrintHelpFunc = void(State);

  /// Return the commands this plugin supports.  Just for initial command completion.
  using GetCommandsFunc = std::vector<std::string>(State);

  /// Handle the input string from the user.
  /// @returns true if the string was handled and false otherwise.  If false is
  ///   returned, we will try the next plugin (if any).  If no plugin can handle
  ///   the command, then an error is printed.
  using TryHandleInputFunc = bool(State, std::string_view);

  // Actual member variables, raw function pointers.
  // Unimplemented functions can just kept as nullptr.
  PluginInitFunc* pluginInit{nullptr};
  PluginPreReloadFunc* pluginPreReload{nullptr};
  PluginPostReloadFunc* pluginPostReload{nullptr};
  PluginDestroyFunc* pluginDestroy{nullptr};
  PrintHelpFunc* printHelp{nullptr};
  GetCommandsFunc* getCommands{nullptr};
  TryHandleInputFunc* tryHandleInput{nullptr};
};
