#pragma once

#include "PluginFunctions.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

/// Wraps a single plugin that can be reloaded.
class PluginWrapper {
public:
  /// Loads and initializes the plugin shared library.
  explicit PluginWrapper(std::string_view path);

  /// Unloads and uninitializes the plugin shared library.
  ~PluginWrapper();

  /// Disable copy
  PluginWrapper(const PluginWrapper&) = delete;
  auto operator=(const PluginWrapper&) noexcept -> PluginWrapper& = delete;

  /// Enable move
  PluginWrapper(PluginWrapper&& other) noexcept;
  auto operator=(PluginWrapper&& other) noexcept -> PluginWrapper&;

  /// Unload and reload the plugin.
  void reload();

  //
  // Plugin Functionality.
  //

  /// Print help documentation for this plugin.
  /// If not implementated, prints nothing.
  void printHelp() const { if (plugin_.printHelp) { plugin_.printHelp(state_); } }

  /// Returns the commands this plugin supports.
  /// If not implemented, returns an empty vector.
  [[nodiscard]] auto getCommands() const -> std::vector<std::string> {
    return safeCall(plugin_.getCommands, state_).value_or(std::vector<std::string>{});
  }

  /// Handle the user input.  If it's not handled by this plugin, return false.
  auto tryHandleInput(const std::string_view input) const -> bool {
    return safeCall(plugin_.tryHandleInput, state_, input).value_or(false);
  }

private:
  // template <typename ReturnType, typename ...Args>
  // static auto safeCall(ReturnType(*func)(Args...), Args&&... args)
  //   -> std::optional<ReturnType>
  template <typename Func, typename ...Args>
  static auto safeCall(Func func, Args&& ...args) 
    -> std::optional<std::invoke_result_t<Func, Args...>>
  {
    if (func) {
      return std::optional{func(std::forward<Args>(args)...)};
    }
    return std::nullopt;
  }

  template <typename Func, typename ...Args>
  static void safeVoidCall(Func func, Args&& ...args) {
    if (func) { func(std::forward<Args>(args)...); }
  }

private:
  std::string path_;
  void* libHandle_;
  PluginFunctions plugin_;
  PluginFunctions::State state_;
};
