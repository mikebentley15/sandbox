#include "PluginWrapper.hpp"

#include <fmt/format.h>

#include <dlfcn.h>

#include <string_view>
#include <stdexcept>

namespace {

auto extractPluginFunctions(void* const handle, const std::string_view path) -> PluginFunctions {
  GetPluginFunctionsFunc* const getPluginFunctions =
    reinterpret_cast<GetPluginFunctionsFunc*>(
        dlsym(handle, getPluginFunctionsName));
  if (nullptr == getPluginFunctions) {
    throw std::runtime_error(fmt::format(
          "Could not find function '{}' in plugin '{}': {}",
          getPluginFunctionsName, path, dlerror()));
  }
  return getPluginFunctions();
}

auto openPluginFile(const std::string_view path) -> void* {
  auto* const handle = dlmopen(LM_ID_NEWLM, path.data(), RTLD_LAZY | RTLD_LOCAL);
  if (nullptr == handle) {
    throw std::runtime_error(fmt::format(
          "Could not load plugin at '{}': {}", path, dlerror()));
  }
  return handle;
}

} // namespace

PluginWrapper::PluginWrapper(const std::string_view path)
  : path_{path}
  , libHandle_{openPluginFile(path_)}
  , plugin_{extractPluginFunctions(libHandle_, path_)}
  , state_{safeCall(plugin_.pluginInit).value_or(PluginFunctions::State{})}
{}

PluginWrapper::~PluginWrapper()
{ safeVoidCall(plugin_.pluginDestroy, state_); }

void PluginWrapper::reload()
{
  safeVoidCall(plugin_.pluginPreReload, state_);
  if (dlclose(libHandle_) != 0) {
    throw std::runtime_error(fmt::format(
          "Failed to close plugin '{}': {}", path_, dlerror()));
  }
  libHandle_ = openPluginFile(path_);
  plugin_ = extractPluginFunctions(libHandle_, path_);
  safeVoidCall(plugin_.pluginPostReload, state_);
}
