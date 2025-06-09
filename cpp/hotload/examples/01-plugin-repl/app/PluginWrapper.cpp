#include "PluginWrapper.hpp"

#include <dlfcn.h>

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace {

auto extractPluginFunctions(void *const handle, const std::string_view path)
    -> PluginFunctions {
  GetPluginFunctionsFunc *const getPluginFunctions =
      reinterpret_cast<GetPluginFunctionsFunc *>(
          dlsym(handle, getPluginFunctionsName));
  if (nullptr == getPluginFunctions) {
    std::ostringstream msgbuilder;
    msgbuilder << "Could not find function '" << getPluginFunctionsName
               << "' in plugin '" << path << "': " << dlerror();
    throw std::runtime_error(std::move(msgbuilder.str()));
  }
  PluginFunctions funcs;
  getPluginFunctions(&funcs);
  return funcs;
}

auto openPluginFile(const std::string_view path) -> void * {
  // auto* const handle = dlmopen(LM_ID_NEWLM, path.data(), RTLD_LAZY |
  // RTLD_LOCAL);
  auto *const handle = dlopen(path.data(), RTLD_LAZY | RTLD_LOCAL);
  if (nullptr == handle) {
    std::ostringstream msgbuilder;
    msgbuilder << "Could not load plugin at '" << path << "': " << dlerror();
    throw std::runtime_error(std::move(msgbuilder.str()));
  }
  return handle;
}

} // namespace

PluginWrapper::PluginWrapper(const std::string_view path)
    : path_{path} // TODO: make a copy of path_ before opening
      ,
      libHandle_{openPluginFile(path_)},
      plugin_{extractPluginFunctions(libHandle_, path_)},
      state_{safeCall(plugin_.pluginInit).value_or(PluginState{})} {}

PluginWrapper::~PluginWrapper() {
  if (plugin_.pluginDestroy) {
    safeVoidCall(plugin_.pluginDestroy, state_);
  }
  if (libHandle_ && dlclose(libHandle_) != 0) {
    std::cerr << "Error: Failed to close plugin '" << path_
              << "': " << dlerror() << '\n';
  }
}

// default move operations; and zero out the moved from.
PluginWrapper::PluginWrapper(PluginWrapper &&other) noexcept
    : path_{std::move(other.path_)},
      libHandle_{std::exchange(other.libHandle_, nullptr)},
      plugin_{std::exchange(other.plugin_, PluginFunctions{})},
      state_{std::exchange(other.state_, PluginState{})} {}

auto PluginWrapper::operator=(PluginWrapper &&other) noexcept
    -> PluginWrapper & {
  this->path_ = std::move(other.path_);
  this->libHandle_ = std::exchange(other.libHandle_, nullptr);
  this->plugin_ = std::exchange(other.plugin_, PluginFunctions{});
  this->state_ = std::exchange(other.state_, PluginState{});
  return *this;
}

void PluginWrapper::reload() {
  safeVoidCall(plugin_.pluginPreReload, state_);
  if (dlclose(libHandle_) != 0) {
    std::ostringstream msgbuilder;
    msgbuilder << "Failed to close plugin '" << path_ << "': " << dlerror();
    throw std::runtime_error(std::move(msgbuilder.str()));
  }
  // TODO: make a copy of path_ before opening.
  libHandle_ = openPluginFile(path_);
  plugin_ = extractPluginFunctions(libHandle_, path_);
  safeVoidCall(plugin_.pluginPostReload, state_);
}
