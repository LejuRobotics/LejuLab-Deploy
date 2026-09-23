/**
 * @file controller_registry.cpp
 * @brief 控制器注册表实现
 */

#include "leju-rl-controller/controllers/controller_registry.h"

#include "leju-rl-controller/rl_log.h"
#include "lejusdk-utils/robot_version.hpp"

namespace leju {

bool ControllerRegistry::registerType(const std::string& type,
                                       ControllerCreator creator) {
  if (type.empty() || !creator) {
    RL_LOGW("ControllerRegistry: attempt to register invalid type '%s'",
            type.c_str());
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  auto [it, inserted] = creators_.emplace(type, std::move(creator));
  if (!inserted) {
    RL_LOGW("ControllerRegistry: type '%s' already registered, skipping",
            type.c_str());
    return false;
  }

  RL_LOGI("ControllerRegistry: registered controller type '%s'", type.c_str());
  return true;
}

std::unique_ptr<ControllerBase> ControllerRegistry::create(
    const std::string& type,
    const RobotVersion& version,
    const std::string& name) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = creators_.find(type);
  if (it == creators_.end()) {
    RL_LOGW("ControllerRegistry: unknown controller type '%s'", type.c_str());
    return nullptr;
  }

  try {
    return it->second(version, name);
  } catch (const std::exception& e) {
    RL_LOGE(
        "ControllerRegistry: failed to create controller '%s' of type '%s': %s",
        name.c_str(), type.c_str(), e.what());
    return nullptr;
  }
}

bool ControllerRegistry::isRegistered(const std::string& type) {
  std::lock_guard<std::mutex> lock(mutex_);
  return creators_.count(type) > 0;
}

std::vector<std::string> ControllerRegistry::getRegisteredTypes() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> types;
  types.reserve(creators_.size());

  for (const auto& [type, _] : creators_) {
    types.push_back(type);
  }
  return types;
}

void ControllerRegistry::unregisterType(const std::string& type) {
  std::lock_guard<std::mutex> lock(mutex_);
  creators_.erase(type);
  RL_LOGI("ControllerRegistry: unregistered controller type '%s'",
          type.c_str());
}

void ControllerRegistry::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  creators_.clear();
  RL_LOGI("ControllerRegistry: cleared all registrations");
}

}  // namespace leju
