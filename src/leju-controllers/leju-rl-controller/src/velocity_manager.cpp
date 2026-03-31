#include "leju-rl-controller/velocity_manager.h"

#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "leju-rl-controller/rl_log.h"
#include "leju-rl-controller/utils/uri_path_resolver.h"
#include "lejusdk-utils/time_utils.hpp"

namespace leju {

namespace {

VelocityManagerConfig loadVelocityManagerConfig(const std::string& controller_manager_config_path) {
  VelocityManagerConfig config;

  YAML::Node manager_root = YAML::LoadFile(controller_manager_config_path);
  const std::string default_controller =
      manager_root["default_controller"] ? manager_root["default_controller"].as<std::string>() : "";

  YAML::Node selected_controller;
  if (manager_root["controllers"]) {
    for (const auto& controller : manager_root["controllers"]) {
      YAML::Node controller_node = controller;
      const bool enabled =
          !controller_node["enabled"] || controller_node["enabled"].as<bool>();
      if (!enabled) {
        continue;
      }
      const std::string name = controller_node["name"].as<std::string>();
      if ((default_controller.empty() && !selected_controller) || name == default_controller) {
        selected_controller = controller_node;
      }
      if (name == default_controller) {
        break;
      }
    }
  }

  if (!selected_controller) {
    throw std::runtime_error("No enabled controller found for velocity_input_manager");
  }

  // 解析控制器配置路径（支持 URI 和绝对/相对路径）
  const std::filesystem::path manager_path(controller_manager_config_path);
  const std::string config_path_str = selected_controller["config"].as<std::string>();
  std::string controller_config_path;
  try {
    controller_config_path = UriPathResolver::resolve(config_path_str, manager_path.parent_path().string());
  } catch (const UriResolveError& e) {
    throw std::runtime_error(std::string("Failed to resolve controller config path: ") + e.what());
  }

  YAML::Node controller_root = YAML::LoadFile(controller_config_path);
  YAML::Node cfg = controller_root["HumanoidRobotCfg"]["velocity_input_manager"];
  if (!cfg) {
    return config;
  }

  if (cfg["timeout_sec"]) {
    config.timeout_sec = cfg["timeout_sec"].as<double>();
  }
  if (cfg["priorities"]) {
    const YAML::Node priorities = cfg["priorities"];
    if (priorities["vr_cmd_vel"]) {
      config.vr_priority = priorities["vr_cmd_vel"].as<int>();
    }
    if (priorities["xbox_joy"]) {
      config.xbox_priority = priorities["xbox_joy"].as<int>();
    }
  }
  return config;
}

}  // namespace

bool VelocityManager::initialize(const VelocityManagerConfig& config) {
  RobotVersion version = RobotVersion::from_env();
  std::unique_ptr<vr::VRBaseAPI> vr_api;
  if (IS_KUAVO(version)) {
    vr_api = std::make_unique<vr::KuavoVRAPI>(version);
  } else if (IS_ROBAN(version)) {
    vr_api = std::make_unique<vr::RobanVRAPI>(version);
  } else {
    RL_LOGE("Unknown robot version, failed to initialize VelocityManager VR API");
    return false;
  }

  if (!vr_api->initialize()) {
    RL_LOGE("Failed to initialize VR API in VelocityManager");
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  vr_state_ = VelocitySourceState{};
  xbox_state_ = VelocitySourceState{};
  vr_api_ = std::move(vr_api);
  initialized_ = true;
  return true;
}

bool VelocityManager::initializeFromControllerManagerConfig(
    const std::string& controller_manager_config_path) {
  try {
    return initialize(loadVelocityManagerConfig(controller_manager_config_path));
  } catch (const std::exception& e) {
    RL_LOGE("Failed to load velocity manager config from %s: %s",
            controller_manager_config_path.c_str(), e.what());
    return false;
  }
}

void VelocityManager::onJoyData(const JoyData& joy) {
  VelocityCommand cmd;
  cmd.linear_x = -joy.axes.left_y;
  cmd.linear_y = -joy.axes.left_x;
  cmd.angular_z = -joy.axes.right_x;
  updateSource(VelocitySource::kXboxJoy, cmd, std::chrono::steady_clock::now());
}

void VelocityManager::onVrCmdVel(const vr::VrVelocityCmd& cmd) {
  VelocityCommand vel;
  vel.linear_x = cmd.linear_x;
  vel.linear_y = cmd.linear_y;
  vel.angular_z = cmd.angular_z;
  updateSource(VelocitySource::kVrCmdVel, vel, std::chrono::steady_clock::now());
}

VelocityCommand VelocityManager::resolve(std::chrono::steady_clock::time_point now) const {
  std::lock_guard<std::mutex> lock(mutex_);

  const bool vr_active = isActive(vr_state_, now);
  const bool xbox_active = isActive(xbox_state_, now);
  if (!vr_active && !xbox_active) {
    VelocityCommand zero;
    zero.setZero();
    return zero;
  }

  if (vr_active && !xbox_active) {
    return vr_state_.cmd;
  }
  if (!vr_active && xbox_active) {
    return xbox_state_.cmd;
  }

  return (config_.vr_priority <= config_.xbox_priority) ? vr_state_.cmd : xbox_state_.cmd;
}

std::optional<VelocitySource> VelocityManager::resolveSource(
    std::chrono::steady_clock::time_point now) const {
  std::lock_guard<std::mutex> lock(mutex_);

  const bool vr_active = isActive(vr_state_, now);
  const bool xbox_active = isActive(xbox_state_, now);
  if (!vr_active && !xbox_active) {
    return std::nullopt;
  }
  if (vr_active && !xbox_active) {
    return VelocitySource::kVrCmdVel;
  }
  if (!vr_active && xbox_active) {
    return VelocitySource::kXboxJoy;
  }
  return (config_.vr_priority <= config_.xbox_priority) ? VelocitySource::kVrCmdVel
                                                        : VelocitySource::kXboxJoy;
}

void VelocityManager::publishResolvedCmdVel() {
  if (!initialized_ || !vr_api_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const VelocityCommand resolved = resolve(now);
  vr_api_->publishVelocityCmd(toVelocityCmd(resolved));
}

void VelocityManager::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  vr_state_ = VelocitySourceState{};
  xbox_state_ = VelocitySourceState{};
}

void VelocityManager::updateSource(VelocitySource source, const VelocityCommand& cmd,
                                        std::chrono::steady_clock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);
  VelocitySourceState& state =
      (source == VelocitySource::kVrCmdVel) ? vr_state_ : xbox_state_;
  state.cmd = cmd;
  state.last_update_time = now;
  state.ever_received = true;
}

bool VelocityManager::isActive(const VelocitySourceState& state,
                                    std::chrono::steady_clock::time_point now) const {
  if (!state.ever_received) {
    return false;
  }
  const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      now - state.last_update_time);
  return elapsed.count() <= config_.timeout_sec;
}

vr::VelocityCmd VelocityManager::toVelocityCmd(const VelocityCommand& cmd) const {
  vr::VelocityCmd out;
  out.linear_x = cmd.linear_x;
  out.linear_y = cmd.linear_y;
  out.angular_z = cmd.angular_z;
  out.timestamp = common::GetSteadyTimestampNs() * 1e-9;
  return out;
}

}  // namespace leju
