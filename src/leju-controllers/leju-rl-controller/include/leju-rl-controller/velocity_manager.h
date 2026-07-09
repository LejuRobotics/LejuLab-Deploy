#pragma once

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <memory>

#include "leju-rl-controller/rl/rl_controller_types.h"
#include "lejusdk-lowlevel/data_types.h"
#include "lejusdk-vr/lejusdk_vr.h"

namespace leju {

enum class VelocitySource {
  kVrCmdVel,
  kXboxJoy,
};

struct VelocitySourceState {
  VelocityCommand cmd;
  std::chrono::steady_clock::time_point last_update_time{};
  bool ever_received = false;
};

struct VelocityManagerConfig {
  double timeout_sec = 1.0;
  int vr_priority = 0;
  int xbox_priority = 1;
};

class VelocityManager {
 public:
  bool initialize(const VelocityManagerConfig& config);
  bool initializeFromControllerManagerConfig(const std::string& controller_manager_config_path);

  void onJoyData(const JoyData& joy);
  void onVrCmdVel(const vr::VrVelocityCmd& cmd);

  VelocityCommand resolve(std::chrono::steady_clock::time_point now) const;
  std::optional<VelocitySource> resolveSource(
      std::chrono::steady_clock::time_point now) const;

  void publishResolvedCmdVel();
  void reset();

 private:
  void updateSource(VelocitySource source, const VelocityCommand& cmd,
                    std::chrono::steady_clock::time_point now);
  bool isActive(const VelocitySourceState& state,
                std::chrono::steady_clock::time_point now) const;
  vr::VelocityCmd toVelocityCmd(const VelocityCommand& cmd) const;

  mutable std::mutex mutex_;
  VelocityManagerConfig config_;
  VelocitySourceState vr_state_;
  VelocitySourceState xbox_state_;
  std::unique_ptr<vr::VRBaseAPI> vr_api_;
  bool initialized_ = false;
};

}  // namespace leju
