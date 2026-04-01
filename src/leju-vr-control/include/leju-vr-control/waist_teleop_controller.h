#pragma once

#include <lejusdk-vr/data_types.h>

namespace leju {
namespace vr_control {

struct WaistTeleopConfig {
  bool enabled = false;
  double yaw_max_abs = 0.52;
  double deadzone = 0.1;
};

struct WaistTeleopState {
  bool active = false;
  double target_yaw = 0.0;
};

class WaistTeleopController {
 public:
  explicit WaistTeleopController(const WaistTeleopConfig& config);

  WaistTeleopState update(const leju::vr::QuestJoystickData& joy);
  leju::vr::JointTrajectoryPoint buildCommand() const;

  bool isActive() const;
  bool shouldBlockWalking() const;
  void reset();

 private:
  WaistTeleopConfig config_;
  WaistTeleopState state_;
};

}  // namespace vr_control
}  // namespace leju
