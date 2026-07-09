#include "leju-vr-control/waist_teleop_controller.h"

#include <cmath>

namespace leju {
namespace vr_control {

WaistTeleopController::WaistTeleopController(const WaistTeleopConfig& config) : config_(config) {}

WaistTeleopState WaistTeleopController::update(const leju::vr::QuestJoystickData& joy) {
  const bool active =
      config_.enabled && joy.left_second_button_touched && !joy.left_first_button_touched;

  state_.active = active;
  if (!active) {
    state_.target_yaw = 0.0;
    return state_;
  }

  float right_x = joy.right_x;
  if (std::abs(right_x) < config_.deadzone) right_x = 0.0f;

  state_.target_yaw = -static_cast<double>(right_x) * config_.yaw_max_abs;
  return state_;
}

leju::vr::JointTrajectoryPoint WaistTeleopController::buildCommand() const {
  leju::vr::JointTrajectoryPoint cmd;
  cmd.q = {state_.target_yaw};
  cmd.v = {0.0};
  cmd.acc = {0.0};
  return cmd;
}

bool WaistTeleopController::isActive() const { return state_.active; }

bool WaistTeleopController::shouldBlockWalking() const { return state_.active; }

void WaistTeleopController::reset() { state_ = WaistTeleopState{}; }

}  // namespace vr_control
}  // namespace leju
