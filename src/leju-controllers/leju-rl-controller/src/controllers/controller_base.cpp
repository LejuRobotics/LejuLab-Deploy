#include "leju-rl-controller/controllers/controller_base.h"
#include "leju-rl-controller/rl_log.h"

#include <thread>

namespace leju {

void ControllerBase::stop() {
  state_ = ControllerState::kStopped;
  RL_LOGI("Controller '%s' stopped", name_.c_str());
}

void ControllerBase::reset() {
  // 基类空实现，子类按需 override
}

void ControllerBase::pause() {
  if (state_ == ControllerState::kRunning) {
    state_ = ControllerState::kPaused;
    RL_LOGI("Controller '%s' paused", name_.c_str());
  } else {
    RL_LOGW("Cannot pause controller '%s': current state=%d", name_.c_str(), static_cast<int>(state_));
  }
}

void ControllerBase::resume() {
  if (state_ != ControllerState::kStopped) {
    state_ = ControllerState::kRunning;
    reset();
    RL_LOGI("Controller '%s' resumed, state reset", name_.c_str());
  } else {
    RL_LOGW("Cannot resume controller '%s': current state is STOPPED", name_.c_str());
  }
}

void ControllerBase::setVelocityCommand(const VelocityCommand& cmd) {
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  velocity_cmd_ = cmd;
}

void ControllerBase::moveToDefaultPos(const RobotState& current_state, double elapse) {
  (void)current_state;
  (void)elapse;
}

std::string ControllerBase::getName() const {
  return name_;
}

ControllerState ControllerBase::getState() const {
  return state_;
}

const array_t& ControllerBase::getDefaultJointPos() const {
  return default_joint_pos_;
}

bool ControllerBase::isActive() const {
  return state_ == ControllerState::kRunning;
}

bool ControllerBase::isPaused() const {
  return state_ == ControllerState::kPaused;
}

bool ControllerBase::isInitialized() const {
  return state_ != ControllerState::kUninitialized;
}

void ControllerBase::onJoyInput(const JoyData& joy, const JoyData::Buttons& prev_buttons) {
  (void)joy;
  (void)prev_buttons;
}

void ControllerBase::waitNextCycle(std::chrono::steady_clock::time_point cycle_start) {
  auto cycle_duration = std::chrono::duration<double>(1.0 / control_frequency_);
  auto next_cycle = cycle_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(cycle_duration);
  std::this_thread::sleep_until(next_cycle);
}

}  // namespace leju
