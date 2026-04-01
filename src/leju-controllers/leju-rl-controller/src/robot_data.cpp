#include "leju-rl-controller/robot_data.h"

#include "leju-rl-controller/rl_log.h"

namespace leju {

bool RobotData::initialize() {
  if (initialized_) {
    return true;
  }

  if (!GlobalRobot::is_initialized()) {
    RL_LOGE("GlobalRobot is not initialized. Please call GlobalRobot::init_env() first.");
    return false;
  }

  // 获取全局机器人实例并注册订阅回调
  auto& robot = GlobalRobot::getInstance();

  robot.subscribeRobotState([this](const RobotStateConstPtr& state) {
    this->onRobotState(state);
  });

  robot.subscribeImuData([this](const ImuDataConstPtr& imu) {
    this->onImuData(imu);
  });

  robot.subscribeHardwareState([this](const StringDataConstPtr& hw_state) {
    this->onHardwareState(hw_state);
  });

  initialized_ = true;
  return true;
}

void RobotData::shutdown() {
  initialized_ = false;
  robot_state_valid_.store(false);
  imu_data_valid_.store(false);
  hw_state_valid_.store(false);
}

bool RobotData::getRobotState(RobotState& state) const {
  // 先检查有效性（无锁），再加锁拷贝
  if (!robot_state_valid_.load()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  state = robot_state_;
  return true;
}

bool RobotData::getImuData(ImuData& imu) const {
  // 先检查有效性（无锁），再加锁拷贝
  if (!imu_data_valid_.load()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu = imu_data_;
  return true;
}

bool RobotData::isDataReady() const {
  return hasRobotState() && hasImuData();
}

void RobotData::onRobotState(const RobotStateConstPtr& state) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  robot_state_ = *state;
  robot_state_valid_.store(true);
}

void RobotData::onImuData(const ImuDataConstPtr& imu) {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_data_ = *imu;
  imu_data_valid_.store(true);
}

void RobotData::onHardwareState(const StringDataConstPtr& hw_state) {
  std::lock_guard<std::mutex> lock(hw_state_mutex_);
  hw_state_ = String2HwState(hw_state->data);
  hw_state_valid_.store(true);
}

HardwareState RobotData::getHardwareState() const {
  if (!hw_state_valid_.load()) {
    return HardwareState::UNKNOWN;
  }
  std::lock_guard<std::mutex> lock(hw_state_mutex_);
  return hw_state_;
}

bool RobotData::isHardwareReady() const {
  return getHardwareState() == HardwareState::READY_OK;
}

}  // namespace leju
