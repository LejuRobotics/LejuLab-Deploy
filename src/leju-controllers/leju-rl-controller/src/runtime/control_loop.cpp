/**
 * @file control_loop.cpp
 * @brief ControlLoop 实现
 */

#include "leju-rl-controller/runtime/control_loop.h"

#include "leju-rl-controller/controllers/controller_manager.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/input/external_interface.h"
#include "leju-rl-controller/runtime/lifecycle.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/robot_data.h"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-lowlevel/leju_sdk.h"

#include <algorithm>
#include <thread>

namespace leju {
namespace runtime {

// ============================================================================
// 构造函数
// ============================================================================

ControlLoop::ControlLoop(RobotData& robot_data,
                         runtime::TriggerBuffer& trigger_buffer,
                         const std::vector<InputSource*>& input_sources,
                         runtime::Lifecycle& lifecycle,
                         ControllerManager& controller_manager)
    : robot_data_(robot_data),
      trigger_buffer_(trigger_buffer),
      input_sources_(input_sources.begin(), input_sources.end()),
      lifecycle_(lifecycle),
      controller_manager_(controller_manager) {
  // 输入源按优先级排序（数值越小优先级越高）
  std::sort(input_sources_.begin(), input_sources_.end(),
            [](InputSource* a, InputSource* b) {
              return static_cast<int>(a->getPriority()) < static_cast<int>(b->getPriority());
            });
}

// ============================================================================
// 主循环
// ============================================================================

void ControlLoop::run() {
  running_.store(true);

  RL_LOGI("ControlLoop started, using controller's frequency");

  while (running_.load()) {
    current_tick_start_ = std::chrono::steady_clock::now();

    tick();

    // 休眠到下一周期 - 委托给 ControllerManager 处理变频
    sleepUntilNextPeriod(current_tick_start_);
  }

  RL_LOGI("ControlLoop stopped");
}

void ControlLoop::stop() {
  running_.store(false);
  RL_LOGI("ControlLoop stop requested");
}

// ============================================================================
// Tick - 单拍执行
// ============================================================================

void ControlLoop::tick() {
  // A. 获取最新状态
  RobotState state;
  ImuData imu_state;
  if (!robot_data_.getRobotState(state) || !robot_data_.getImuData(imu_state)) {
    // 无法获取状态或IMU，跳过本周期
    return;
  }

  // B. 从 TriggerBuffer Drain 所有触发器（取出并清空）
  auto triggers = trigger_buffer_.drainAll();
  if (!triggers.empty()) {
    RL_LOGD("ControlLoop: Drained %zu triggers", triggers.size());
  }

  // C. 检查 ready 状态
  bool ready = robot_data_.isDataReady() && robot_data_.isHardwareReady();

  // D. 更新 Lifecycle（Lifecycle 只关心 Start/Quit，忽略其他 trigger）
  lifecycle_.update(ready, triggers);

  // E. 检查是否需要退出
  if (lifecycle_.shouldExit()) {
    stop();
    return;
  }

  // F. 未 Running 时直接返回
  if (!lifecycle_.isRunning()) {
    return;
  }

  // G. Running 状态：调用 ControlLogic 处理策略决策
  // ControlLogic 只处理它关心的 trigger 类型，忽略其他

  double now = std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
                   .count();

  // H. 从所有输入源获取命令快照并按优先级合并
  CommandBuffer::Snapshot merged_cmd;
  merged_cmd.cmd_vel = mergeAllCmdVel();
  merged_cmd.arm_target = mergeArmTarget();
  merged_cmd.head_target = mergeHeadTarget();

  // J. 调用 ControlLogic 处理策略决策
  control_logic_.tick(state, triggers, lifecycle_, controller_manager_, merged_cmd, now);

  // K. 调用 ControllerManager 更新（封装 active_controller）
  RobotCmd cmd = controller_manager_.update(state, imu_state, merged_cmd);

  // L. 发布命令
  publishRobotCmd(cmd);
}

// ============================================================================
// 时序控制
// ============================================================================

void ControlLoop::sleepUntilNextPeriod(std::chrono::steady_clock::time_point cycle_start) {
  // 委托给 ControllerManager，由它调用当前控制器的 waitNextCycle()
  // 实现不同控制器的变频控制
  controller_manager_.waitNextCycle(cycle_start);
}

// ============================================================================
// 命令发布
// ============================================================================

void ControlLoop::publishRobotCmd(const RobotCmd& cmd) {
  if (!cmd.isValid()) {
    RL_LOGW("Invalid RobotCmd, skip publishing");
    return;
  }

  GlobalRobot::getInstance().publishRobotCmd(cmd);
}

// ============================================================================
// 命令合并
// ============================================================================

std::optional<ExternalJointTarget> ControlLoop::mergeArmTarget() const {
  // 按优先级遍历所有输入源（已排序，高优先级在前）
  for (const auto* source : input_sources_) {
    if (!source) continue;

    auto snapshot = source->getSnapshot();
    if (snapshot.arm_target.has_value()) {
      return snapshot.arm_target;
    }
  }
  return std::nullopt;
}

std::optional<ExternalJointTarget> ControlLoop::mergeHeadTarget() const {
  // 按优先级遍历所有输入源（已排序，高优先级在前）
  for (const auto* source : input_sources_) {
    if (!source) continue;

    auto snapshot = source->getSnapshot();
    if (snapshot.head_target.has_value()) {
      return snapshot.head_target;
    }
  }
  return std::nullopt;
}

MotionCommand ControlLoop::mergeAllCmdVel() const {
  // 按优先级遍历所有输入源（已排序，高优先级在前）
  for (const auto* source : input_sources_) {
    if (!source) continue;

    auto snapshot = source->getSnapshot();
    // 输入源必须有效，且速度非零（不在死区内）
    if (snapshot.cmd_vel.valid &&
        !snapshot.cmd_vel.isNearZero(VelocityDeadzone::kLinearX,
                                      VelocityDeadzone::kLinearY,
                                      VelocityDeadzone::kAngularZ)) {
      return snapshot.cmd_vel;
    }
  }

  // 所有输入源都在死区内或无效，返回零速度（但有效）
  MotionCommand zero_cmd;
  zero_cmd.setZero();
  zero_cmd.valid = true;
  return zero_cmd;
}

}  // namespace runtime
}  // namespace leju
