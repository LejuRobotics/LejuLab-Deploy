/**
 * @file control_loop.cpp
 * @brief ControlLoop 实现
 */

#include "leju-rl-controller/runtime/control_loop.h"

#include "leju-rl-controller/controllers/controller_manager.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/fall_stand_coordinator.h"
#include "leju-rl-controller/runtime/input/external_interface.h"
#include "leju-rl-controller/runtime/lifecycle.h"
#include "leju-rl-controller/runtime/transport_mode_coordinator.h"
#include "leju-rl-controller/runtime/transport_fall_stand_scheduler.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/robot_data.h"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-lowlevel/leju_sdk.h"

#include <algorithm>
#include <cmath>
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
  // 已激活的退出下蹲仅依赖 steady clock，避免状态帧中断时无法到期退出。
  const double now = std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
                   .count();
  if (quit_squat_active_) {
    std::vector<ActionTrigger> timeout_triggers;
    maybeExpireQuitSquat(now, timeout_triggers);
    if (!timeout_triggers.empty()) {
      lifecycle_.update(false, timeout_triggers);
      last_lifecycle_state_ = lifecycle_.state();
      stop();
      return;
    }
  }

  // A. 获取最新状态
  RobotState state;
  ImuData imu_state;
  if (!robot_data_.getSynchronizedData(state, imu_state)) {
    // 无法获取状态或IMU，跳过本周期
    return;
  }

  // B. 从 TriggerBuffer Drain 所有触发器（取出并清空）
  auto triggers = trigger_buffer_.drainAll();
  if (!triggers.empty()) {
    RL_LOGD("ControlLoop: Drained %zu triggers", triggers.size());
    consumeQuitSquatTriggers(triggers, now);
  }

  // C. 检查 ready 状态
  bool ready = robot_data_.isDataReady() && robot_data_.isHardwareReady();

  // D. 更新 Lifecycle（Lifecycle 只关心 Start/Quit，忽略其他 trigger）
  lifecycle_.update(ready, triggers);

  // E. 检查是否需要退出
  if (lifecycle_.shouldExit()) {
    last_lifecycle_state_ = lifecycle_.state();
    stop();
    return;
  }

  // E2. 运控模块 tick（仅注入时参与，Roban 2.2）
  if (transport_coordinator_) {
    transport_coordinator_->tick(controller_manager_, lifecycle_.isNormalRunning(), now);
  }
  if (fall_stand_coordinator_) {
    fall_stand_coordinator_->tick(controller_manager_, lifecycle_.allowsControlOutput(), now);
  }
  if (transport_fall_stand_scheduler_) {
    transport_fall_stand_scheduler_->tick(imu_state, now);
  }

  const bool just_entered_running =
      lifecycle_.isNormalRunning() &&
      last_lifecycle_state_ != LifecycleState::kRunning;
  if (just_entered_running) {
    clearAllInputCmdVel();
  }

  // F. 未放行控制输出时直接返回
  if (!lifecycle_.allowsControlOutput()) {
    last_lifecycle_state_ = lifecycle_.state();
    return;
  }

  // G. Running 状态：调用 ControlLogic 处理策略决策
  // ControlLogic 只处理它关心的 trigger 类型，忽略其他

  // H. 从所有输入源获取命令快照并按优先级合并
  CommandBuffer::Snapshot merged_cmd;
  merged_cmd.cmd_vel = mergeAllCmdVel();
  merged_cmd.arm_target = mergeArmTarget();
  merged_cmd.head_target = mergeHeadTarget();
  merged_cmd.waist_target = mergeWaistTarget();
  merged_cmd.hand_target = mergeHandTarget();

  // 搬运/倒地起身期间源头清零 cmd_vel（对齐研杨 joy 早返），从按下那一刻起不再响应行走输入
  if ((transport_coordinator_ && transport_coordinator_->isInputGating()) ||
      controller_manager_.getCurrentControllerName() == "mimic_fall_stand") {
    merged_cmd.cmd_vel.linear_x = 0.0;
    merged_cmd.cmd_vel.linear_y = 0.0;
    merged_cmd.cmd_vel.angular_z = 0.0;
    merged_cmd.cmd_vel.valid = false;
  }

  // 退出下蹲覆写：仅支持 cmd_stance 且未进入安全门控时执行；
  // 其他模式保持零速度，等待同一个 deadline 后退出，避免把下蹲高度误作角速度。
  if (quit_squat_active_) {
    const bool transport_gated =
        transport_coordinator_ && transport_coordinator_->isInputGating();
    if (!transport_gated &&
        controller_manager_.getCurrentControllerName() == "amp") {
      applyQuitSquatOverride(merged_cmd.cmd_vel);
    } else {
      merged_cmd.cmd_vel.linear_x = 0.0;
      merged_cmd.cmd_vel.linear_y = 0.0;
      merged_cmd.cmd_vel.angular_z = 0.0;
      merged_cmd.cmd_vel.valid = false;
    }
  }
  // J. 调用 ControlLogic 处理策略决策
  if (lifecycle_.isNormalRunning()) {
    control_logic_.tick(state, imu_state, triggers, lifecycle_, controller_manager_, merged_cmd, now);
  }

  // K. 调用 ControllerManager 更新（封装 active_controller）
  RobotCmd cmd = controller_manager_.update(state, imu_state, merged_cmd);

  // L. 发布命令
  publishRobotCmd(cmd);

  last_lifecycle_state_ = lifecycle_.state();
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

VersionedJointTarget ControlLoop::mergeArmTarget() const {
  // 按优先级遍历所有输入源（已排序，高优先级在前）
  for (const auto* source : input_sources_) {
    if (!source) continue;

    auto snapshot = source->getSnapshot();
    if (snapshot.arm_target.hasValue()) {
      return snapshot.arm_target;  // 含 seq 一起返回
    }
  }
  return {};
}

VersionedJointTarget ControlLoop::mergeHeadTarget() const {
  // 按优先级遍历所有输入源（已排序，高优先级在前）
  for (const auto* source : input_sources_) {
    if (!source) continue;

    auto snapshot = source->getSnapshot();
    if (snapshot.head_target.hasValue()) {
      return snapshot.head_target;
    }
  }
  return {};
}


VersionedJointTarget ControlLoop::mergeWaistTarget() const {
  for (const auto* source : input_sources_) {
    if (!source) continue;

    auto snapshot = source->getSnapshot();
    if (snapshot.waist_target.hasValue()) {
      return snapshot.waist_target;
    }
  }
  return {};
}

VersionedHandTarget ControlLoop::mergeHandTarget() const {
  for (const auto* source : input_sources_) {
    if (!source) continue;

    auto snapshot = source->getSnapshot();
    if (snapshot.hand_target.hasValue()) {
      return snapshot.hand_target;
    }
  }
  return {};
}

MotionCommand ControlLoop::mergeAllCmdVel() const {
  // 按优先级遍历所有输入源（已排序，高优先级在前）
  for (const auto* source : input_sources_) {
    if (!source) continue;

    auto snapshot = source->getSnapshot();
    // 输入源必须有效；行走速度非零，或 posture 命令（cmd_stance_valid）均可接管
    if (snapshot.cmd_vel.valid &&
        (snapshot.cmd_vel.cmd_stance_valid ||
         !snapshot.cmd_vel.isNearZero(VelocityDeadzone::kLinearX,
                                      VelocityDeadzone::kLinearY,
                                      VelocityDeadzone::kAngularZ))) {
      return snapshot.cmd_vel;
    }
  }

  // 所有输入源都在死区内或无效，返回零速度（但有效）
  MotionCommand zero_cmd;
  zero_cmd.setZero();
  zero_cmd.valid = true;
  return zero_cmd;
}

void ControlLoop::clearAllInputCmdVel() {
  for (auto* source : input_sources_) {
    if (!source) continue;
    source->clearCmdVel();
  }
  RL_LOGI("ControlLoop: Cleared cached cmd_vel on all input sources");
}

void ControlLoop::consumeQuitSquatTriggers(std::vector<ActionTrigger>& triggers,
                                           double now) {
  // 同批已有 Quit：Quit 优先（直接退出），忽略 QuitSquat、不进入下蹲序列。
  bool has_quit = false;
  for (const auto& t : triggers) {
    if (t.type == ActionType::Quit) {
      has_quit = true;
      break;
    }
  }

  // 非法 QuitSquat → 降级为立即 Quit（fail-safe，避免 runtime 不退、等 monitor 8s 强杀）。
  // 用标记位收集，循环结束后统一 push，避免迭代中 push_back 使迭代器失效。
  bool degraded_to_quit = false;

  for (auto it = triggers.begin(); it != triggers.end();) {
    if (it->type != ActionType::QuitSquat) {
      ++it;
      continue;
    }

    if (has_quit) {
      // Quit 已存在：消费掉 QuitSquat，保留 Quit 走退出路径
      it = triggers.erase(it);
      continue;
    }

    // 参数校验：类型错误 / NaN/Inf / 非下蹲高度 / 非法时长时忽略，不崩溃。
    // NaN 时比较恒为 false，故先 isfinite 拦截。
    const auto* args = dynamic_cast<const QuitSquatArgs*>(it->args.get());
    if (!args || !std::isfinite(args->duration_sec) || !std::isfinite(args->squat_height) ||
        args->duration_sec <= 0.0 || args->duration_sec > 60.0 ||
        args->squat_height >= 0.0 || args->squat_height < -1.0) {
      RL_LOGW("ControlLoop: QuitSquat invalid params ignored, degraded to immediate Quit");
      degraded_to_quit = true;
      it = triggers.erase(it);
      continue;
    }

    if (!quit_squat_active_) {
      // 能否下蹲：运行态 && 非搬运门控 && AMP（唯一认 cmd_stance 协议的控制器）。
      // 只有能下蹲才进入「下蹲 1.5s 再退出」序列；其余（未运行/舞蹈/倒地/搬运门控）
      // 一律立即退出，不空等 1.5s。
      const bool transport_gated =
          transport_coordinator_ && transport_coordinator_->isInputGating();
      const bool can_squat =
          lifecycle_.allowsControlOutput() && !transport_gated &&
          controller_manager_.getCurrentControllerName() == "amp";
      if (can_squat) {
        // 激活下蹲序列，记高度与截止时间
        quit_squat_active_ = true;
        quit_squat_height_ = args->squat_height;
        quit_squat_deadline_ = now + args->duration_sec;
        RL_LOGI("ControlLoop: quit-squat sequence started (height=%.2fm, deadline=%.3fs)",
                quit_squat_height_, quit_squat_deadline_);
      } else {
        // 不能下蹲 → 立即退出（原路径），不空等
        RL_LOGI("ControlLoop: quit-squat requested but cannot squat "
                "(not running / non-AMP / gated), degrade to immediate quit");
        degraded_to_quit = true;
      }
    } else {
      // 已激活：忽略重复 QuitSquat（不刷新 deadline，防重按续期）
      RL_LOGI("ControlLoop: duplicate QuitSquat ignored during active sequence");
    }
    it = triggers.erase(it);  // 消费掉，不传给 lifecycle
  }

  if (degraded_to_quit && !has_quit) {
    triggers.push_back(MakeQuitTrigger());  // 循环结束后 push，迭代器安全
  }
}

void ControlLoop::applyQuitSquatOverride(MotionCommand& cmd_vel) const {
  cmd_vel.valid = true;
  cmd_vel.cmd_stance_valid = true;
  cmd_vel.cmd_stance_mode = 1;               // 1=站立/下蹲/弯腰
  cmd_vel.linear_x = 0.0;
  cmd_vel.linear_y = 0.0;
  cmd_vel.angular_z = quit_squat_height_;    // 负值=下蹲
}

void ControlLoop::maybeExpireQuitSquat(double now,
                                       std::vector<ActionTrigger>& triggers) {
  if (!quit_squat_active_ || now < quit_squat_deadline_) {
    return;
  }
  triggers.push_back(ActionTrigger(ActionType::Quit));
  quit_squat_active_ = false;
  RL_LOGI("ControlLoop: quit-squat deadline reached; requesting exit");
}

}  // namespace runtime
}  // namespace leju
