/**
 * @file control_logic.cpp
 * @brief 控制逻辑策略层实现
 */

#include "leju-rl-controller/runtime/control_logic.h"

#include <unordered_map>
#include <unordered_set>

#include "leju-rl-controller/controllers/controller_manager.h"
#include "leju-rl-controller/rl/rl_controller_types.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-lowlevel/data_types.h"
#include "lejusdk-utils/time_utils.hpp"

namespace leju {
namespace runtime {

// ============================================================================
// 主控制逻辑入口
// ============================================================================

void ControlLogic::tick(const RobotState& state,
                        const std::vector<ActionTrigger>& triggers,
                        Lifecycle& lifecycle,
                        ControllerManager& controller_manager,
                        const CommandBuffer::Snapshot& command_snapshot,
                        double now) {
  // 1. 处理 Running 状态入口
  handleRunningEntry(state, lifecycle, controller_manager, command_snapshot, now);

  // 2. 非 Running 状态直接返回
  if (!lifecycle.isRunning()) {
    last_lifecycle_state_ = lifecycle.state();
    return;
  }

  // 3. 处理 ActionTrigger
  handleActionTriggers(triggers, lifecycle, controller_manager, command_snapshot, now);

  // 4. 处理 CommandSnapshot 中的外部控制请求
  processExternalTargets(controller_manager, command_snapshot);
  processVelocityCommand(controller_manager, command_snapshot);

  // 5. 处理自动 Controller 转移
  handleAutoTransitions(state, controller_manager, command_snapshot, now);

  // 6. 处理跌倒保护与恢复逻辑
  handleFallLogic(state, controller_manager, command_snapshot, now);

  last_lifecycle_state_ = lifecycle.state();
}

// ============================================================================
// Running 状态入口逻辑
// ============================================================================

void ControlLogic::handleRunningEntry(const RobotState& state,
                                       Lifecycle& lifecycle,
                                       ControllerManager& controller_manager,
                                       const CommandBuffer::Snapshot& command_snapshot,
                                       double now) {
  (void)state;
  (void)now;
  (void)command_snapshot;

  // 检查是否刚进入 Running 状态
  bool just_entered_running = (lifecycle.state() == LifecycleState::kRunning) &&
                               (last_lifecycle_state_ != LifecycleState::kRunning);

  if (!just_entered_running || running_entry_handled_) {
    return;
  }

  RL_LOGI("ControlLogic: Handling Running entry");
  RL_LOGI("ControlLogic: Current controller before entry: %s",
          controller_manager.getCurrentControllerName().c_str());

  // 根据机器人状态决定启动路径
  if (isFallen(state)) {
    // 倒地 -> 尝试切换到 GroundToStand 或恢复控制器
    if (controller_manager.hasController("ground_to_stand")) {
      RL_LOGI("ControlLogic: Robot is fallen, switching to ground_to_stand");
      requestSwitch(controller_manager, "ground_to_stand");
    } else if (controller_manager.hasController("recovery")) {
      RL_LOGI("ControlLogic: Robot is fallen, switching to recovery");
      requestSwitch(controller_manager, "recovery");
    } else {
      RL_LOGW("ControlLogic: Robot is fallen but no recovery controller available");
    }
  } else {
    // 正常站立 -> 使用当前默认控制器
    // 不硬编码特定控制器名称，由配置文件决定默认控制器
    RL_LOGI("ControlLogic: Robot is standing, NOT switching controller");
  }

  running_entry_handled_ = true;
}

// ============================================================================
// ActionTrigger 处理
// ============================================================================

void ControlLogic::handleActionTriggers(const std::vector<ActionTrigger>& triggers,
                                        Lifecycle& lifecycle,
                                        ControllerManager& controller_manager,
                                        const CommandBuffer::Snapshot& command_snapshot,
                                        double now) {
  (void)now;
  (void)command_snapshot;

  // 1. 先处理重复 triggers
  auto processed = processDuplicateTriggers(triggers);

  // 调试日志：打印收到的 triggers
  for (const auto& trigger : processed) {
    RL_LOGD("ControlLogic: Received trigger type=%d", static_cast<int>(trigger.type));
  }

  if (processed.empty()) {
    return;
  }

  // 2. 按优先级阶段处理
  // Phase 1: Lifecycle (Start, Quit) - 已由 Lifecycle::update() 处理，这里跳过

  // Phase 2: Controller Switch
  for (const auto& trigger : processed) {
    if (trigger.type == ActionType::SwitchController) {
      if (trigger.args) {
        auto* args = dynamic_cast<NamedArgs*>(trigger.args.get());
        if (args && !args->name.empty()) {
          RL_LOGI("ControlLogic: Processing switch_controller to '%s'", args->name.c_str());
          requestSwitch(controller_manager, args->name);
        }
      }
    }
  }

  // Phase 3: Mode Change (Arm, Waist)
  for (const auto& trigger : processed) {
    if (trigger.type == ActionType::SetArmMode) {
      RL_LOGI("ControlLogic: Handling SetArmMode trigger");
      if (trigger.args) {
        auto* args = dynamic_cast<NamedArgs*>(trigger.args.get());
        if (args && !args->name.empty()) {
          RL_LOGI("ControlLogic: Processing set_arm_mode to '%s'", args->name.c_str());
          auto mode_opt = StringToArmControlMode(args->name);
          if (mode_opt.has_value()) {
            std::string message;
            bool success = controller_manager.setArmMode(mode_opt.value(), message);
            RL_LOGI("ControlLogic: setArmMode result=%s, msg=%s", success ? "true" : "false", message.c_str());
          } else {
            RL_LOGW("ControlLogic: Unknown arm mode '%s'", args->name.c_str());
          }
        } else {
          RL_LOGW("ControlLogic: SetArmMode trigger has no args or empty name");
        }
      } else {
        RL_LOGW("ControlLogic: SetArmMode trigger has no args");
      }
    } else if (trigger.type == ActionType::SetWaistMode) {
      if (trigger.args) {
        auto* args = dynamic_cast<NamedArgs*>(trigger.args.get());
        if (args && !args->name.empty()) {
          RL_LOGI("ControlLogic: Processing set_waist_mode to '%s'", args->name.c_str());
          auto mode_opt = StringToWaistControlMode(args->name);
          if (mode_opt.has_value()) {
            std::string message;
            controller_manager.setWaistMode(mode_opt.value(), message);
          } else {
            RL_LOGW("ControlLogic: Unknown waist mode '%s'", args->name.c_str());
          }
        }
      }
    }
  }

  // Phase 4: Motion
  for (const auto& trigger : processed) {
    if (trigger.type == ActionType::MotionCommand) {
      if (trigger.args) {
        auto* args = dynamic_cast<MotionCommandArgs*>(trigger.args.get());
        if (args && args->op == MotionCommandArgs::Operation::Start) {
          if (!args->motion_name.empty()) {
            RL_LOGI("ControlLogic: Processing motion_start '%s'", args->motion_name.c_str());
            controller_manager.startMotion(args->motion_name);
          } else {
            controller_manager.startMotion();
          }
        }
      }
    }
  }
}

// ============================================================================
// 重复 Trigger 处理
// ============================================================================

bool ControlLogic::isDuplicateSensitive(ActionType type) const {
  switch (type) {
    case ActionType::Start:
    case ActionType::Quit:
    case ActionType::MotionCommand:
      return true;
    default:
      return false;
  }
}

bool ControlLogic::isLastWins(ActionType type) const {
  switch (type) {
    case ActionType::SwitchController:
    case ActionType::SetArmMode:
    case ActionType::SetWaistMode:
      return true;
    default:
      return false;
  }
}

std::vector<ActionTrigger> ControlLogic::processDuplicateTriggers(
    const std::vector<ActionTrigger>& triggers) {
  if (triggers.empty()) {
    return {};
  }

  std::vector<ActionTrigger> result;
  result.reserve(triggers.size());

  // 记录每种类型最后出现的索引（用于 LastWins 策略）
  std::unordered_map<ActionType, size_t> last_index;

  // 第一遍：统计每种类型的最后位置
  for (size_t i = 0; i < triggers.size(); ++i) {
    last_index[triggers[i].type] = i;
  }

  // 第二遍：根据策略处理
  std::unordered_set<ActionType> seen;
  for (size_t i = 0; i < triggers.size(); ++i) {
    const auto& trigger = triggers[i];
    ActionType type = trigger.type;

    // 跳过无效类型
    if (type == ActionType::None) {
      continue;
    }

    if (isDuplicateSensitive(type)) {
      // 去重策略：只保留第一个
      if (seen.insert(type).second) {
        result.push_back(trigger);
      }
    } else if (isLastWins(type)) {
      // 最后生效策略：只保留最后一个
      if (i == last_index[type]) {
        result.push_back(trigger);
      }
    } else {
      // 其他类型：保留所有
      result.push_back(trigger);
    }
  }

  return result;
}

// ============================================================================
// 自动 Controller 转移
// ============================================================================

void ControlLogic::handleAutoTransitions(const RobotState& state,
                                         ControllerManager& controller_manager,
                                         const CommandBuffer::Snapshot& command_snapshot,
                                         double now) {
  (void)state;
  (void)now;
  (void)command_snapshot;

  // 自动转移逻辑：根据当前控制器状态和名称决定是否切换
  // 例如：GroundToStand 完成后自动切换到运动控制器

  std::string current_name = controller_manager.getCurrentControllerName();
  if (current_name.empty()) {
    return;
  }

  // TODO: 实现自动转移逻辑
  // 需要控制器提供 "IsFinished" 或类似接口
  // 暂时留空，由具体控制器决定何时完成
}

// ============================================================================
// 跌倒保护与恢复逻辑
// ============================================================================

void ControlLogic::handleFallLogic(const RobotState& state,
                                   ControllerManager& controller_manager,
                                   const CommandBuffer::Snapshot& command_snapshot,
                                   double now) {
  (void)now;
  (void)command_snapshot;

  // 检测跌倒
  if (!isFallen(state)) {
    return;
  }

  // 已经在保护或恢复控制器中，不重复触发
  std::string current_name = controller_manager.getCurrentControllerName();
  if (current_name == "protective_fall" ||
      current_name == "ground_to_stand" ||
      current_name == "recovery") {
    return;
  }

  RL_LOGW("ControlLogic: Fall detected! Current controller: %s", current_name.c_str());

  // 进入保护控制
  if (controller_manager.hasController("protective_fall")) {
    RL_LOGI("ControlLogic: Switching to protective_fall");
    requestSwitch(controller_manager, "protective_fall");
  }
  // 如果没有专门的保护控制器，停止当前控制器
}

// ============================================================================
// 辅助函数
// ============================================================================

bool ControlLogic::isFallen(const RobotState& state) const {
  // 通过 IMU 判断机器人是否倒地
  // 简单的启发式：如果 torso 的 roll 或 pitch 角度超过阈值

  // 注意：这里需要根据实际机器人状态来实现
  // 暂时使用简化的判断：检查 IMU 数据

  // 假设 state 包含 IMU 数据或可以通过其他方式获取
  // 这里是一个占位实现，实际应该根据机器人姿态判断

  (void)state;

  // TODO: 实现实际的倒地检测逻辑
  // 例如：检查 base 的姿态角是否超过阈值

  return false;  // 默认不倒地
}

bool ControlLogic::requestSwitch(ControllerManager& controller_manager,
                                  const std::string& name) {
  if (!controller_manager.hasController(name)) {
    RL_LOGW("ControlLogic: Controller '%s' not found", name.c_str());
    return false;
  }

  // 使用两阶段切换（requestSwitch -> Transition -> commitSwitch）
  double now = common::GetSteadyTimestampNs() * 1e-9;
  bool success = controller_manager.requestSwitch(name, now);
  if (success) {
    RL_LOGI("ControlLogic: Requested switch to '%s' (transition started)", name.c_str());
  } else {
    RL_LOGW("ControlLogic: Failed to request switch to '%s'", name.c_str());
  }
  return success;
}

// ============================================================================
// CommandSnapshot 处理
// ============================================================================

void ControlLogic::processExternalTargets(ControllerManager& controller_manager,
                                          const CommandBuffer::Snapshot& command_snapshot) {
  // 处理手臂关节目标
  if (command_snapshot.arm_target.has_value()) {
    const auto& target = command_snapshot.arm_target.value();
    if (!target.q.empty()) {
      controller_manager.setArmTarget(target);
    }
  }

  // 处理腰部关节目标
  if (command_snapshot.waist_target.has_value()) {
    const auto& target = command_snapshot.waist_target.value();
    if (!target.q.empty()) {
      controller_manager.setWaistTarget(target);
    }
  }

  // 处理头部关节目标
  if (command_snapshot.head_target.has_value()) {
    const auto& target = command_snapshot.head_target.value();
    if (!target.q.empty()) {
      controller_manager.setHeadTarget(target);
    }
  }
}

void ControlLogic::processVelocityCommand(ControllerManager& controller_manager,
                                          const CommandBuffer::Snapshot& command_snapshot) {
  // 处理速度指令（从已合并的 CommandSnapshot）
  // 注意：输入优先级选择已在 ControlLoop::mergeAllCmdVel() 中完成
  if (command_snapshot.cmd_vel.valid) {
    VelocityCommand vel_cmd;
    vel_cmd.linear_x = command_snapshot.cmd_vel.linear_x;
    vel_cmd.linear_y = command_snapshot.cmd_vel.linear_y;
    vel_cmd.angular_z = command_snapshot.cmd_vel.angular_z;
    controller_manager.setVelocityCommand(vel_cmd);
  }
}

}  // namespace runtime
}  // namespace leju
