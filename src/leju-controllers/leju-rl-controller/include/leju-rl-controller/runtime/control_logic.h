/**
 * @file control_logic.h
 * @brief 控制逻辑策略层
 *
 * ControlLogic 是控制框架中的策略决策层 (Strategy Layer)。
 *
 * 职责：
 * - 根据 RobotState、Lifecycle、ActionTrigger 等信息
 * - 决定当前系统应该运行哪个 controller
 * - 决定哪些模式应该更新
 * - 决定是否触发自动转移
 *
 * 位置：
 * Input Semantics -> ControlLogic -> ControllerManager -> Controller
 */

#pragma once

#include <string>
#include <vector>

#include "leju-rl-controller/runtime/input/action_trigger.h"
#include "leju-rl-controller/runtime/input/command_buffer.h"
#include "leju-rl-controller/runtime/lifecycle.h"

namespace leju {

class ControllerManager;
class RobotState;

namespace runtime {

// 前向声明
class CommandBuffer;
struct ActionTrigger;

/**
 * @brief 控制逻辑策略层
 *
 * 不负责：
 * - Input 解析（TeleopAdapter 负责）
 * - Lifecycle 管理（Lifecycle 负责）
 * - Controller 管理（ControllerManager 负责）
 * - 控制算法（Controller 负责）
 *
 * 负责：
 * - Trigger 语义解释
 * - Controller 切换策略
 * - 自动转移逻辑
 * - 跌倒与恢复策略
 */
class ControlLogic {
 public:
  ControlLogic() = default;
  ~ControlLogic() = default;

  /**
   * @brief 主控制逻辑入口
   *
   * 每个控制周期调用一次，执行顺序：
   * 1. 处理 Lifecycle 进入 Running 的启动逻辑
   * 2. 非 Running 状态直接返回
   * 3. 处理 ActionTrigger
   * 4. 处理自动 controller 转移
   * 5. 处理跌倒保护与恢复逻辑
   *
   * @param state 当前机器人状态
   * @param triggers 动作触发器列表
   * @param lifecycle 生命周期管理器
   * @param controller_manager 控制器管理器
   * @param command_snapshot 命令快照（已合并的最终命令）
   * @param now 当前时间戳
   */
  void tick(const RobotState& state,
            const std::vector<ActionTrigger>& triggers,
            Lifecycle& lifecycle,
            ControllerManager& controller_manager,
            const CommandBuffer::Snapshot& command_snapshot,
            double now);

 private:
  /**
   * @brief 处理 Running 状态入口逻辑
   *
   * 当 Lifecycle 从 WaitingForStart -> Running 时：
   * - 倒地 -> GroundToStand
   * - 正常 -> AMP
   */
  void handleRunningEntry(const RobotState& state,
                          Lifecycle& lifecycle,
                          ControllerManager& controller_manager,
                          const CommandBuffer::Snapshot& command_snapshot,
                          double now);

  /**
   * @brief 处理 ActionTrigger
   *
   * 处理来自 InputAdapter 的触发事件：
   * - switch_controller
   * - set_arm_mode
   * - recover_from_fall 等
   */
  void handleActionTriggers(const std::vector<ActionTrigger>& triggers,
                            Lifecycle& lifecycle,
                            ControllerManager& controller_manager,
                            const CommandBuffer::Snapshot& command_snapshot,
                            double now);

  /**
   * @brief 处理自动 Controller 转移
   *
   * 某些 controller 完成后需要自动切换：
   * - GroundToStand 完成 -> AMP
   * - ProtectiveFall 完成 -> Suspended
   * - Recovery 完成 -> AMP
   */
  void handleAutoTransitions(const RobotState& state,
                             ControllerManager& controller_manager,
                             const CommandBuffer::Snapshot& command_snapshot,
                             double now);

  /**
   * @brief 处理跌倒与恢复逻辑
   *
   * - 检测跌倒
   * - 决定是否进入保护控制
   * - 恢复触发策略
   */
  void handleFallLogic(const RobotState& state,
                       ControllerManager& controller_manager,
                       const CommandBuffer::Snapshot& command_snapshot,
                       double now);

  /**
   * @brief 判断机器人是否倒地
   * @param state 机器人状态
   * @return true 表示已倒地
   */
  bool isFallen(const RobotState& state) const;

  /**
   * @brief 请求切换控制器
   * @param controller_manager 控制器管理器
   * @param name 目标控制器名称
   * @return 是否切换成功
   */
  bool requestSwitch(ControllerManager& controller_manager,
                     const std::string& name);

  /**
   * @brief 处理外部关节目标
   *
   * 从 CommandSnapshot 读取 external_arm_target, external_waist_target,
   * external_head_target，并调用 ControllerManager 的 setXXX 方法
   *
   * @param controller_manager 控制器管理器
   * @param command_snapshot 命令快照
   */
  void processExternalTargets(ControllerManager& controller_manager,
                              const CommandBuffer::Snapshot& command_snapshot);

  /**
   * @brief 处理速度指令
   *
   * 从 CommandSnapshot 读取 cmd_vel，并调用 ControllerManager
   *
   * @param controller_manager 控制器管理器
   * @param command_snapshot 命令快照
   */
  void processVelocityCommand(ControllerManager& controller_manager,
                              const CommandBuffer::Snapshot& command_snapshot);

  /**
   * @brief 处理重复的 ActionTrigger
   *
   * 根据 ActionType 的不同策略处理重复触发器：
   * - 去重策略（Start, Quit, StartMotion）：同一 tick 内只保留一个
   * - 最后生效策略（SwitchController, SetArmMode, SetWaistMode）：保留最后一个
   *
   * @param triggers 原始触发器列表
   * @return 处理后的触发器列表
   */
  std::vector<ActionTrigger> processDuplicateTriggers(
      const std::vector<ActionTrigger>& triggers);

  /**
   * @brief 判断 ActionType 是否使用去重策略
   * @param type 动作类型
   * @return true 表示需要去重（只保留一个）
   */
  bool isDuplicateSensitive(ActionType type) const;

  /**
   * @brief 判断 ActionType 是否使用最后生效策略
   * @param type 动作类型
   * @return true 表示最后生效（保留最后一个）
   */
  bool isLastWins(ActionType type) const;

 private:
  LifecycleState last_lifecycle_state_ = LifecycleState::kWaitingForReady;
  bool running_entry_handled_ = false;  ///< 标记 Running 入口是否已处理
};

}  // namespace runtime
}  // namespace leju
