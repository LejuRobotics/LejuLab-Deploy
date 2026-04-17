/**
 * @file control_loop.h
 * @brief ControlLoop - 主控制循环
 *
 * 核心设计：
 * - 固定频率运行
 * - 每拍 Tick() 推进控制系统
 * - ControlLoop 不知道 active_controller 细节，只调用 ControllerManager.update()
 * - 通过 Drain 语义消费触发器
 */

#pragma once

#include <atomic>
#include <chrono>
#include <optional>
#include <vector>

#include "lejusdk-lowlevel/leju_sdk.h"

#include "leju-rl-controller/runtime/input/command_buffer.h"
#include "leju-rl-controller/runtime/control_logic.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/input/input_source.h"

namespace leju {

// 前向声明
class ControllerManager;
class RobotData;

namespace runtime {

// 前向声明
class TriggerBuffer;
class Lifecycle;

/**
 * @brief ControlLoop - 主控制循环
 *
 * 职责：
 * 1. 每拍执行 Tick()：获取状态、更新 Lifecycle、调用 ControllerManager
 * 2. 封装控制时序，主逻辑通过 ControllerManager 委托
 * 3. 控制频率由当前活跃控制器决定（通过 ControllerManager::waitNextCycle）
 *
 * 设计原则：
 * - ControlLoop 不知道 controller 数量、名称、类型、频率
 * - 只调用 ControllerManager.update()，不直接访问 active_controller
 * - Trigger 消费使用 Drain 语义（取出并清空）
 */
class ControlLoop {
 public:
  /**
   * @brief 构造函数
   * @param robot_data 机器人数据接口（获取传感器状态）
   * @param trigger_buffer 触发器缓冲区
   * @param input_sources 输入源列表（不拥有所有权），ControlLoop 内部会按优先级排序
   * @param lifecycle 生命周期管理器
   * @param controller_manager 控制器管理器
   */
  ControlLoop(RobotData& robot_data,
              runtime::TriggerBuffer& trigger_buffer,
              const std::vector<InputSource*>& input_sources,
              runtime::Lifecycle& lifecycle,
              ControllerManager& controller_manager);

  /**
   * @brief 主循环入口（阻塞直到 stop() 被调用）
   *
   * 内部循环：
   * while (running_) {
   *     Tick();
   *     SleepUntilNextPeriod();
   * }
   */
  void run();

  /**
   * @brief 停止循环（线程安全）
   *
   * 可以在其他线程调用，使 Run() 退出
   */
  void stop();

  /**
   * @brief 检查是否正在运行
   * @return true 表示循环正在运行
   */
  bool isRunning() const { return running_.load(); }

 private:
  /**
   * @brief 单拍执行
   *
   * 执行流程：
   * 1. 获取最新 RobotState
   * 2. 从 TriggerBuffer Drain triggers（按类别）
   * 3. 更新 Lifecycle（使用 lifecycle triggers）
   * 4. 未 Running -> return
   * 5. Running -> ControlLogic.Tick() / 更新 Mode
   * 6. 获取命令快照并合并
   * 7. 调用 ControllerManager.update(state, command_snapshot)
   * 8. 发布 JointCommand
   */
  void tick();

  /**
   * @brief 睡眠到下一控制周期
   * @param cycle_start 本次循环开始时间点
   *
   * 委托给 ControllerManager，由它调用当前控制器的 waitNextCycle() 实现变频控制
   */
  void sleepUntilNextPeriod(std::chrono::steady_clock::time_point cycle_start);

  /**
   * @brief 发布机器人指令
   * @param cmd 控制指令
   */
  void publishRobotCmd(const RobotCmd& cmd);

  /**
   * @brief 合并所有输入源的速度指令
   *
   * 按照输入源的优先级（数值越小优先级越高）合并速度指令。
   * 高优先级的有效指令会覆盖低优先级的指令。
   *
   * @return 合并后的速度指令
   */
  MotionCommand mergeAllCmdVel() const;

  /**
   * @brief 合并所有输入源的手臂目标
   *
   * 按照输入源的优先级（数值越小优先级越高）合手臂目标。
   * 高优先级的有效目标会覆盖低优先级的目标。
   *
   * @return 合并后的手臂目标
   */
  std::optional<ExternalJointTarget> mergeArmTarget() const;

  /**
   * @brief 合并所有输入源的头部目标
   *
   * 按照输入源的优先级（数值越小优先级越高）合并头部目标。
   * 高优先级的有效目标会覆盖低优先级的目标。
   *
   * @return 合并后的头部目标
   */
  std::optional<ExternalJointTarget> mergeHeadTarget() const;

 private:
  std::atomic<bool> running_{false};       ///< 运行标志

  // 外部模块引用（由调用者保证生命周期）
  RobotData& robot_data_;
  runtime::TriggerBuffer& trigger_buffer_;
  std::vector<InputSource*> input_sources_;  ///< 输入源列表（按优先级排序）
  runtime::Lifecycle& lifecycle_;
  ControllerManager& controller_manager_;

  // 策略决策层
  ControlLogic control_logic_;

  // 时序控制
  std::chrono::steady_clock::time_point current_tick_start_;
};

}  // namespace runtime
}  // namespace leju
