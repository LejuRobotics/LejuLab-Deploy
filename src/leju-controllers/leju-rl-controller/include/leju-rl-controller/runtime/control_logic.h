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

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "leju-rl-controller/runtime/input/action_trigger.h"
#include "leju-rl-controller/runtime/input/command_buffer.h"
#include "leju-rl-controller/runtime/lifecycle.h"
#include "leju-rl-controller/runtime/transport_mode_coordinator.h"

namespace leju {

class ControllerManager;
class RobotState;
struct ImuData;

namespace runtime {

// 前向声明
class CommandBuffer;
struct ActionTrigger;
class TransportFallStandScheduler;

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
            const ImuData& imu,
            const std::vector<ActionTrigger>& triggers,
            Lifecycle& lifecycle,
            ControllerManager& controller_manager,
            const CommandBuffer::Snapshot& command_snapshot,
            double now);

  /**
   * @brief 注入音乐播放回调（可选）
   *
   * MotionCommand{Start} 携带 music 时，ControlLogic 调用此回调播放配套音乐。
   * 回调由组装层接线（发布到 DDS /rt/audio_play_file，非阻塞）。未注入时退化为
   * 仅打日志，保持 ControlLogic 无 DDS 依赖、可单测。
   *
   * @param cb 接收音乐文件名的回调
   */
  void setMusicPlayer(std::function<void(const std::string&)> cb) {
    music_player_ = std::move(cb);
  }

  /**
   * @brief 注入搬运模式协调器（可选，仅 Roban 2.2 注入）
   *
   * 由 main.cpp 在 ControlLoop 运行前注入；ControlLogic 仅做门控查询
   * （isInputGating/isFallProtectionSuppressed），副作用与状态发布由
   * coordinator 自身 tick 完成（ControlLoop 直调）。未注入时搬运模式不参与控制。
   */
  void setTransportModeCoordinator(
      std::shared_ptr<TransportModeCoordinator> coordinator) {
    transport_coordinator_ = std::move(coordinator);
  }

  /// 注入手柄搬运/倒地起身调度器（TransportFallStand 事件入口）
  void setTransportFallStandScheduler(
      std::shared_ptr<TransportFallStandScheduler> scheduler) {
    transport_fall_stand_scheduler_ = std::move(scheduler);
  }

 private:
  /**
   * @brief 处理 Running 状态入口逻辑
   *
   * 当 Lifecycle 从 WaitingForStart -> Running 时：
   * - 倒地 -> GroundToStand
   * - 正常 -> AMP
   */
  void handleRunningEntry(const RobotState& state,
                          const ImuData& imu,
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
  void handleActionTriggers(const ImuData& imu,
                            const std::vector<ActionTrigger>& triggers,
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
                       const ImuData& imu,
                       ControllerManager& controller_manager,
                       const CommandBuffer::Snapshot& command_snapshot,
                       double now);

  /**
   * @brief 判断机器人是否倒地（IMU |roll|/|pitch| 超过 IsFallenPose 阈值，对齐 kuavo）
   */
  bool isFallen(const ImuData& imu) const;

  /**
   * @brief 请求切换控制器
   * @param controller_manager 控制器管理器
   * @param name 目标控制器名称
   * @return 是否切换成功
   */
  bool requestSwitch(ControllerManager& controller_manager,
                     const std::string& name,
                     bool auto_start_motion = false,
                     bool instant_commit = false);

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

  /**
   * @brief 调度自定义组合键动作的配套音乐
   *
   * delay<=0 时立即播放；delay>0 时登记为待播，由 firePendingMusic 在 now 到点后触发
   * （用于对齐舞蹈起播的就位/blend 时间，避免音乐跑在动作前面）。music_name 空则忽略。
   *
   * @param music_name 音乐文件名
   * @param delay 起播延迟[s]
   * @param now 当前时间戳[s]
   */
  void scheduleMusic(const std::string& music_name, double delay, double now);

  /**
   * @brief 检查并触发到点的待播音乐（每拍调用，不阻塞）
   * @param now 当前时间戳[s]
   */
  void firePendingMusic(double now);

  /**
   * @brief 实际触发音乐播放（调注入的 music_player_ 回调，为空时打日志）
   */
  void playMusicNow(const std::string& music_name) const;

 private:
  LifecycleState last_lifecycle_state_ = LifecycleState::kWaitingForReady;
  bool running_entry_handled_ = false;  ///< 标记 Running 入口是否已处理

  ///< 音乐播放回调（可选，组装层注入；为空时退化为日志）
  std::function<void(const std::string&)> music_player_;
  ///< 待播音乐（延迟起播用）：文件名 + 到点时间戳；fire_time<0 表示无待播
  std::string pending_music_;
  double pending_music_fire_time_ = -1.0;

  // 已应用过的外部目标 seq，用于去重：只在 snapshot 里的 seq 跟这个不一致时
  // 才调用 setXxxTarget，避免 CommandBuffer 的粘性缓存被反复重推。
  uint64_t last_applied_arm_target_seq_ = 0;
  uint64_t last_applied_head_target_seq_ = 0;
  uint64_t last_applied_waist_target_seq_ = 0;
  uint64_t last_applied_hand_target_seq_ = 0;

  /// 深蹲守备期间 exit_posture 边沿 log 已打（避免每控制周期刷屏）
  /// exit_posture 仅停高度；守备解除后需用户再次按遥控器退出姿态模式
  bool squat_exit_posture_deferred_logged_ = false;

  // 搬运模式协调器（可选，由组装层注入；仅做门控查询）
  std::shared_ptr<TransportModeCoordinator> transport_coordinator_;
  std::shared_ptr<TransportFallStandScheduler> transport_fall_stand_scheduler_;

  /// 搬运模式进行中：外部输入需被门控
  bool isTransportGating() const {
    return transport_coordinator_ && transport_coordinator_->isInputGating();
  }
};

}  // namespace runtime
}  // namespace leju
