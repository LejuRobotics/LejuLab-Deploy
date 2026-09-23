/**
 * @file control_logic.cpp
 * @brief 控制逻辑策略层实现
 */

#include "leju-rl-controller/runtime/control_logic.h"

#include <unordered_map>
#include <unordered_set>

#include "leju-rl-controller/controllers/controller_manager.h"
#include "leju-rl-controller/controllers/generic_rl_controller.h"
#include "leju-rl-controller/rl/rl_controller_types.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/transport_fall_stand_scheduler.h"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-lowlevel/data_types.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/time_utils.hpp"

namespace leju {
namespace runtime {

// 自定义组合键动作的配套音乐播放.
//
// music_player_ 回调由组装层注入 (发布到 DDS /rt/audio_play_file, 非阻塞,
// 由 leju-audio 的 audio_player_node 按文件名解码播放). 回调未注入时仅打日志,
// 不阻塞动作执行 (单测/无音频设备场景安全).
void ControlLogic::playMusicNow(const std::string& music_name) const {
  if (music_name.empty()) return;
  if (music_player_) {
    RL_LOGI("ControlLogic: play music '%s'", music_name.c_str());
    music_player_(music_name);
  } else {
    RL_LOGI("ControlLogic: [music hook] '%s' 无播放回调 (未注入)", music_name.c_str());
  }
}

// 调度音乐：delay<=0 立即播；delay>0 登记待播，由 firePendingMusic 到点触发.
// 用于对齐舞蹈起播的就位/blend 时间，避免音乐跑在动作前面.
void ControlLogic::scheduleMusic(const std::string& music_name, double delay, double now) {
  if (music_name.empty()) return;
  if (delay <= 0.0) {
    pending_music_fire_time_ = -1.0;  // 取消上一条待播
    playMusicNow(music_name);
    return;
  }
  pending_music_ = music_name;
  pending_music_fire_time_ = now + delay;
  RL_LOGI("ControlLogic: music '%s' scheduled in %.2fs", music_name.c_str(), delay);
}

// 每拍检查到点的待播音乐（不阻塞控制环）.
void ControlLogic::firePendingMusic(double now) {
  if (pending_music_fire_time_ < 0.0) return;
  if (now >= pending_music_fire_time_) {
    playMusicNow(pending_music_);
    pending_music_fire_time_ = -1.0;
    pending_music_.clear();
  }
}

// ============================================================================
// 主控制逻辑入口
// ============================================================================

void ControlLogic::tick(const RobotState& state,
                        const ImuData& imu,
                        const std::vector<ActionTrigger>& triggers,
                        Lifecycle& lifecycle,
                        ControllerManager& controller_manager,
                        const CommandBuffer::Snapshot& command_snapshot,
                        double now) {
  // 1. 处理 Running 状态入口
  handleRunningEntry(state, imu, lifecycle, controller_manager, command_snapshot, now);

  // 2. 非 Running 状态直接返回
  if (!lifecycle.isRunning()) {
    last_lifecycle_state_ = lifecycle.state();
    return;
  }

  // 3. 处理 ActionTrigger
  handleActionTriggers(imu, triggers, lifecycle, controller_manager, command_snapshot, now);

  // 3.1 到点触发延迟起播的配套音乐（对齐舞蹈起播）
  firePendingMusic(now);

  // 4. 处理 CommandSnapshot 中的外部控制请求
  processExternalTargets(controller_manager, command_snapshot);
  processVelocityCommand(controller_manager, command_snapshot);

  // 5. 处理自动 Controller 转移
  handleAutoTransitions(state, controller_manager, command_snapshot, now);

  // 6. 处理跌倒保护与恢复逻辑
  handleFallLogic(state, imu, controller_manager, command_snapshot, now);

  last_lifecycle_state_ = lifecycle.state();
}

// ============================================================================
// Running 状态入口逻辑
// ============================================================================

void ControlLogic::handleRunningEntry(const RobotState& state,
                                       const ImuData& imu,
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

  // running_entry_handled_ 永不复位：入口姿态判定仅进程内首次 Start 执行一次，勿在 Quit 处重置
  if (!just_entered_running || running_entry_handled_) {
    return;
  }

  RL_LOGI("ControlLogic: Handling Running entry");
  RL_LOGI("ControlLogic: Current controller before entry: %s",
          controller_manager.getCurrentControllerName().c_str());
  controller_manager.clearVelocityOnRunningEntry();

  // RoBan 2.2 倒地时直接进入 FALL_DOWN 瘫软；首次按键才 PREPARE。
  if (isFallen(imu)) {
    if (transport_coordinator_ && controller_manager.hasController("mimic_fall_stand")) {
      const std::string current = controller_manager.getCurrentControllerName();
      if (current != "mimic_fall_stand") {
        RL_LOGI("ControlLogic: Robot is fallen at START, switch to mimic_fall_stand FALL_DOWN");
        requestSwitch(controller_manager, "mimic_fall_stand", /*auto_start_motion=*/false,
                      /*instant_commit=*/true);
      }
    } else if (controller_manager.hasController("ground_to_stand")) {
      RL_LOGI("ControlLogic: Robot is fallen, switching to ground_to_stand");
      requestSwitch(controller_manager, "ground_to_stand");
    } else if (controller_manager.hasController("recovery")) {
      RL_LOGI("ControlLogic: Robot is fallen, switching to recovery");
      requestSwitch(controller_manager, "recovery");
    } else {
      RL_LOGW("ControlLogic: Robot is fallen but no recovery controller available");
    }
  } else {
    // 直立 -> 保持启动姿态策略选定的默认控制器（通常为 amp）
    RL_LOGI("ControlLogic: Robot is upright, NOT switching controller");
  }

  running_entry_handled_ = true;
}

// ============================================================================
// ActionTrigger 处理
// ============================================================================

void ControlLogic::handleActionTriggers(const ImuData& imu,
                                        const std::vector<ActionTrigger>& triggers,
                                        Lifecycle& lifecycle,
                                        ControllerManager& controller_manager,
                                        const CommandBuffer::Snapshot& command_snapshot,
                                        double now) {
  (void)lifecycle;
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

  // 搬运模式进行中（非 INACTIVE）：门控外部触发（对齐研杨：搬运期间 joy 按键
  // 整体早返），防止手柄/上位机切控制器、改部位模式、起播动作破坏搬运姿态
  // 或全身锁死状态。Lifecycle（Start/Quit）不门控。
  const bool transport_gating = isTransportGating();

  // 2. 按优先级阶段处理
  // Phase 1: Lifecycle (Start, Quit) - 已由 Lifecycle::update() 处理，这里跳过

  // Phase 2: Controller Switch
  for (const auto& trigger : processed) {
    if (trigger.type == ActionType::SwitchController) {
      if (transport_gating) {
        RL_LOGW("ControlLogic: switch_controller ignored, transport mode active");
        continue;
      }
      if (trigger.args) {
        auto* args = dynamic_cast<NamedArgs*>(trigger.args.get());
        if (args && !args->name.empty()) {
          RL_LOGI("ControlLogic: Processing switch_controller to '%s' (auto_start=%d)",
                  args->name.c_str(), args->auto_start_motion ? 1 : 0);
          requestSwitch(controller_manager, args->name, args->auto_start_motion);
          // auto_start_motion 场景: 提前调度目标控制器的配套音乐（延迟对齐过渡时间）
          if (args->auto_start_motion) {
            std::string music;
            double music_delay = 0.0;
            if (controller_manager.getControllerMusic(args->name, music, music_delay)) {
              scheduleMusic(music, music_delay, now);
            }
          }
        }
      }
    }
  }

  // Phase 3: Mode Change (Arm, Waist)
  for (const auto& trigger : processed) {
    if (trigger.type == ActionType::SetArmMode) {
      if (transport_gating) {
        RL_LOGW("ControlLogic: set_arm_mode ignored, transport mode active");
        continue;
      }
      RL_LOGI("ControlLogic: Handling SetArmMode trigger");
      if (trigger.args) {
        auto* args = dynamic_cast<NamedArgs*>(trigger.args.get());
        if (args && !args->name.empty()) {
          RL_LOGI("ControlLogic: Processing set_arm_mode to '%s'", args->name.c_str());
          std::optional<ArmControlMode> mode_opt;
          if (args->name == "toggle_keep_pose") {
            // LB+B：仅锁定手臂为 kKeepPose（不再翻转解锁）
            mode_opt = ArmControlMode::kKeepPose;
          } else if (args->name == "cycle_arm_mode") {
            // LB+Y 模式轮转：auto → external → keep_pose → auto 循环
            const auto current = controller_manager.getCurrentArmMode();
            if (current == ArmControlMode::kAuto) {
              mode_opt = ArmControlMode::kExternal;
            } else if (current == ArmControlMode::kExternal) {
              mode_opt = ArmControlMode::kKeepPose;
            } else {
              // kKeepPose 或未知 → auto
              mode_opt = ArmControlMode::kAuto;
            }
          } else {
            mode_opt = StringToArmControlMode(args->name);
          }
          if (mode_opt.has_value()) {
            std::string message;
            bool success = controller_manager.setArmMode(mode_opt.value(), message);
            RL_LOGI("ControlLogic: setArmMode result=%s, msg=%s", success ? "true" : "false", message.c_str());
            // TactPlayer 播完/冻结时复位 M1/M2 速度屏蔽标记：
            // - kAuto：TactPlayer 正常播完恢复
            // - kKeepPose：LB+B 冻结手臂触发 TactPlayer::Freeze()，跳过 kAuto 恢复
            if (success && (mode_opt.value() == ArmControlMode::kAuto ||
                            mode_opt.value() == ArmControlMode::kKeepPose)) {
              auto* ctrl = controller_manager.getCurrentController();
              auto* generic_rl = dynamic_cast<GenericRLController*>(ctrl);
              if (generic_rl && generic_rl->isBlockingVelocityInMotion()) {
                generic_rl->setBlockVelocityInMotion(false);
              }
            }
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
      if (transport_gating) {
        RL_LOGW("ControlLogic: set_waist_mode ignored, transport mode active");
        continue;
      }
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
      if (transport_gating) {
        RL_LOGW("ControlLogic: motion command ignored, transport mode active");
        continue;
      }
      if (trigger.args) {
        auto* args = dynamic_cast<MotionCommandArgs*>(trigger.args.get());
        if (args && args->op == MotionCommandArgs::Operation::Start) {
          // M1/M2 组合键动作播放期间屏蔽摇杆行走
          if (args->block_velocity_during_motion) {
            auto* ctrl = controller_manager.getCurrentController();
            auto* generic_rl = dynamic_cast<GenericRLController*>(ctrl);
            if (generic_rl) {
              generic_rl->setBlockVelocityInMotion(true);
            }
          }
          bool motion_ok = false;
          if (!args->motion_name.empty()) {
            RL_LOGI("ControlLogic: Processing motion_start '%s'", args->motion_name.c_str());
            motion_ok = controller_manager.startMotion(args->motion_name);
          } else {
            motion_ok = controller_manager.startMotion();
          }
          // motion 启动失败（如空 name 占位绑定无对应 tact 文件）：立即复位屏蔽标记
          if (!motion_ok && args->block_velocity_during_motion) {
            auto* ctrl = controller_manager.getCurrentController();
            auto* generic_rl = dynamic_cast<GenericRLController*>(ctrl);
            if (generic_rl && generic_rl->isBlockingVelocityInMotion()) {
              generic_rl->setBlockVelocityInMotion(false);
            }
          }
          // 自定义组合键动作的配套音乐（延迟可选，对齐舞蹈起播；发布非阻塞）
          // 触发器未显式指定 music 时，回退到当前激活控制器在 controller_manager.yaml
          // 里配置的默认配乐——这样多支舞蹈可以共享同一个"启动"按键，各自配乐不冲突。
          std::string music = args->music;
          double music_delay = args->music_delay;
          if (music.empty()) {
            std::string fallback_music;
            double fallback_delay = 0.0;
            if (controller_manager.getCurrentControllerMusic(fallback_music, fallback_delay)) {
              music = fallback_music;
              music_delay = fallback_delay;
            }
          }
          scheduleMusic(music, music_delay, now);
        }
      }
    }
  }

  // Phase 5: 搬运/倒地起身调度事件
  for (const auto& trigger : processed) {
    if (trigger.type != ActionType::TransportFallStand) {
      continue;
    }
    std::string name;
    if (trigger.args) {
      if (auto* named = dynamic_cast<NamedArgs*>(trigger.args.get())) {
        name = named->name;
      }
    }
    if (transport_fall_stand_scheduler_) {
      transport_fall_stand_scheduler_->onEvent(name, imu, now);
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
  // mimic_fall_stand 起身完成后的回切 amp 由 FallStandCoordinator 编排
}

// ============================================================================
// 跌倒保护与恢复逻辑
// ============================================================================

void ControlLogic::handleFallLogic(const RobotState& state,
                                   const ImuData& imu,
                                   ControllerManager& controller_manager,
                                   const CommandBuffer::Snapshot& command_snapshot,
                                   double now) {
  (void)state;
  (void)now;
  (void)command_snapshot;

  // 搬运 ACTIVE（hold_pose 全身锁死、被搬运）期间 IMU 姿态可以是任意的，
  // 不触发跌倒保护，否则会把控制器从锁死状态切走
  if (transport_coordinator_ && transport_coordinator_->isFallProtectionSuppressed()) {
    return;
  }

  if (!isFallen(imu)) {
    return;
  }

  const std::string current_name = controller_manager.getCurrentControllerName();
  if (current_name == "mimic_fall_stand") {
    return;
  }

  // TransportModeCoordinator 仅在 RoBan 2.2 接线；倒地时直接进入零力矩 FALL_DOWN。
  if (transport_coordinator_ && controller_manager.hasController("mimic_fall_stand")) {
    RL_LOGW("ControlLogic: Fall detected! Current controller: %s", current_name.c_str());
    RL_LOGI("ControlLogic: Switching to mimic_fall_stand FALL_DOWN");
    requestSwitch(controller_manager, "mimic_fall_stand", /*auto_start_motion=*/false,
                  /*instant_commit=*/true);
    return;
  }

  // 非 RoBan 2.2 维持原有 protective_fall 回退路径。
  if (current_name == "protective_fall" ||
      current_name == "ground_to_stand" ||
      current_name == "recovery") {
    return;
  }
  if (controller_manager.hasController("protective_fall")) {
    RL_LOGW("ControlLogic: Fall detected! Current controller: %s", current_name.c_str());
    RL_LOGI("ControlLogic: Switching to protective_fall");
    requestSwitch(controller_manager, "protective_fall");
  }
}

// ============================================================================
// 辅助函数
// ============================================================================

bool ControlLogic::isFallen(const ImuData& imu) const {
  return IsFallenPose(imu);
}

bool ControlLogic::requestSwitch(ControllerManager& controller_manager,
                                  const std::string& name,
                                  bool auto_start_motion,
                                  bool instant_commit) {
  if (!controller_manager.hasController(name)) {
    RL_LOGW("ControlLogic: Controller '%s' not found", name.c_str());
    return false;
  }

  // 使用两阶段切换（requestSwitch -> Transition -> commitSwitch）
  double now = common::GetSteadyTimestampNs() * 1e-9;
  bool success = controller_manager.requestSwitch(name, now, auto_start_motion,
                                                  instant_commit);
  if (success) {
    RL_LOGI("ControlLogic: Requested switch to '%s' (instant=%d)",
            name.c_str(), instant_commit ? 1 : 0);
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
  // 搬运模式进行中（非 INACTIVE）：门控手臂/腰部外部目标（对齐研杨 ACTIVE 锁手臂），
  // 防止外部关节目标与搬运零位/锁死姿态打架；头部/手部不锁，放行。
  // 注意 seq 照常消费，避免退出搬运后把搬运期间写入的陈旧目标补推出去。
  const bool transport_gating = isTransportGating();

  // 处理手臂关节目标：仅在 seq 发生变化（即上游有新写入）时才推送，避免
  // CommandBuffer 粘性缓存导致同一目标在模式切换后被反复重推。
  if (command_snapshot.arm_target.isNewerThan(last_applied_arm_target_seq_)) {
    if (!transport_gating) {
      const auto& target = command_snapshot.arm_target.getValue();
      if (!target.q.empty()) {
        controller_manager.setArmTarget(target);
      }
    }
    last_applied_arm_target_seq_ = command_snapshot.arm_target.seq;
  }

  // 处理腰部关节目标
  if (command_snapshot.waist_target.isNewerThan(last_applied_waist_target_seq_)) {
    if (!transport_gating) {
      const auto& target = command_snapshot.waist_target.getValue();
      if (!target.q.empty()) {
        controller_manager.setWaistTarget(target);
      }
    }
    last_applied_waist_target_seq_ = command_snapshot.waist_target.seq;
  }


  // 处理手部目标：通过 DDS /rt/hand_cmd 发给 hardware_node，再由硬件进程调用 SetHandCommand。
  if (command_snapshot.hand_target.isNewerThan(last_applied_hand_target_seq_)) {
    const auto& target = command_snapshot.hand_target.getValue();
    if (target.isValid()) {
      leju::HandCmd hand_cmd(target.position);
      hand_cmd.timestamp = leju::common::GetUnixTimestampS();
      if (!leju::GlobalRobot::getInstance().publishHandCmd(hand_cmd)) {
        RL_LOGW("ControlLogic: failed to publish hand target");
      }
    }
    last_applied_hand_target_seq_ = command_snapshot.hand_target.seq;
  }

  // 处理头部关节目标
  if (command_snapshot.head_target.isNewerThan(last_applied_head_target_seq_)) {
    const auto& target = command_snapshot.head_target.getValue();
    if (!target.q.empty()) {
      controller_manager.setHeadTarget(target);
    }
    last_applied_head_target_seq_ = command_snapshot.head_target.seq;
  }
}

void ControlLogic::processVelocityCommand(ControllerManager& controller_manager,
                                          const CommandBuffer::Snapshot& command_snapshot) {
  // 搬运/倒地起身期间的速度门控在 ControlLoop 源头完成（cmd_vel 清零置 invalid）

  // 处理速度指令（从已合并的 CommandSnapshot）
  // 注意：输入优先级选择已在 ControlLoop::mergeAllCmdVel() 中完成
  if (!command_snapshot.cmd_vel.valid) {
    return;
  }

  const bool is_amp = controller_manager.getCurrentControllerName() == "amp";
  const bool squat_defense =
      is_amp && controller_manager.isDeepSquatGuardActive();

  VelocityCommand vel_cmd;
  vel_cmd.linear_x = command_snapshot.cmd_vel.linear_x;
  vel_cmd.linear_y = command_snapshot.cmd_vel.linear_y;
  vel_cmd.angular_z = command_snapshot.cmd_vel.angular_z;

  // 深蹲守备：膝角过深时屏蔽走/转，并锁定 cmd_stance=1（姿态/下蹲模式）。exit_posture（摇杆推行走、cmd_stance_mode=0）仅清零高度命令 angular_z，
  // 不在此退出 posture；守备期间禁止 setCmdStanceMode(0)。
  // 待膝角物理站起、守备解除后，用户需再次按遥控器退出姿态模式（cmd_stance→0）才能行走。
  if (squat_defense) {
    const bool exit_posture =
        command_snapshot.cmd_vel.cmd_stance_valid &&
        command_snapshot.cmd_vel.cmd_stance_mode == 0;
    if (exit_posture && !squat_exit_posture_deferred_logged_) {
      RL_LOGD(
          "Squat posture defense: exit posture deferred until knees stand up "
          "(vel zeroed, cmd_stance remains 1)");
      squat_exit_posture_deferred_logged_ = true;
    } else if (!exit_posture) {
      squat_exit_posture_deferred_logged_ = false;
    }
    vel_cmd.linear_x = 0.0;
    vel_cmd.linear_y = 0.0;
    if (exit_posture) {
      // 用户想起身：先停高度，不切换 cmd_stance_mode
      vel_cmd.angular_z = 0.0;
    } else if (command_snapshot.cmd_vel.cmd_stance_mode != 0) {
      vel_cmd.angular_z = command_snapshot.cmd_vel.angular_z;
    } else {
      vel_cmd.angular_z = 0.0;
    }
    controller_manager.setVelocityCommand(vel_cmd);
    if (command_snapshot.cmd_vel.cmd_stance_valid) {
      // 守备期间保持 posture，忽略遥控器的 cmd_stance_mode=0 退出请求
      std::string message;
      controller_manager.setCmdStanceMode(1, message);
    }
    return;
  }

  squat_exit_posture_deferred_logged_ = false;

  controller_manager.setVelocityCommand(vel_cmd);

  if (command_snapshot.cmd_vel.cmd_stance_valid && is_amp) {
    std::string message;
    controller_manager.setCmdStanceMode(command_snapshot.cmd_vel.cmd_stance_mode,
                                      message);
  }
}

}  // namespace runtime
}  // namespace leju
