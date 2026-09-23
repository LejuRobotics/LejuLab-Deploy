/**
 * @file joy_teleop_adapter.cpp
 * @brief Joy 手柄输入适配器实现（配置驱动组合键）
 */

#include "leju-rl-controller/runtime/input/teleop/joy_teleop_adapter.h"

#include <chrono>
#include <cmath>

#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/runtime/dexterous_hand_poses.h"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/time_utils.hpp"

namespace leju {
namespace runtime {

// ============================================================================
// 构造/析构
// ============================================================================

JoyTeleopAdapter::JoyTeleopAdapter(TriggerBuffer* trigger_buffer)
    : TeleopAdapterBase<JoyTeleopAdapter, JoyData>(trigger_buffer) {}

JoyTeleopAdapter::~JoyTeleopAdapter() {
  shutdown();
}

// ============================================================================
// 初始化和关闭
// ============================================================================

bool JoyTeleopAdapter::initialize() {
  if (running_.load()) {
    return true;
  }

  if (!GlobalRobot::is_initialized()) {
    RL_LOGE("JoyTeleopAdapter: GlobalRobot not initialized");
    return false;
  }

  running_ = true;
  prev_buttons_ = {};

  // 直接订阅 SDK 手柄数据
  auto& robot = GlobalRobot::getInstance();
  robot.subscribeJoyData([this](const JoyDataConstPtr& joy) {
    if (!running_.load()) return;

    this->onJoyData(*joy, prev_buttons_);
    prev_buttons_ = joy->buttons;
  });
  robot.subscribeHandState([this](const HandStateConstPtr& state) {
    if (!running_.load()) return;
    bool expected = false;
    if (!hand_presence_resolved_.compare_exchange_strong(expected, true)) return;
    const bool available = state && state->left_valid && state->right_valid;
    dexterous_hands_available_.store(available);
    RL_LOGI("JoyTeleopAdapter: Dexterous hands %s for hand shortcuts",
            available ? "available" : "unavailable");
  });

  RL_LOGI("JoyTeleopAdapter: Initialized and subscribed to joy data");
  return true;
}

void JoyTeleopAdapter::shutdown() {
  if (!running_.load()) {
    return;
  }

  running_ = false;
  RL_LOGI("JoyTeleopAdapter: Shutdown");
}

bool JoyTeleopAdapter::isInitialized() const {
  return running_.load();
}

// ============================================================================
// Joy 数据处理（内部回调）
// ============================================================================

void JoyTeleopAdapter::onJoyData(const JoyData& joy, const JoyData::Buttons& prev) {
  if (!trigger_buffer_) {
    return;  // 未设置 TriggerBuffer，无法输出触发器
  }

  // 整帧持锁: 与热重载 (setBindingConfig) 互斥, 保护下游对 config_ / binding_config_
  // 的读 (processVelocityImpl / applyBindingForCombo). 重载罕见, 该锁基本无竞争.
  std::lock_guard<std::mutex> binding_lk(binding_mutex_);

  // 当前时间戳
  double current_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

  // 1. 走路状态（基于原始输入，与屏蔽状态解耦）—— 供 block_when_walking 门控
  const bool walking = isWalking(joy);

  // 2. 识别当前组合键
  ComboKey current_combo = detectCurrentComboImpl(joy);

  // 调试：打印组合键变化
  if (has_prev_state_ && !(prev_combo_ == current_combo)) {
    RL_LOGD("JoyTeleopAdapter: Combo changed from [%s] to [%s]",
            ComboKeyToString(prev_combo_).c_str(),
            ComboKeyToString(current_combo).c_str());
  }

  // 3. 处理系统级按钮（START/BACK）
  std::vector<ActionTrigger> triggers;
  processSystemButtons(joy, prev, triggers);

  // 4. 处理组合键边沿（按 press/release + 走路门控产出触发器）
  if (has_prev_state_) {
    processComboEdges(prev_combo_, current_combo, walking, triggers);
    // LB+B 手臂模式 toggle（特殊处理：复用 SetArmMode 接口，需 adapter 内部状态）
    handleArmModeToggle(current_combo, triggers);

    // LB+Y 手臂模式轮转
    handleArmUnlock(current_combo, triggers);

    // LB+X 灵巧手抓握 toggle
    handleHandGripToggle(current_combo);
  }

  // 仅依据本帧产生的 START/切 AMP 事件启动保护；后续计数也只发生在 Joy 回调。
  armVelocityNeutralGuard(triggers);

  // 4.1 长按绑定检测（hold_duration > 0；不受 has_prev_state_ 限制，首帧即可计时）
  processHoldBinding(current_combo, walking, current_time, triggers);

  // 5. 拦截 SetInputMask（adapter 内部消费，不下发 TriggerBuffer）
  consumeInputMaskTriggers(triggers);

  // 6. 处理摇杆/方向键（连续控制）-> 写入内部 CommandBuffer（受屏蔽状态影响）
  handleCmdVelLineXLimitDpad(joy, prev);
  processVelocityImpl(joy, cmd_buffer_, current_time);

  // 7. 将剩余触发器推入 TriggerBuffer
  if (!triggers.empty()) {
    for (const auto& trigger : triggers) {
      RL_LOGD("JoyTeleopAdapter: Pushing trigger type=%d", static_cast<int>(trigger.type));
      trigger_buffer_->push(trigger);
    }
  }

  // 8. 保存当前状态供下一帧使用
  prev_combo_ = current_combo;
  has_prev_state_ = true;
}

bool JoyTeleopAdapter::isWalking(const JoyData& joy) const {
  // 遥杆任一轴越过死区，即视为走路
  if (std::abs(applyDeadzone(joy.axes.left_x)) > 0.0f) return true;
  if (std::abs(applyDeadzone(joy.axes.left_y)) > 0.0f) return true;
  if (std::abs(applyDeadzone(joy.axes.right_x)) > 0.0f) return true;
  return false;
}

void JoyTeleopAdapter::processComboEdges(const ComboKey& prev_combo,
                                         const ComboKey& current_combo,
                                         bool walking,
                                         std::vector<ActionTrigger>& out_triggers) {
  if (prev_combo == current_combo) {
    return;  // 无边沿变化
  }

  const auto& device_config = getDeviceConfig();

  // applyBinding 升级版：block_when_walking + MotionCommand 时传播标记
  auto applyWithBlockFlag = [this](const TeleopBinding& b,
                                    std::vector<ActionTrigger>& out) {
    if (b.block_when_walking && b.action.type == ActionType::MotionCommand &&
        b.action.args) {
      auto* mc = dynamic_cast<MotionCommandArgs*>(b.action.args.get());
      if (mc) {
        auto new_args = std::make_shared<MotionCommandArgs>(*mc);
        new_args->block_velocity_during_motion = true;
        out.push_back(ActionTrigger(ActionType::MotionCommand, new_args));
        return;
      }
    }
    applyBinding(b, out);
  };

  // 松开边沿：上一帧组合键被释放，触发其 release 绑定
  // （hold_duration > 0 的绑定由 processHoldBinding 处理，跳过）
  if (const auto* prev_binding = device_config.findBinding(prev_combo)) {
    if (prev_binding->edge == TriggerEdge::kRelease &&
        prev_binding->hold_duration <= 0.0 &&
        !(prev_binding->block_when_walking && walking)) {
      RL_LOGD("JoyTeleopAdapter: Release edge for combo [%s]",
              ComboKeyToString(prev_combo).c_str());
      applyWithBlockFlag(*prev_binding, out_triggers);
    }
  }

  // 按下边沿：当前帧组合键被按下，触发其 press 绑定
  if (const auto* cur_binding = device_config.findBinding(current_combo)) {
    if (cur_binding->edge == TriggerEdge::kPress &&
        cur_binding->hold_duration <= 0.0 &&
        !(cur_binding->block_when_walking && walking)) {
      RL_LOGD("JoyTeleopAdapter: Press edge for combo [%s]",
              ComboKeyToString(current_combo).c_str());
      applyWithBlockFlag(*cur_binding, out_triggers);
    }
  }
}

void JoyTeleopAdapter::processHoldBinding(const ComboKey& current_combo,
                                          bool walking,
                                          double current_time,
                                          std::vector<ActionTrigger>& out_triggers) {
  const auto* binding = getDeviceConfig().findBinding(current_combo);
  if (!binding || binding->hold_duration <= 0.0) {
    hold_combo_key_.clear();
    hold_fired_ = false;
    return;
  }

  const std::string key = ComboKeyToString(current_combo);
  if (key != hold_combo_key_) {
    hold_combo_key_ = key;
    hold_start_time_ = current_time;
    hold_fired_ = false;
    return;
  }

  if (hold_fired_ || current_time - hold_start_time_ < binding->hold_duration) {
    return;
  }
  if (binding->block_when_walking && walking) {
    return;  // 走路门控：保持计时但不触发（松开即重置）
  }

  hold_fired_ = true;
  RL_LOGD("JoyTeleopAdapter: Hold %.1fs reached for combo [%s]",
          binding->hold_duration, key.c_str());
  applyBinding(*binding, out_triggers);
}

void JoyTeleopAdapter::consumeInputMaskTriggers(std::vector<ActionTrigger>& triggers) {
  auto it = triggers.begin();
  while (it != triggers.end()) {
    if (it->type != ActionType::SetInputMask) {
      ++it;
      continue;
    }
    std::string mode = "toggle";
    if (auto* named = dynamic_cast<NamedArgs*>(it->args.get())) {
      if (!named->name.empty()) mode = named->name;
    }
    if (mode == "on") {
      input_masked_ = true;
    } else if (mode == "off") {
      input_masked_ = false;
    } else {  // toggle
      input_masked_ = !input_masked_;
    }
    RL_LOGI("JoyTeleopAdapter: Input mask -> %s", input_masked_ ? "ON" : "OFF");
    it = triggers.erase(it);  // 内部消费，不下发
  }
}

void JoyTeleopAdapter::armVelocityNeutralGuard(
    const std::vector<ActionTrigger>& triggers) {
  if (!config_.velocity_entry_neutral_guard.enabled) {
    return;
  }

  bool should_arm = false;
  const char* reason = nullptr;
  for (const auto& trigger : triggers) {
    if (trigger.type == ActionType::Start &&
        !start_neutral_guard_consumed_) {
      start_neutral_guard_consumed_ = true;
      should_arm = true;
      reason = "START";
      break;
    }
    if (trigger.type == ActionType::SwitchController) {
      const auto* named = dynamic_cast<const NamedArgs*>(trigger.args.get());
      if (named && named->name == "amp") {
        should_arm = true;
        reason = "switch-to-amp";
        break;
      }
    }
  }

  if (!should_arm) {
    return;
  }
  velocity_neutral_guard_active_ = true;
  velocity_neutral_frames_ = 0;
  neutral_guard_nonzero_logged_ = false;
  RL_LOGI("JoyTeleopAdapter: velocity neutral guard armed by %s; "
          "waiting for %d real Joy frames",
          reason, config_.velocity_entry_neutral_guard.neutral_frames);
}

bool JoyTeleopAdapter::shouldHoldVelocityForNeutral(const JoyData& joy) {
  if (!velocity_neutral_guard_active_) {
    return false;
  }

  const bool neutral =
      applyDeadzone(joy.axes.left_x) == 0.0f &&
      applyDeadzone(joy.axes.left_y) == 0.0f &&
      applyDeadzone(joy.axes.right_x) == 0.0f &&
      applyDeadzone(joy.axes.right_y) == 0.0f;
  if (!neutral) {
    velocity_neutral_frames_ = 0;
    if (!neutral_guard_nonzero_logged_) {
      neutral_guard_nonzero_logged_ = true;
      RL_LOGW("JoyTeleopAdapter: velocity neutral guard sees non-neutral axes "
              "(lx=%.3f, ly=%.3f, rx=%.3f, ry=%.3f)",
              joy.axes.left_x, joy.axes.left_y,
              joy.axes.right_x, joy.axes.right_y);
    }
    return true;
  }

  ++velocity_neutral_frames_;
  if (velocity_neutral_frames_ <
      config_.velocity_entry_neutral_guard.neutral_frames) {
    return true;
  }

  velocity_neutral_guard_active_ = false;
  velocity_neutral_frames_ = 0;
  RL_LOGI("JoyTeleopAdapter: velocity input unlocked after real Joy neutral");
  return false;
}

void JoyTeleopAdapter::handleArmModeToggle(const ComboKey& current_combo,
                                           std::vector<ActionTrigger>& out_triggers) {
  if (!has_prev_state_) {
    return;
  }
  // LB+B 组合的规范字符串（按字母序）。组合键无法用静态 yaml 绑定表达，
  // 与 LB+A 输入屏蔽一样由 adapter 内部处理，复用已实现的 SetArmMode 下游接口。
  static const std::string kArmToggleCombo = "B+LB";
  const bool was = (ComboKeyToString(prev_combo_) == kArmToggleCombo);
  const bool now = (ComboKeyToString(current_combo) == kArmToggleCombo);
  if (now && !was) {
    // 翻转决策由 ControlLogic 基于实际当前模式解析（kKeepPose→auto，否则→keep_pose）。
    // adapter 侧不维护本地翻转状态：TactPlayer 等组件会绕过手柄改模式，
    // 本地状态失步后会误发 auto 导致手臂复位而非冻结。
    out_triggers.push_back(MakeSetArmModeTrigger("toggle_keep_pose"));
    RL_LOGI("JoyTeleopAdapter: LB+B 手臂冻结翻转请求");
  }
}

void JoyTeleopAdapter::handleArmUnlock(const ComboKey& current_combo,
                                       std::vector<ActionTrigger>& out_triggers) {
  if (!has_prev_state_) {
    return;
  }
  // LB+Y 组合：三态轮转（auto→external→keep_pose→auto）
  static const std::string kArmUnlockCombo = "LB+Y";
  const bool was = (ComboKeyToString(prev_combo_) == kArmUnlockCombo);
  const bool now = (ComboKeyToString(current_combo) == kArmUnlockCombo);
  if (now && !was) {
    out_triggers.push_back(MakeSetArmModeTrigger("cycle_arm_mode"));
    RL_LOGI("JoyTeleopAdapter: LB+Y 手臂模式轮转请求");
    if (dexterous_hands_available_.load()) {
      leju::HandCmd hand_cmd;
      for (std::size_t i = 0; i < kDexterousHandDof; ++i) {
        hand_cmd.position[i] = kDefaultOpenHandPose[i];
        hand_cmd.position[kDexterousHandDof + i] = kDefaultOpenHandPose[i];
      }
      hand_cmd.timestamp = leju::common::GetUnixTimestampS();
      if (leju::GlobalRobot::getInstance().publishHandCmd(hand_cmd)) {
        RL_LOGI("JoyTeleopAdapter: LB+Y restored dexterous hands to default open pose");
      } else {
        RL_LOGW("JoyTeleopAdapter: LB+Y failed to restore dexterous hands");
      }
    } else {
      RL_LOGD("JoyTeleopAdapter: LB+Y hand restore skipped, dexterous hands unavailable");
    }
  }
}

void JoyTeleopAdapter::handleHandGripToggle(const ComboKey& current_combo) {
  if (!has_prev_state_) {
    return;
  }
  static const std::string kHandGripCombo = "LB+X";
  const bool was = (ComboKeyToString(prev_combo_) == kHandGripCombo);
  const bool now = (ComboKeyToString(current_combo) == kHandGripCombo);
  if (now && !was) {
    hand_grip_toggled_ = !hand_grip_toggled_;
    leju::HandCmd cmd;
    const auto& pose = hand_grip_toggled_ ? kFullyClosedHandPose : kFullyOpenHandPose;
    for (std::size_t i = 0; i < kDexterousHandDof; ++i) {
      cmd.position[i] = pose[i];
      cmd.position[kDexterousHandDof + i] = pose[i];
    }
    cmd.timestamp = leju::common::GetUnixTimestampS();
    leju::GlobalRobot::getInstance().publishHandCmd(cmd);
    RL_LOGI("JoyTeleopAdapter: LB+X 灵巧手 %s", hand_grip_toggled_ ? "闭合" : "张开");
  }
}

void JoyTeleopAdapter::processSystemButtons(const JoyData& joy,
                                              const JoyData::Buttons& prev,
                                              std::vector<ActionTrigger>& out_triggers) {
  const bool back_edge  = joy.buttons.back  && !prev.back;
  const bool start_edge = joy.buttons.start && !prev.start;

  // 同帧检测：任意一边沿命中且当前帧两键都按下。
  if ((back_edge || start_edge) && joy.buttons.back && joy.buttons.start) {
    if (config_.quit_squat.enabled) {
      // 先下蹲再退出：交由 ControlLoop 执行（下蹲命令 + 超时 + 退出），
      // 适配器只负责发出请求，避免时序依赖手柄帧。
      out_triggers.push_back(MakeQuitSquatTrigger(config_.quit_squat.squat_height,
                                                  config_.quit_squat.duration_sec));
      RL_LOGI("JoyTeleopAdapter: BACK+START => request quit-squat (%.2fm, %.1fs)",
              config_.quit_squat.squat_height, config_.quit_squat.duration_sec);
    } else {
      // 未启用下蹲 → 保持原行为立即 Quit
      out_triggers.push_back(ActionTrigger(ActionType::Quit));
      RL_LOGI("JoyTeleopAdapter: BACK+START combo => Quit");
    }
    return;  // 绝不再发 Start
  }

  if (start_edge && !joy.buttons.back) {
    out_triggers.push_back(ActionTrigger(ActionType::Start));
    RL_LOGI("JoyTeleopAdapter: START button pressed");
  }
}

// ============================================================================
// CRTP 钩子方法实现
// ============================================================================

double JoyTeleopAdapter::getCmdVelLineXLimitForLevel(int level) const {
  const auto& gear_cfg = config_.amp_hand_cmd_vel_line_x_gear;
  if (level <= 0) {
    return gear_cfg.policy_linear_x_low;
  }
  if (level >= 2) {
    return gear_cfg.policy_linear_x_up;
  }
  return gear_cfg.policy_linear_x;
}

double JoyTeleopAdapter::getEffectiveMaxLinearXForward() const {
  const auto& gear_cfg = config_.amp_hand_cmd_vel_line_x_gear;
  if (!gear_cfg.enabled || gear_cfg.policy_linear_x <= 0.0) {
    return config_.max_linear_x;
  }
  // velocity_scale.walking.linear_x* 即 policy 输入上限，填多少就是多少
  return getCmdVelLineXLimitForLevel(cmd_vel_line_x_limit_level_);
}

double JoyTeleopAdapter::getEffectiveMaxLinearXBackward() const {
  const auto& gear_cfg = config_.amp_hand_cmd_vel_line_x_gear;
  if (gear_cfg.enabled) {
    return gear_cfg.policy_linear_x_negative;
  }
  return config_.max_linear_x;
}

double JoyTeleopAdapter::resolveMaxLinearX(double signed_axis) const {
  return signed_axis < 0.0 ? getEffectiveMaxLinearXBackward()
                           : getEffectiveMaxLinearXForward();
}

void JoyTeleopAdapter::handleCmdVelLineXLimitDpad(const JoyData& joy,
                                                   const JoyData::Buttons& prev) {
  if (!config_.amp_hand_cmd_vel_line_x_gear.enabled) {
    return;
  }

  const bool dpad_up_edge = joy.buttons.dpad_up != 0 && prev.dpad_up == 0;
  const bool dpad_down_edge = joy.buttons.dpad_down != 0 && prev.dpad_down == 0;
  if (!dpad_up_edge && !dpad_down_edge) {
    return;
  }

  if (dpad_down_edge) {
    cmd_vel_line_x_limit_level_ = 0;
  } else if (cmd_vel_line_x_limit_level_ < 2) {
    ++cmd_vel_line_x_limit_level_;
  }

  const double limit = getCmdVelLineXLimitForLevel(cmd_vel_line_x_limit_level_);
  const char* level_name = cmd_vel_line_x_limit_level_ <= 0
                               ? "low"
                               : (cmd_vel_line_x_limit_level_ >= 2 ? "up"
                                                                   : "default");
  RL_LOGI("JoyTeleopAdapter: cmdVelLineX limit level=%s, limit=%.2f",
          level_name, limit);
}

void JoyTeleopAdapter::processVelocityImpl(const JoyData& joy,
                                            CommandBuffer& buffer,
                                            double /*current_time*/) {
  MotionCommand cmd;
  cmd.valid = true;

  // 输入屏蔽（LB+A）：遥杆与方向键全部失效，输出零速度
  if (input_masked_) {
    cmd.linear_x = 0.0;
    cmd.linear_y = 0.0;
    cmd.angular_z = 0.0;
    buffer.writeCmdVel(cmd);
    return;
  }

  if (shouldHoldVelocityForNeutral(joy)) {
    cmd.setZero();
    cmd.valid = true;
    buffer.writeCmdVel(cmd);
    return;
  }

  const float left_y = applyDeadzone(joy.axes.left_y);
  const float left_x = applyDeadzone(joy.axes.left_x);
  const float right_x = applyDeadzone(joy.axes.right_x);
  const float right_y = applyDeadzone(joy.axes.right_y);
  const double max_linear_x_forward = getEffectiveMaxLinearXForward();
  const double max_linear_x_backward = getEffectiveMaxLinearXBackward();

  const bool rt_active = joy.axes.right_trigger >= config_.trigger_threshold;
  if (config_.amp_hand_posture_axis.enabled && rt_active) {
    // RT 按住：右摇杆仅用于头控（ControllerManager），此处跳过 posture 轴解析。
    if (!rt_was_active_) {
      frozen_squat_cmd_ = last_published_angular_z_;
    }
    rt_was_active_ = true;

    const double signed_linear_x_axis = -left_y;
    cmd.linear_y = -left_x * config_.max_linear_y;
    cmd.cmd_stance_valid = true;

    if (posture_control_mode_) {
      cmd.cmd_stance_mode = 1;
      cmd.angular_z = frozen_squat_cmd_;
      cmd.linear_x = 0.0;
    } else {
      cmd.cmd_stance_mode = 0;
      cmd.angular_z = 0.0;
      cmd.linear_x = signed_linear_x_axis *
                     resolveMaxLinearX(signed_linear_x_axis);
    }

    last_published_angular_z_ = cmd.angular_z;
    buffer.writeCmdVel(cmd);
    return;
  }
  rt_was_active_ = false;

  if (config_.amp_hand_posture_axis.enabled) {
    processAmpHandPostureAxis(left_y, left_x, right_x, right_y,
                              max_linear_x_forward, max_linear_x_backward, cmd);
    last_published_angular_z_ = cmd.angular_z;
    buffer.writeCmdVel(cmd);
    return;
  }

  const double signed_linear_x_axis = -left_y;
  cmd.linear_x = signed_linear_x_axis *
                 resolveMaxLinearX(signed_linear_x_axis);
  cmd.linear_y = -left_x * config_.max_linear_y;
  cmd.angular_z = -right_x * config_.max_angular_z;

  last_published_angular_z_ = cmd.angular_z;
  buffer.writeCmdVel(cmd);
}

void JoyTeleopAdapter::processAmpHandPostureAxis(float left_y, float left_x,
                                                 float right_x, float right_y,
                                                 double max_linear_x_forward,
                                                 double max_linear_x_backward,
                                                 MotionCommand& cmd) {
  const auto& posture_cfg = config_.amp_hand_posture_axis;
  const double threshold = posture_cfg.axis_threshold;
  // 当前手柄 right_y 下推为正；posture 语义保持正=站起、负=下蹲。
  const double a_squat = -right_y;

  double mapped = 0.0;
  if (std::abs(a_squat) > threshold) {
    mapped = (a_squat + (a_squat > 0.0 ? -threshold : threshold)) /
             (1.0 - threshold);
  }
  const double squat_cmd =
      mapped < 0.0 ? mapped * std::abs(posture_cfg.squat_height_min)
                   : mapped * posture_cfg.squat_height_max;

  const double signed_linear_x_axis = -left_y;
  const double max_linear_x =
      signed_linear_x_axis < 0.0 ? max_linear_x_backward : max_linear_x_forward;
  const double cmd_x = std::abs(signed_linear_x_axis) * max_linear_x;
  const double cmd_y = std::abs(-left_x) * config_.max_linear_y;
  const double cmd_ang_z = std::abs(-right_x) * config_.max_angular_z;
  const double block_threshold = posture_cfg.walk_cmd_block_threshold;
  const bool posture_axis_active = std::abs(a_squat) > threshold;
  const double policy_turn_cmd =
      cmd_ang_z * posture_cfg.policy_angular_z_scale;
  // 深蹲守备：下蹲轴仍在深蹲区间时，不因走/转指令退出 posture（对齐闭源 cmd_z<-0.05）
  const bool deep_squat_hold =
      posture_control_mode_ && squat_cmd < posture_cfg.deep_squat_guard;
  const bool small_turn_in_posture =
      posture_control_mode_ && cmd_ang_z >= block_threshold &&
      policy_turn_cmd < posture_cfg.turn_exit_guard_threshold;

  // 下蹲轴区间内忽略左摇杆前后；下压过程中忽略右摇杆 X 避免左右偏误触发退出。
  // 下蹲完成（轴回死区）或浅蹲时，右摇杆左右可像闭源一样退出 posture 并转身。
  const bool turn_blocks_posture =
      cmd_ang_z >= block_threshold &&
      !(posture_control_mode_ && (posture_axis_active || deep_squat_hold)) &&
      !small_turn_in_posture;
  const bool walk_cmd_blocks_posture =
      cmd_y >= block_threshold ||
      (cmd_x >= block_threshold &&
       !(posture_control_mode_ && posture_axis_active)) ||
      turn_blocks_posture;

  if (walk_cmd_blocks_posture) {
    if (!deep_squat_hold) {
      if (posture_control_mode_) {
        posture_control_mode_ = false;
        RL_LOGI("JoyTeleopAdapter: Exit posture mode due to walk/turn command");
      }
      cmd.linear_x = signed_linear_x_axis *
                     resolveMaxLinearX(signed_linear_x_axis);
      cmd.linear_y = -left_x * config_.max_linear_y;
      cmd.angular_z = -right_x * config_.max_angular_z;
      cmd.cmd_stance_valid = true;
      cmd.cmd_stance_mode = 0;
      return;
    }
  }

  if (std::abs(a_squat) <= threshold) {
    if (posture_control_mode_) {
      // 已在姿态模式：死区内保持 cmd_stance=1，仅将高度命令归零，
      // 避免下推→回中→上推经过死区时反复退出/进入导致平滑状态被重置。
      cmd.cmd_stance_valid = true;
      cmd.cmd_stance_mode = 1;
      cmd.linear_x = 0.0;
      cmd.linear_y = -left_x * config_.max_linear_y;
      cmd.angular_z = 0.0;
      return;
    }
    cmd.linear_x = signed_linear_x_axis *
                   resolveMaxLinearX(signed_linear_x_axis);
    cmd.linear_y = -left_x * config_.max_linear_y;
    cmd.angular_z = -right_x * config_.max_angular_z;
    cmd.cmd_stance_valid = true;
    cmd.cmd_stance_mode = 0;
    return;
  }

  if (!posture_control_mode_) {
    posture_control_mode_ = true;
    RL_LOGI("JoyTeleopAdapter: Enter posture mode (right_y=%.2f)", right_y);
  }

  cmd.cmd_stance_valid = true;
  cmd.cmd_stance_mode = 1;
  cmd.linear_x = 0.0;
  cmd.linear_y = -left_x * config_.max_linear_y;
  cmd.angular_z = squat_cmd;
}

ComboKey JoyTeleopAdapter::detectCurrentComboImpl(const JoyData& joy) const {
  ComboKey combo;
  const auto& btns = joy.buttons;

  // LT/RT 是模拟扳机（axes），超过阈值视为虚拟按键 "LT"/"RT"
  if (joy.axes.left_trigger >= config_.trigger_threshold) combo.buttons.push_back("LT");
  if (joy.axes.right_trigger >= config_.trigger_threshold) combo.buttons.push_back("RT");

  if (btns.south != 0) combo.buttons.push_back("A");
  if (btns.east != 0) combo.buttons.push_back("B");
  if (btns.west != 0) combo.buttons.push_back("X");
  if (btns.north != 0) combo.buttons.push_back("Y");
  if (btns.guide != 0) combo.buttons.push_back("GUIDE");
  if (btns.left_stick != 0) combo.buttons.push_back("L3");
  if (btns.right_stick != 0) combo.buttons.push_back("R3");
  if (btns.left_shoulder != 0) combo.buttons.push_back("LB");
  if (btns.right_shoulder != 0) combo.buttons.push_back("RB");
  if (btns.dpad_up != 0) combo.buttons.push_back("DPAD_UP");
  if (btns.dpad_down != 0) combo.buttons.push_back("DPAD_DOWN");
  if (btns.dpad_left != 0) combo.buttons.push_back("DPAD_LEFT");
  if (btns.dpad_right != 0) combo.buttons.push_back("DPAD_RIGHT");
  if (btns.misc1 != 0) combo.buttons.push_back("MISC");
  if (btns.misc2 != 0) combo.buttons.push_back("MISC2");
  // 注意：START 和 BACK 在系统级处理，不包含在组合键中

  return combo;
}

} // namespace runtime
} // namespace leju
