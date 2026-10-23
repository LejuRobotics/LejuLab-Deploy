/**
 * @file pico_teleop_adapter.cpp
 * @brief Pico VR 输入适配器实现（DDS 直接订阅）
 */

#include "leju-rl-controller/runtime/input/teleop/pico_teleop_adapter.h"

#include <algorithm>
#include <chrono>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/QuestJoysticks.hpp>
#include <lejusdk-topic-pubsub/topic_names.h>
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>

#include "leju-rl-controller/motion/heft_csv_record_gate.h"
#include "leju-rl-controller/motion/heft_stream_freeze_gate.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace runtime {

// ============================================================================
// DDS pimpl
// ============================================================================
struct PicoTeleopAdapter::DdsImpl {
  explicit DdsImpl(PicoTeleopAdapter* owner)
      : participant(0),
        subscriber(participant, dds_topics::kPicoJoysticks,
                   [owner](const msgs::QuestJoysticks& msg) {
                     PicoJoystickData data;
                     data.header_sec = msg.header_sec();
                     data.header_nanosec = msg.header_nanosec();
                     data.left_x = msg.left_x();
                     data.left_y = msg.left_y();
                     data.left_trigger = msg.left_trigger();
                     data.left_grip = msg.left_grip();
                     data.left_first_button_pressed =
                         msg.left_first_button_pressed();
                     data.left_second_button_pressed =
                         msg.left_second_button_pressed();
                     data.left_first_button_touched =
                         msg.left_first_button_touched();
                     data.left_second_button_touched =
                         msg.left_second_button_touched();
                     data.right_x = msg.right_x();
                     data.right_y = msg.right_y();
                     data.right_trigger = msg.right_trigger();
                     data.right_grip = msg.right_grip();
                     data.right_first_button_pressed =
                         msg.right_first_button_pressed();
                     data.right_second_button_pressed =
                         msg.right_second_button_pressed();
                     data.right_first_button_touched =
                         msg.right_first_button_touched();
                     data.right_second_button_touched =
                         msg.right_second_button_touched();
                     owner->onPicoData(data);
                   }) {}

  dds::domain::DomainParticipant participant;
  dds_common::TopicSubscriber<msgs::QuestJoysticks> subscriber;
};

// ============================================================================
// 构造/析构
// ============================================================================
PicoTeleopAdapter::PicoTeleopAdapter(TriggerBuffer* trigger_buffer)
    : TeleopAdapterBase<PicoTeleopAdapter, PicoJoystickData>(trigger_buffer) {}

PicoTeleopAdapter::~PicoTeleopAdapter() {
  shutdown();
}

// ============================================================================
// 初始化和关闭
// ============================================================================
bool PicoTeleopAdapter::initialize() {
  if (running_.load()) {
    return true;
  }

  if (!trigger_buffer_) {
    RL_LOGE("PicoTeleopAdapter: TriggerBuffer not set");
    return false;
  }

  try {
    dds_ = std::make_unique<DdsImpl>(this);
  } catch (const std::exception& e) {
    RL_LOGE("PicoTeleopAdapter: DDS subscription failed: %s", e.what());
    dds_.reset();
    return false;
  } catch (...) {
    RL_LOGE("PicoTeleopAdapter: DDS subscription failed (unknown)");
    dds_.reset();
    return false;
  }

  running_ = true;
  RL_LOGI("PicoTeleopAdapter: Initialized and subscribed to %s",
          dds_topics::kPicoJoysticks);
  return true;
}

void PicoTeleopAdapter::shutdown() {
  if (!running_.load()) {
    return;
  }
  running_ = false;
  dds_.reset();
  RL_LOGI("PicoTeleopAdapter: Shutdown");
}

bool PicoTeleopAdapter::isInitialized() const {
  return running_.load() && dds_ != nullptr;
}

// ============================================================================
// Pico 数据处理（DDS 回调线程）
// ============================================================================
void PicoTeleopAdapter::onPicoData(const PicoJoystickData& data) {
  if (!running_.load() || !trigger_buffer_) {
    return;
  }

  // 整帧持锁: 与热重载 (setBindingConfig) 互斥（与 Quest adapter 一致）
  std::lock_guard<std::mutex> binding_lk(binding_mutex_);

  double current_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

  // 1. 摇杆 → 速度指令
  processVelocityImpl(data, cmd_buffer_, current_time);

  // 2. 组合键识别
  ComboKey current_combo = detectCurrentComboImpl(data);

  // 3. 边缘触发（空->非空）
  std::vector<ActionTrigger> triggers;
  if (has_prev_state_) {
    if (shouldTriggerCombo(prev_combo_, current_combo)) {
      applyBindingForCombo(current_combo, triggers);
    }
  }

  // 4. 推入 TriggerBuffer
  for (const auto& trigger : triggers) {
    trigger_buffer_->push(trigger);
  }

  // 5. LT/RT → 灵巧手整体抓握（与组合键并行，避开前端占用组合）
  handleHandGripFromTriggers(data);

  // 6. LT+Y/X CSV record；RT+Y/X stream freeze (C++ consumption hold)
  handleCsvRecordCombos(data);
  handleStreamFreezeCombos(data);

  // 7. 保存状态
  prev_combo_ = current_combo;
  has_prev_state_ = true;
}

void PicoTeleopAdapter::handleCsvRecordCombos(const PicoJoystickData& data) {
  const bool lt = data.left_trigger > 0.5f;
  const bool rt = data.right_trigger > 0.5f;
  const bool y = data.left_second_button_pressed;
  const bool x = data.left_first_button_pressed;
  const bool y_rising = y && !prev_left_y_;
  const bool x_rising = x && !prev_left_x_;

  // Symmetric with frontend RT+Y/X freeze; require LT and not RT.
  if (lt && !rt && y_rising) {
    HeftCsvRecordGate::instance().requestStart();
    RL_LOGI("PicoTeleopAdapter: LT+Y → HEFT policy CSV record START");
  }
  if (lt && !rt && x_rising) {
    HeftCsvRecordGate::instance().requestStop();
    RL_LOGI("PicoTeleopAdapter: LT+X → HEFT policy CSV record STOP");
  }
}

void PicoTeleopAdapter::handleStreamFreezeCombos(const PicoJoystickData& data) {
  const bool lt = data.left_trigger > 0.5f;
  const bool rt = data.right_trigger > 0.5f;
  const bool y = data.left_second_button_pressed;
  const bool x = data.left_first_button_pressed;
  const bool y_rising = y && !prev_left_y_;
  const bool x_rising = x && !prev_left_x_;
  prev_left_y_ = y;
  prev_left_x_ = x;

  // RT+Y freeze / RT+X resume (require RT and not LT).
  if (rt && !lt && y_rising) {
    HeftStreamFreezeGate::instance().requestFreeze();
    RL_LOGI("PicoTeleopAdapter: RT+Y → HEFT live stream FREEZE");
  }
  if (rt && !lt && x_rising) {
    HeftStreamFreezeGate::instance().requestResume();
    RL_LOGI("PicoTeleopAdapter: RT+X → HEFT live stream RESUME");
  }
}

void PicoTeleopAdapter::handleHandGripFromTriggers(
    const PicoJoystickData& data) {
  // 前端占用组合按下时不驱动对应手整体抓握：
  // RT+B 半身校准、RT+Y 冻结、RT+X 恢复、LT+B 全身校准。
  const bool skip_right =
      data.right_trigger > 0.5f &&
      (data.right_second_button_pressed ||  // RT+B
       data.left_second_button_pressed ||   // RT+Y
       data.left_first_button_pressed);     // RT+X
  const bool skip_left =
      data.left_trigger > 0.5f &&
      (data.right_second_button_pressed ||   // LT+B cali
       data.left_second_button_pressed ||    // LT+Y CSV start
       data.left_first_button_pressed);      // LT+X CSV stop

  auto clamp01 = [](float v) {
    return std::clamp(static_cast<double>(v), 0.0, 1.0);
  };

  double left_cmd = last_left_hand_cmd_;
  double right_cmd = last_right_hand_cmd_;
  if (!skip_left) {
    left_cmd = clamp01(data.left_trigger) * 100.0;
  }
  if (!skip_right) {
    right_cmd = clamp01(data.right_trigger) * 100.0;
  }

  // 拇指副关节（每手 index 1 / ThumbAux）：LG / RG 侧扳机内扣至 50；主关节 index 0 跟 LT/RT。
  // 避开 RG+A / RG+B 控制器切换组合。
  const bool left_thumb_curl = data.left_grip > 0.5f;
  const bool right_thumb_curl =
      data.right_grip > 0.5f &&
      !data.right_first_button_pressed &&   // 避开 RG+A
      !data.right_second_button_pressed;  // 避开 RG+B

  constexpr int kThumbAuxIndex = 1;
  constexpr double kThumbAuxCurl = 50.0;

  // 每帧写入 CommandBuffer（~100Hz），由 ControlLogic 统一按 100Hz 重发 /rt/hand_cmd。
  // 不去重：数值不变也要刷新 seq，才能撑住 control_logic 的 0.25s watchdog。
  last_left_hand_cmd_ = left_cmd;
  last_right_hand_cmd_ = right_cmd;
  last_left_thumb_curl_ = left_thumb_curl;
  last_right_thumb_curl_ = right_thumb_curl;

  ExternalHandTarget target;
  target.position.resize(12);
  for (int i = 0; i < 6; ++i) {
    target.position[static_cast<size_t>(i)] =
        (i == kThumbAuxIndex && left_thumb_curl) ? kThumbAuxCurl : left_cmd;
    target.position[static_cast<size_t>(6 + i)] =
        (i == kThumbAuxIndex && right_thumb_curl) ? kThumbAuxCurl : right_cmd;
  }
  cmd_buffer_.writeHandTarget(target);
}

// ============================================================================
// CRTP 钩子实现
// ============================================================================
void PicoTeleopAdapter::processVelocityImpl(const PicoJoystickData& data,
                                            CommandBuffer& buffer,
                                            double /*current_time*/) {
  // 与 Quest 相同的映射：左摇杆平移，右摇杆旋转
  const float left_x = applyDeadzone(data.left_x);
  const float left_y = applyDeadzone(data.left_y);
  const float right_x = applyDeadzone(data.right_x);

  MotionCommand cmd;
  cmd.linear_x = left_y * config_.max_linear_x;
  cmd.linear_y = -left_x * config_.max_linear_y;
  cmd.angular_z = -right_x * config_.max_angular_z;
  cmd.valid = true;
  buffer.writeCmdVel(cmd);
}

ComboKey PicoTeleopAdapter::detectCurrentComboImpl(
    const PicoJoystickData& data) const {
  ComboKey combo;

  // 扳机/握把（阈值判定，与 Quest 一致）
  if (data.left_trigger > 0.5f) combo.buttons.push_back("LEFT_TRIGGER");
  if (data.left_grip > 0.5f) combo.buttons.push_back("LEFT_GRIP");
  if (data.right_trigger > 0.5f) combo.buttons.push_back("RIGHT_TRIGGER");
  if (data.right_grip > 0.5f) combo.buttons.push_back("RIGHT_GRIP");

  // 按钮：LEFT_FIRST=X, LEFT_SECOND=Y, RIGHT_FIRST=A, RIGHT_SECOND=B
  if (data.left_first_button_pressed) combo.buttons.push_back("LEFT_FIRST");
  if (data.left_second_button_pressed) combo.buttons.push_back("LEFT_SECOND");
  if (data.right_first_button_pressed) combo.buttons.push_back("RIGHT_FIRST");
  if (data.right_second_button_pressed) combo.buttons.push_back("RIGHT_SECOND");

  return combo;
}

bool PicoTeleopAdapter::shouldTriggerCombo(const ComboKey& prev,
                                           const ComboKey& current) const {
  // 组合变化即触发（与 Joy 一致，而非 Quest 的空->非空）：
  // 支持"按住 RG 再按 A"的时序 —— prev=[RG] -> current=[RG,A] 触发，
  // 绑定查找按精确组合匹配，因此松开时的 [RG] 不会误触发。
  return !(prev == current) && !current.empty();
}

}  // namespace runtime
}  // namespace leju
