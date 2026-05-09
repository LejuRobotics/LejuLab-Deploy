/**
 * @file joy_teleop_adapter.cpp
 * @brief Joy 手柄输入适配器实现（配置驱动组合键）
 */

#include "leju-rl-controller/runtime/input/teleop/joy_teleop_adapter.h"

#include <chrono>
#include <cmath>

#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-lowlevel/leju_sdk.h"

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

  // 当前时间戳
  double current_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

  // 1. 处理摇杆（连续控制）- 写入内部 CommandBuffer
  processVelocityImpl(joy, cmd_buffer_, current_time);

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

  // 4. 处理组合键绑定（边缘触发）
  if (has_prev_state_) {
    // 使用基类的默认边缘检测策略（变化即触发）
    if (!(prev_combo_ == current_combo)) {
      RL_LOGD("JoyTeleopAdapter: Edge detected, applying binding for combo [%s]",
              ComboKeyToString(current_combo).c_str());
      size_t triggers_before = triggers.size();
      applyBindingForCombo(current_combo, triggers);
      RL_LOGD("JoyTeleopAdapter: Added %zu triggers", triggers.size() - triggers_before);
    }
  }

  // 5. 将触发器推入 TriggerBuffer
  if (!triggers.empty()) {
    for (const auto& trigger : triggers) {
      RL_LOGD("JoyTeleopAdapter: Pushing trigger type=%d", static_cast<int>(trigger.type));
      trigger_buffer_->push(trigger);
    }
  }

  // 6. 保存当前状态供下一帧使用
  prev_combo_ = current_combo;
  has_prev_state_ = true;
}

void JoyTeleopAdapter::processSystemButtons(const JoyData& joy,
                                              const JoyData::Buttons& prev,
                                              std::vector<ActionTrigger>& out_triggers) {
  const bool back_edge  = joy.buttons.back  && !prev.back;
  const bool start_edge = joy.buttons.start && !prev.start;

  // 同帧检测：任意一边沿命中且当前帧两键都按下 → 立即 Quit，绝不再发 Start
  if ((back_edge || start_edge) && joy.buttons.back && joy.buttons.start) {
    out_triggers.push_back(ActionTrigger(ActionType::Quit));
    RL_LOGI("JoyTeleopAdapter: BACK+START combo => Quit");
    return;
  }

  if (start_edge && !joy.buttons.back) {
    out_triggers.push_back(ActionTrigger(ActionType::Start));
    RL_LOGI("JoyTeleopAdapter: START button pressed");
  }
}

// ============================================================================
// CRTP 钩子方法实现
// ============================================================================

void JoyTeleopAdapter::processVelocityImpl(const JoyData& joy,
                                            CommandBuffer& buffer,
                                            double /*current_time*/) {
  // 应用死区并计算速度
  float left_y = applyDeadzone(joy.axes.left_y);
  float left_x = applyDeadzone(joy.axes.left_x);
  float right_x = applyDeadzone(joy.axes.right_x);

  // 创建速度指令
  MotionCommand cmd;
  cmd.linear_x = -left_y * config_.max_linear_x;
  cmd.linear_y = -left_x * config_.max_linear_y;
  cmd.angular_z = -right_x * config_.max_angular_z;
  cmd.valid = true;

  // 写入 CommandBuffer
  buffer.writeCmdVel(cmd);
}

ComboKey JoyTeleopAdapter::detectCurrentComboImpl(const JoyData& joy) const {
  ComboKey combo;
  const auto& btns = joy.buttons;

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
  // 注意：START 和 BACK 在系统级处理，不包含在组合键中

  return combo;
}

} // namespace runtime
} // namespace leju
