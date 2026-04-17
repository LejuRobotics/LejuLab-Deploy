/**
 * @file quest_teleop_adapter.cpp
 * @brief Quest VR 输入适配器实现（配置驱动组合键）
 */

#include "leju-rl-controller/runtime/input/teleop/quest_teleop_adapter.h"

#include <chrono>
#include <cmath>

#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace runtime {

// ============================================================================
// 构造/析构
// ============================================================================

QuestTeleopAdapter::QuestTeleopAdapter(const RobotVersion& version,
                                       TriggerBuffer* trigger_buffer)
    : TeleopAdapterBase<QuestTeleopAdapter, QuestJoystickData>(trigger_buffer),
      robot_version_(version) {}

QuestTeleopAdapter::~QuestTeleopAdapter() {
  shutdown();
}


// ============================================================================
// 初始化和关闭
// ============================================================================

bool QuestTeleopAdapter::initialize() {
  if (running_.load()) {
    return true;
  }

  if (!trigger_buffer_) {
    RL_LOGE("QuestTeleopAdapter: TriggerBuffer not set");
    return false;
  }

  // 创建并初始化 VR API
  vr_api_ = std::make_unique<vr::VRBaseAPI>(robot_version_);
  if (!vr_api_->initialize()) {
    RL_LOGE("QuestTeleopAdapter: Failed to initialize VR API");
    vr_api_.reset();
    return false;
  }

  running_ = true;

  // 订阅 Quest 手柄数据
  vr_api_->subscribeQuestJoystickData([this](const vr::QuestJoystickData& data) {
    if (!running_.load()) return;
    this->onQuestData(data);
  });

  RL_LOGI("QuestTeleopAdapter: Initialized and subscribed to Quest data");
  return true;
}

void QuestTeleopAdapter::shutdown() {
  if (!running_.load()) {
    return;
  }

  running_ = false;

  if (vr_api_) {
    vr_api_->shutdown();
    vr_api_.reset();
  }

  RL_LOGI("QuestTeleopAdapter: Shutdown");
}

bool QuestTeleopAdapter::isInitialized() const {
  return running_.load() && vr_api_ && vr_api_->isInitialized();
}

// ============================================================================
// Quest 数据处理（内部回调）
// ============================================================================

void QuestTeleopAdapter::onQuestData(const QuestJoystickData& data) {
  if (!trigger_buffer_) {
    return;
  }

  // 当前时间戳
  double current_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

  // 1. 处理摇杆（连续控制）- 写入内部 CommandBuffer
  processVelocityImpl(data, cmd_buffer_, current_time);

  // 2. 识别当前组合键
  ComboKey current_combo = detectCurrentComboImpl(data);

  // 3. 处理组合键绑定（边缘触发）
  std::vector<ActionTrigger> triggers;
  if (has_prev_state_) {
    // 使用 Quest 特有的边缘检测策略（空->非空）
    if (shouldTriggerCombo(prev_combo_, current_combo)) {
      applyBindingForCombo(current_combo, triggers);
    }
  }

  // 4. 将触发器推入 TriggerBuffer
  if (!triggers.empty()) {
    for (const auto& trigger : triggers) {
      trigger_buffer_->push(trigger);
    }
  }

  // 5. 保存当前状态供下一帧使用
  prev_combo_ = current_combo;
  has_prev_state_ = true;
}

// ============================================================================
// CRTP 钩子方法实现
// ============================================================================

void QuestTeleopAdapter::processVelocityImpl(const QuestJoystickData& data,
                                             CommandBuffer& buffer,
                                             double /*current_time*/) {
  // 一次性合成完整速度，避免右摇杆写入覆盖左摇杆的平移分量。
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

void QuestTeleopAdapter::processLeftController(const QuestJoystickData& data,
                                               CommandBuffer& buffer,
                                               double /*current_time*/) {
  (void)buffer;  // 使用内部 cmd_buffer_

  // 应用死区并计算速度
  float left_x = applyDeadzone(data.left_x);
  float left_y = applyDeadzone(data.left_y);

  // Quest 坐标系映射：
  // - left_x (左摇杆 X) -> linear_y (侧向，右为正)
  // - left_y (左摇杆 Y) -> linear_x (前进，前为正)
  MotionCommand cmd;
  cmd.linear_x = left_y * config_.max_linear_x;
  cmd.linear_y = -left_x * config_.max_linear_y;
  cmd.angular_z = 0.0;  // 由右控制器处理
  cmd.valid = true;
  cmd_buffer_.writeCmdVel(cmd);
}

void QuestTeleopAdapter::processRightController(const QuestJoystickData& data,
                                                CommandBuffer& buffer,
                                                double /*current_time*/) {
  (void)buffer;  // 使用内部 cmd_buffer_

  // 应用死区并计算速度
  float right_x = applyDeadzone(data.right_x);

  // Quest 坐标系映射：
  // - right_x (右摇杆 X) -> angular_z (旋转)
  // 注意：angular_z 是与 linear_x/y 合并的，所以需要先读取当前值
  // 为简化，直接覆盖（假设左控制器先处理）
  MotionCommand cmd;
  cmd.linear_x = 0.0;  // 由左控制器处理
  cmd.linear_y = 0.0;  // 由左控制器处理
  cmd.angular_z = -right_x * config_.max_angular_z;
  cmd.valid = true;
  cmd_buffer_.writeCmdVel(cmd);
}

ComboKey QuestTeleopAdapter::detectCurrentComboImpl(const QuestJoystickData& data) const {
  ComboKey combo;

  // 扳机（使用阈值判断）
  if (data.left_trigger > 0.5f) combo.buttons.push_back("LEFT_TRIGGER");
  if (data.left_grip > 0.5f) combo.buttons.push_back("LEFT_GRIP");
  if (data.right_trigger > 0.5f) combo.buttons.push_back("RIGHT_TRIGGER");
  if (data.right_grip > 0.5f) combo.buttons.push_back("RIGHT_GRIP");

  // 按钮
  if (data.left_first_button_pressed) combo.buttons.push_back("LEFT_FIRST");
  if (data.left_second_button_pressed) combo.buttons.push_back("LEFT_SECOND");
  if (data.right_first_button_pressed) combo.buttons.push_back("RIGHT_FIRST");
  if (data.right_second_button_pressed) combo.buttons.push_back("RIGHT_SECOND");

  return combo;
}

bool QuestTeleopAdapter::shouldTriggerCombo(const ComboKey& prev,
                                            const ComboKey& current) const {
  // Quest 特有的边缘检测策略：只在组合键从空变为非空时触发
  // （即按下时触发，而不是释放时）
  return prev.empty() && !current.empty();
}

}  // namespace runtime
}  // namespace leju
