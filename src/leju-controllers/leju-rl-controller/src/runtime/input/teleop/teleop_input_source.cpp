/**
 * @file teleop_input_source.cpp
 * @brief 遥操作输入源实现
 */

#include "leju-rl-controller/runtime/input/teleop/teleop_input_source.h"

#include <algorithm>

#include "leju-rl-controller/runtime/input/teleop/joy_teleop_adapter.h"
#include "leju-rl-controller/runtime/input/teleop/quest_teleop_adapter.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding_config.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-utils/robot_version.hpp"

namespace leju {
namespace runtime {

// ============================================================================
// 构造函数/析构函数
// ============================================================================

TeleopInputSource::TeleopInputSource(const RobotVersion& version,
                                     TriggerBuffer* trigger_buffer)
    : trigger_buffer_(trigger_buffer),
      joy_adapter_(std::make_unique<JoyTeleopAdapter>(trigger_buffer)),
      quest_adapter_(std::make_unique<QuestTeleopAdapter>(version, trigger_buffer)) {
  RL_LOGI("TeleopInputSource: Created Joy and Quest adapters");
}

TeleopInputSource::~TeleopInputSource() {
  shutdown();
}

// ============================================================================
// 生命周期管理
// ============================================================================

bool TeleopInputSource::initialize() {
  if (initialized_.load()) {
    return true;
  }

  bool any_initialized = false;

  // 初始化 Joy 适配器（需要主动订阅 SDK）
  if (joy_adapter_ && !joy_adapter_->isInitialized()) {
    if (joy_adapter_->initialize()) {
      any_initialized = true;
      RL_LOGI("TeleopInputSource: Initialized Joy adapter");
    } else {
      RL_LOGW("TeleopInputSource: Failed to initialize Joy adapter");
    }
  }

  // 初始化 Quest 适配器（主动订阅 Quest 数据）
  if (quest_adapter_ && !quest_adapter_->isInitialized()) {
    if (quest_adapter_->initialize()) {
      any_initialized = true;
      RL_LOGI("TeleopInputSource: Initialized Quest adapter");
    } else {
      RL_LOGW("TeleopInputSource: Failed to initialize Quest adapter");
    }
  }

  if (any_initialized) {
    initialized_ = true;
    RL_LOGI("TeleopInputSource: Initialization complete");
  }

  return any_initialized;
}

void TeleopInputSource::shutdown() {
  if (!initialized_.load()) {
    return;
  }

  // 关闭 Joy 适配器
  if (joy_adapter_) {
    joy_adapter_->shutdown();
    RL_LOGI("TeleopInputSource: Shutdown Joy adapter");
  }

  // 关闭 Quest 适配器
  if (quest_adapter_) {
    quest_adapter_->shutdown();
    RL_LOGI("TeleopInputSource: Shutdown Quest adapter");
  }

  initialized_ = false;
  RL_LOGI("TeleopInputSource: Shutdown complete");
}

// ============================================================================
// InputSource 接口实现
// ============================================================================

CommandBuffer::Snapshot TeleopInputSource::getSnapshot() const {
  // 按优先级遍历：Joy > Quest
  // 1. 先尝试 Joy
  if (joy_adapter_) {
    auto snapshot = joy_adapter_->getSnapshot();
    if (snapshot.cmd_vel.valid) {
      // RL_LOGD("TeleopInputSource: Using cmd_vel from Joy");
      return snapshot;
    }
  }

  // 2. 再尝试 Quest
  if (quest_adapter_) {
    auto snapshot = quest_adapter_->getSnapshot();
    if (snapshot.cmd_vel.valid) {
      // RL_LOGD("TeleopInputSource: Using cmd_vel from Quest");
      return snapshot;
    }
  }

  // 所有输入源都无效，返回空快照
  return CommandBuffer::Snapshot{};
}

// ============================================================================
// 配置加载
// ============================================================================

bool TeleopInputSource::loadBindingConfig(const std::string& config_path) {
  // 先加载一次配置验证文件有效，避免重复打印错误
  TeleopBindingConfig config;
  if (!config.loadFromFile(config_path)) {
    return false;
  }

  // 将已加载的配置传给各个适配器
  if (joy_adapter_) {
    joy_adapter_->setBindingConfig(config);
    RL_LOGI("TeleopInputSource: Loaded binding config for Joy");
  }

  if (quest_adapter_) {
    quest_adapter_->setBindingConfig(config);
    RL_LOGI("TeleopInputSource: Loaded binding config for Quest");
  }

  return true;
}

}  // namespace runtime
}  // namespace leju
