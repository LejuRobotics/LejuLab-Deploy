#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <vector>

#include "leju-rl-controller/runtime/input/command_buffer.h"
#include "leju-rl-controller/runtime/input/input_source.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding.hpp"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding_config.h"
#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace runtime {

// 前向声明
class TriggerBuffer;

/**
 * @brief 手柄遥控配置参数
 */
struct TeleopConfig {
  float stick_deadzone = 0.05f;     ///< 摇杆死区阈值
  double max_linear_x = 1.0;        ///< 最大前进速度 [m/s]
  double max_linear_y = 0.6;        ///< 最大侧向速度 [m/s]
  double max_angular_z = 0.3;       ///< 最大旋转速度 [rad/s]
};

template<typename Derived, typename InputData>
class TeleopAdapterBase : public InputSource {
public:
  /**
   * @brief 构造函数
   * @param trigger_buffer 触发器缓冲区指针（非拥有，用于输出离散事件）
   */
  explicit TeleopAdapterBase(TriggerBuffer* trigger_buffer = nullptr)
      : trigger_buffer_(trigger_buffer) {}

  ~TeleopAdapterBase() override = default;

  // InputSource 接口实现
  CommandBuffer::Snapshot getSnapshot() const override {
    return cmd_buffer_.getSnapshot();
  }

  // 以下方法需要派生类实现
  InputPriority getPriority() const override = 0;
  const char* getName() const override = 0;

  // 禁止拷贝
  TeleopAdapterBase(const TeleopAdapterBase&) = delete;
  TeleopAdapterBase& operator=(const TeleopAdapterBase&) = delete;

  // 允许移动
  TeleopAdapterBase(TeleopAdapterBase&&) noexcept = default;
  TeleopAdapterBase& operator=(TeleopAdapterBase&&) noexcept = default;

  /**
   * @brief 从 YAML 文件加载组合键绑定配置
   * @param config_path 配置文件路径
   * @return 是否加载成功
   */
  bool loadBindingConfig(const std::string& config_path) {
    return binding_config_.loadFromFile(config_path);
  }

  /**
   * @brief 从已加载的配置对象设置绑定配置
   * @param config 已加载的 TeleopBindingConfig
   */
  void setBindingConfig(const TeleopBindingConfig& config) {
    binding_config_ = config;
  }

  /**
   * @brief 核心处理流程（模板方法模式）
   *
   * 处理流程：
   * 1. 解析摇杆 → 速度指令（调用派生类 processVelocityImpl）
   * 2. 识别当前组合键（调用派生类 detectCurrentComboImpl）
   * 3. 边缘检测（调用 shouldTriggerCombo）
   * 4. 查找并应用绑定
   *
   * @param data 输入数据
   * @param buffer 指令缓冲区（连续控制）
   * @param out_triggers 动作触发器列表（离散事件，输出）
   * @param current_time 当前时间戳 [s]
   */
  void processInput(const InputData& data,
                    CommandBuffer& buffer,
                    std::vector<ActionTrigger>& out_triggers,
                    double current_time) {
    // 1. 速度处理 - 调用派生类实现
    static_cast<Derived*>(this)->processVelocityImpl(data, buffer, current_time);

    // 2. Combo 检测 - 调用派生类实现
    ComboKey current_combo = static_cast<Derived*>(this)->detectCurrentComboImpl(data);

    // 3. 边缘检测 - 可覆盖
    if (!static_cast<Derived*>(this)->shouldTriggerCombo(prev_combo_, current_combo)) {
      prev_combo_ = current_combo;
      has_prev_state_ = true;
      return;
    }

    // 4. 查找并应用绑定
    applyBindingForCombo(current_combo, out_triggers);

    // 5. 保存当前状态供下一帧使用
    prev_combo_ = current_combo;
    has_prev_state_ = true;
  }

protected:
  /**
   * @brief 应用死区处理
   * @param value 原始摇杆值 [-1.0, 1.0]
   * @return 处理后的值
   */
  float applyDeadzone(float value) const {
    if (std::abs(value) < config_.stick_deadzone) {
      return 0.0f;
    }
    float sign = (value > 0.0f) ? 1.0f : -1.0f;
    return sign * (std::abs(value) - config_.stick_deadzone) /
           (1.0f - config_.stick_deadzone);
  }

  /**
   * @brief 默认边缘检测策略（变化即触发）
   *
   * 派生类可覆盖此方法实现特定策略（如 Quest 的空→非空检测）
   *
   * @param prev 上一帧组合键
   * @param current 当前帧组合键
   * @return 是否应触发绑定
   */
  bool shouldTriggerCombo(const ComboKey& prev, const ComboKey& current) const {
    // 默认策略：只要有变化就触发（适用于大多数情况）
    return !(prev == current);
  }

  /**
   * @brief 应用绑定
   * @param binding 绑定配置
   * @param out_triggers 动作触发器列表（离散事件）
   */
  void applyBinding(const TeleopBinding& binding,
                    std::vector<ActionTrigger>& out_triggers) {
    if (binding.action.IsValid()) {
      out_triggers.push_back(binding.action);
    }
  }

  /**
   * @brief 获取设备绑定配置（派生类必须实现此方法）
   *
   * 派生类应返回对应设备的配置引用：
   * - JoyTeleopAdapter: return binding_config_.getJoyConfig();
   * - QuestTeleopAdapter: return binding_config_.getQuestConfig();
   *
   * @return 设备绑定配置
   */

  /**
   * @brief 为指定组合键应用绑定
   * @param combo 组合键
   * @param out_triggers 动作触发器列表
   */
  void applyBindingForCombo(const ComboKey& combo,
                            std::vector<ActionTrigger>& out_triggers) {
    // 通过 CRTP 调用派生类的 getDeviceConfig() 方法
    const auto& device_config = static_cast<const Derived*>(this)->getDeviceConfig();
    const auto* binding = device_config.findBinding(combo);
    if (binding) {
      RL_LOGD("TeleopAdapter: Found binding for combo [%s], action type=%d",
              ComboKeyToString(combo).c_str(), static_cast<int>(binding->action.type));
      applyBinding(*binding, out_triggers);
    } else {
      RL_LOGD("TeleopAdapter: No binding found for combo [%s]",
              ComboKeyToString(combo).c_str());
    }
  }

protected:
  CommandBuffer cmd_buffer_;                ///< 内部命令缓冲区（线程安全）
  TriggerBuffer* trigger_buffer_;           ///< 触发器缓冲区（非拥有）
  TeleopConfig config_;                     ///< 速度控制配置
  TeleopBindingConfig binding_config_;      ///< 组合键绑定配置

  // 上一帧状态（用于边缘检测）
  ComboKey prev_combo_;                     ///< 上一帧组合键
  bool has_prev_state_ = false;             ///< 是否已有上一帧数据
};

} // namespace runtime
} // namespace leju
