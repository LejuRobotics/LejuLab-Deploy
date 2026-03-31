/**
 * @file joy_teleop_adapter.h
 * @brief Joy 手柄输入适配器（配置驱动组合键）
 *
 * 将原始 JoyData 转换为 CommandBuffer，支持配置驱动的组合键绑定。
 * 内部直接订阅 SDK 手柄数据，对外提供统一的初始化和关闭接口。
 *
 * 配置示例（YAML）：
 * @code{.yaml}
 * joy_bindings:
 *   - combo: [LB, X]
 *     type: controller_action
 *     controller_id: dance_controller
 *     action_id: dance_a
 *   - combo: [RB, A]
 *     type: arm_mode
 *     arm_mode: auto
 * @endcode
 */

#pragma once

#include <atomic>
#include <functional>
#include <memory>

#include "leju-rl-controller/runtime/input/teleop/teleop_adapter_base.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding.hpp"
#include "lejusdk-lowlevel/data_types.h"

namespace leju {

namespace runtime {

// 前向声明
class TriggerBuffer;

/**
 * @brief Joy 手柄输入适配器
 *
 * 继承自 TeleopAdapterBase，实现 Joy 手柄特定的输入处理。
 * 采用主动订阅 SDK 模式，内部管理手柄数据订阅生命周期。
 */
class JoyTeleopAdapter : public TeleopAdapterBase<JoyTeleopAdapter, JoyData> {
public:
  /**
   * @brief 构造函数
   * @param trigger_buffer 触发器缓冲区指针（非拥有，用于输出离散事件）
   */
  explicit JoyTeleopAdapter(TriggerBuffer* trigger_buffer);
  ~JoyTeleopAdapter();

  // 禁止拷贝
  JoyTeleopAdapter(const JoyTeleopAdapter&) = delete;
  JoyTeleopAdapter& operator=(const JoyTeleopAdapter&) = delete;

  // 允许移动
  JoyTeleopAdapter(JoyTeleopAdapter&&) noexcept = default;
  JoyTeleopAdapter& operator=(JoyTeleopAdapter&&) noexcept = default;

  /**
   * @brief 初始化并订阅手柄数据
   * @return 是否成功
   */
  bool initialize();

  /**
   * @brief 关闭并停止手柄数据订阅
   */
  void shutdown();

  /**
   * @brief 检查是否已初始化
   */
  bool isInitialized() const;

private:
  friend class TeleopAdapterBase<JoyTeleopAdapter, JoyData>;

  // ========================================================================
  // InputSource 接口实现
  // ========================================================================

  InputPriority getPriority() const override {
    return InputPriority::kJoy;
  }

  const char* getName() const override {
    return "Joy";
  }

  // ========================================================================
  // CRTP 钩子方法实现
  // ========================================================================

  /**
   * @brief 获取设备绑定配置（Joy 配置）
   * @return Joy 设备绑定配置
   */
  const DeviceBindingConfig& getDeviceConfig() const {
    return binding_config_.getJoyConfig();
  }

  /**
   * @brief 识别当前组合键
   * @param joy 当前帧手柄数据
   * @return 当前按下的组合键
   */
  ComboKey detectCurrentComboImpl(const JoyData& joy) const;

  /**
   * @brief 处理摇杆 → 速度指令（写入 CommandBuffer）
   * @param joy 手柄数据
   * @param buffer 指令缓冲区
   * @param current_time 当前时间戳 [s]
   */
  void processVelocityImpl(const JoyData& joy, CommandBuffer& buffer, double current_time);

  // ========================================================================
  // 内部方法
  // ========================================================================

  /**
   * @brief 处理 Joy 数据回调
   * @param joy 当前帧手柄数据
   * @param prev 上一帧手柄按钮状态
   */
  void onJoyData(const JoyData& joy, const JoyData::Buttons& prev);

  /**
   * @brief 处理系统级按钮（START/BACK）
   * @param joy 当前帧手柄数据
   * @param prev 上一帧按钮状态
   * @param out_triggers 动作触发器列表
   */
  void processSystemButtons(const JoyData& joy,
                            const JoyData::Buttons& prev,
                            std::vector<ActionTrigger>& out_triggers);

private:
  // SDK 订阅状态
  std::atomic<bool> running_{false};        ///< 是否正在运行
  JoyData::Buttons prev_buttons_{};         ///< 上一帧按钮状态（用于边缘检测）
};

} // namespace runtime
} // namespace leju
