/**
 * @file quest_teleop_adapter.h
 * @brief Quest VR 输入适配器（配置驱动组合键）
 *
 * 将 QuestJoystickData 转换为 CommandBuffer，支持配置驱动的组合键绑定。
 */

#pragma once

#include <atomic>
#include <memory>

#include "leju-rl-controller/runtime/input/teleop/teleop_adapter_base.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding.hpp"
#include "lejusdk-utils/robot_version.hpp"
#include "lejusdk-vr/data_types.h"
#include "lejusdk-vr/vr_api/vr_base.h"

namespace leju {
namespace runtime {

// 前向声明
class TriggerBuffer;

// 类型别名（与 VR SDK 保持一致）
using QuestJoystickData = vr::QuestJoystickData;
using VrVelocityCmd = vr::VrVelocityCmd;

/**
 * @brief Quest VR 输入适配器
 *
 * 继承自 TeleopAdapterBase，实现 Quest VR 手柄特定的输入处理。
 * 采用主动订阅 SDK 模式，内部管理 Quest 数据订阅生命周期。
 *
 * 将 QuestJoystickData 转换为 CommandBuffer：
 * - 左摇杆 -> linear_x/linear_y
 * - 右摇杆 -> angular_z
 * - 组合键 -> 查询配置 -> 生成语义请求
 *
 * 使用配置驱动的组合键绑定，通过 TeleopBindingConfig 加载。
 *
 * 使用方式：
 * @code
 *   QuestTeleopAdapter adapter(robot_version, &trigger_buffer);
 *
 *   // 初始化并订阅 Quest 数据
 *   adapter.initialize();
 *
 *   // 加载配置
 *   adapter.loadBindingConfig("/path/to/bindings.yaml");
 * @endcode
 */
class QuestTeleopAdapter
    : public TeleopAdapterBase<QuestTeleopAdapter, QuestJoystickData> {
 public:
  /**
   * @brief 构造函数
   * @param version 机器人版本
   * @param trigger_buffer 触发器缓冲区指针（非拥有，用于输出离散事件）
   */
  QuestTeleopAdapter(const RobotVersion& version, TriggerBuffer* trigger_buffer = nullptr);
  ~QuestTeleopAdapter();

  // 禁止拷贝和移动（基类包含不可移动成员）
  QuestTeleopAdapter(const QuestTeleopAdapter&) = delete;
  QuestTeleopAdapter& operator=(const QuestTeleopAdapter&) = delete;

  /**
   * @brief 初始化并订阅 Quest 数据
   * @return 是否成功
   */
  bool initialize();

  /**
   * @brief 关闭并停止 Quest 数据订阅
   */
  void shutdown();

  /**
   * @brief 检查是否已初始化
   */
  bool isInitialized() const;

 private:
  friend class TeleopAdapterBase<QuestTeleopAdapter, QuestJoystickData>;

  // ========================================================================
  // InputSource 接口实现
  // ========================================================================

  InputPriority getPriority() const override {
    return InputPriority::kQuest;
  }

  const char* getName() const override {
    return "Quest";
  }

  // ========================================================================
  // CRTP 钩子方法实现
  // ========================================================================

  /**
   * @brief 获取设备绑定配置（Quest 配置）
   * @return Quest 设备绑定配置
   */
  const DeviceBindingConfig& getDeviceConfig() const {
    return binding_config_.getQuestConfig();
  }

  /**
   * @brief 识别当前组合键
   * @param data Quest 手柄数据
   * @return 当前按下的组合键
   */
  ComboKey detectCurrentComboImpl(const QuestJoystickData& data) const;

  /**
   * @brief 处理摇杆 -> 速度指令
   * @param data Quest 手柄数据
   * @param[out] buffer 指令缓冲区
   * @param current_time 当前时间戳 [s]
   */
  void processVelocityImpl(const QuestJoystickData& data,
                           CommandBuffer& buffer,
                           double current_time);

  /**
   * @brief 边缘检测策略：空->非空转换触发
   *
   * Quest 特有的边缘检测策略，只在组合键从空变为非空时触发
   *（即按下时触发，而不是释放时）
   *
   * @param prev 上一帧组合键
   * @param current 当前帧组合键
   * @return 是否应触发绑定
   */
  bool shouldTriggerCombo(const ComboKey& prev, const ComboKey& current) const;

  // ========================================================================
  // 内部方法
  // ========================================================================

  /**
   * @brief 处理 Quest 数据回调（内部使用）
   * @param data Quest 手柄数据
   */
  void onQuestData(const QuestJoystickData& data);

  /**
   * @brief 处理左控制器输入（速度指令）
   * @param data Quest 手柄数据
   * @param[out] buffer 指令缓冲区
   * @param current_time 当前时间戳 [s]
   */
  void processLeftController(const QuestJoystickData& data,
                             CommandBuffer& buffer,
                             double current_time);

  /**
   * @brief 处理右控制器输入（速度指令）
   * @param data Quest 手柄数据
   * @param[out] buffer 指令缓冲区
   * @param current_time 当前时间戳 [s]
   */
  void processRightController(const QuestJoystickData& data,
                              CommandBuffer& buffer,
                              double current_time);

 private:
  // VR API
  RobotVersion robot_version_;                    ///< 机器人版本
  std::unique_ptr<vr::VRBaseAPI> vr_api_;         ///< VR API 实例
  std::atomic<bool> running_{false};              ///< 是否正在运行

  // 状态管理
  ComboKey prev_combo_;                           ///< 上一帧组合键
  bool has_prev_state_ = false;                   ///< 是否有上一帧状态
};

}  // namespace runtime
}  // namespace leju
