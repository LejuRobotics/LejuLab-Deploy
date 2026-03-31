/**
 * @file teleop_input_source.h
 * @brief 遥操作输入源
 *
 * 将多个手柄输入源（Joy、Quest等）统一成一个输入源输出给 ControlLoop。
 * 内部处理优先级和命令合并，对外提供单一的 InputSource 接口。
 *
 * 设计特点：
 * - 内部自动创建和管理 JoyTeleopAdapter、QuestTeleopAdapter
 * - 内部按优先级获取命令快照
 * - 对外表现为单一输入源（InputSource）
 * - main 函数无需关心内部适配器细节
 */

#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include "leju-rl-controller/runtime/input/command_buffer.h"
#include "leju-rl-controller/runtime/input/input_source.h"
#include "lejusdk-utils/robot_version.hpp"

namespace leju {
namespace runtime {

// 前向声明
class JoyTeleopAdapter;
class QuestTeleopAdapter;
class TriggerBuffer;

/**
 * @brief 遥操作输入源
 *
 * 内部自动创建 Joy 和 Quest 适配器，按优先级合并命令，对外提供单一接口。
 *
 * 优先级顺序（数值越小优先级越高）：
 * 1. Joy (手柄) - 最高优先级
 * 2. Quest (VR) - 次优先级
 *
 * 使用方式：
 * @code
 *   TeleopInputSource teleop_source(robot_version, &trigger_buffer);
 *   teleop_source.initialize();
 *   teleop_source.loadBindingConfig(config_path);
 *
 *   // ControlLoop 只使用 teleop_source
 *   ControlLoop loop(..., &teleop_source, ...);
 * @endcode
 */
class TeleopInputSource : public InputSource {
 public:
  /**
   * @brief 构造函数
   * @param version 机器人版本
   * @param trigger_buffer 触发器缓冲区指针（非拥有，用于输出离散事件）
   */
  TeleopInputSource(const RobotVersion& version, TriggerBuffer* trigger_buffer);
  ~TeleopInputSource() override;

  // 禁止拷贝
  TeleopInputSource(const TeleopInputSource&) = delete;
  TeleopInputSource& operator=(const TeleopInputSource&) = delete;

  // 允许移动
  TeleopInputSource(TeleopInputSource&&) noexcept = default;
  TeleopInputSource& operator=(TeleopInputSource&&) noexcept = default;

  /**
   * @brief 初始化内部适配器
   * @return 是否成功初始化至少一个适配器
   */
  bool initialize();

  /**
   * @brief 关闭所有输入源
   */
  void shutdown();

  /**
   * @brief 检查是否已初始化
   */
  bool isInitialized() const { return initialized_.load(); }

  // ========================================================================
  // InputSource 接口实现
  // ========================================================================

  /**
   * @brief 获取当前命令快照（按优先级合并所有输入源）
   * @return 当前命令快照
   *
   * 按优先级遍历所有注册的输入源，返回第一个有效的命令。
   * 这实现了：Joy > Quest 的优先级策略。
   */
  CommandBuffer::Snapshot getSnapshot() const override;

  /**
   * @brief 获取输入源优先级
   * @return 遥操作输入源作为整体的优先级（Joy 级别）
   *
   * 注意：这是 TeleopInputSource 相对于其他输入源的优先级，
   * 内部各个手柄输入源的优先级在 getSnapshot() 中处理。
   */
  InputPriority getPriority() const override { return InputPriority::kJoy; }

  /**
   * @brief 获取输入源名称
   * @return 输入源名称
   */
  const char* getName() const override { return "TeleopInput"; }

  /**
   * @brief 加载绑定配置到所有适配器
   * @param config_path 配置文件路径
   * @return 是否至少有一个适配器成功加载
   */
  bool loadBindingConfig(const std::string& config_path);

 private:
  TriggerBuffer* trigger_buffer_;           ///< 触发器缓冲区（非拥有）
  std::atomic<bool> initialized_{false};    ///< 是否已初始化

  // 内部适配器（按优先级排序）
  std::unique_ptr<JoyTeleopAdapter> joy_adapter_;   ///< Joy 手柄适配器
  std::unique_ptr<QuestTeleopAdapter> quest_adapter_; ///< Quest VR 适配器
};

}  // namespace runtime
}  // namespace leju
