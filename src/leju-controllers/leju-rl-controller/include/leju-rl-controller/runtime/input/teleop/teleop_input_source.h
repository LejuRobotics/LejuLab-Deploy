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

#include <functional>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
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

  // 禁止拷贝/移动 (持有 std::mutex, 不可移动; 全程原地构造, 不需要)
  TeleopInputSource(const TeleopInputSource&) = delete;
  TeleopInputSource& operator=(const TeleopInputSource&) = delete;
  TeleopInputSource(TeleopInputSource&&) = delete;
  TeleopInputSource& operator=(TeleopInputSource&&) = delete;

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

  /**
   * @brief 热重载: 重新读取上次成功加载的配置文件并应用到各适配器
   *
   * 供遥控器配置热重载信号 (kReloadTeleopConfig) 的回调调用. 线程安全:
   * 适配器内部 setBindingConfig 持锁, 路径成员由 path_mutex_ 保护.
   *
   * @return 成功返回 true; 若从未成功加载过 (无已知路径) 返回 false
   */
  bool reloadBindingConfig();

  /**
   * @brief 从 AMP 控制器配置合并 amp_hand 手柄参数到各适配器
   */
  bool mergeAmpHandTeleopFromControllerConfig(const std::string& config_path);

  /**
   * @brief 设置蹲起状态查询回调
   *
   * 当 External 处于下蹲/姿态模式（cmd_stance=1）时，遥操作行走命令不应接管，
   * 否则会在 mergeAllCmdVel 里用 mode=0 的行走命令覆盖 External 的下蹲高度指令。
   * 该回调由 main 注入（查询 controller_manager 的当前 cmd_stance_mode）。
   *
   * @param callback 返回 true 表示当前处于蹲起状态
   */
  void setStanceActiveQuery(std::function<bool()> callback) {
    stance_active_query_ = std::move(callback);
  }

 private:
  TriggerBuffer* trigger_buffer_;           ///< 触发器缓冲区（非拥有）
  std::atomic<bool> initialized_{false};    ///< 是否已初始化

  mutable std::mutex path_mutex_;           ///< 保护 binding_config_path_
  std::string binding_config_path_;         ///< 上次成功加载的配置路径（用于热重载）

  // 内部适配器（按优先级排序）
  std::unique_ptr<JoyTeleopAdapter> joy_adapter_;   ///< Joy 手柄适配器
  std::unique_ptr<QuestTeleopAdapter> quest_adapter_; ///< Quest VR 适配器

  // 蹲起状态查询回调：蹲起活跃时 teleop 行走命令让路给 External 的 stance 指令
  std::function<bool()> stance_active_query_;
};

}  // namespace runtime
}  // namespace leju
