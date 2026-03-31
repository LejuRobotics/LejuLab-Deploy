#pragma once

#include "leju-rl-controller/runtime/input/command_buffer.h"

namespace leju {
namespace runtime {

/**
 * @brief 输入源优先级
 *
 * 数值越小优先级越高
 */
enum class InputPriority : int {
  kJoy = 0,       ///< Joy 手柄
  kQuest = 1,     ///< Quest VR 手柄
  kExternal = 2,  ///< 外部接口
};

/**
 * @brief 速度死区阈值常量
 *
 * 用于判断输入源是否处于"静止"状态，供 MotionCommand::isNearZero() 使用
 */
struct VelocityDeadzone {
  static constexpr double kLinearX = 0.01;    ///< x方向线速度阈值 [m/s]
  static constexpr double kLinearY = 0.01;    ///< y方向线速度阈值 [m/s]
  static constexpr double kAngularZ = 0.01;   ///< 角速度阈值 [rad/s]
};

/**
 * @brief 输入源基类
 *
 * 所有输入源的抽象基类，提供统一的接口获取命令快照。
 * 使用虚函数实现运行时多态，允许 ControlLoop 统一处理不同类型的输入源。
 */
class InputSource {
 public:
  virtual ~InputSource() = default;

  /**
   * @brief 获取当前命令快照（线程安全）
   * @return 当前命令快照
   */
  virtual CommandBuffer::Snapshot getSnapshot() const = 0;

  /**
   * @brief 获取输入源优先级
   * @return 输入源优先级，数值越小优先级越高
   */
  virtual InputPriority getPriority() const = 0;

  /**
   * @brief 获取输入源名称（用于调试/日志）
   * @return 输入源名称
   */
  virtual const char* getName() const = 0;
};

}  // namespace runtime
}  // namespace leju
