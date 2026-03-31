#pragma once

#include <string>
#include <vector>

#include "leju-rl-controller/runtime/input/action_trigger.h"

namespace leju {
namespace runtime {

/**
 * @brief 生命周期状态枚举
 */
enum class LifecycleState {
  kWaitingForReady,  ///< 等待传感器就绪
  kWaitingForStart,  ///< 就绪，等待启动信号
  kRunning,          ///< 运行中
  kExiting,          ///< 退出
};

/**
 * @brief 生命周期管理器
 *
 * 独立于控制器的系统级生命周期管理，负责：
 * 1. 监控传感器/硬件就绪状态
 * 2. 管理 start/quit 触发器
 * 3. 在适当时机触发控制器启动/停止
 *
 * 状态转换规则：
 * @code
 *         +-------------+               +-------------+               +----------+
 *         |  kWaiting   |  ready=true   |  kWaiting   |    start      | kRunning |
 *         |   ForReady  |-------------> |  ForStart   |-------------> |          |
 *         +------+------+               +------+------+               +----+-----+
 *                |                             |                            |
 *           quit |                        quit |                       quit |
 *                v                             v                            v
 *         +-------------------------------------------------------------------+
 *         |                            kExiting                               |
 *         +-------------------------------------------------------------------+
 *
 * @endcode
 *
 */
class Lifecycle {
 public:
  /**
   * @brief 默认构造函数
   */
  Lifecycle() = default;

  /**
   * @brief 更新生命周期状态（由控制线程每帧调用）
   * @param ready 机器人数据是否就绪
   * @param triggers 动作触发器列表
   */
  void update(bool ready, const std::vector<ActionTrigger>& triggers);

  /**
   * @brief 是否处于运行状态
   * @return true 表示系统正在运行，允许执行控制器逻辑
   */
  bool isRunning() const {
    return state_ == LifecycleState::kRunning;
  }

  /**
   * @brief 是否需要退出程序
   * @return true 表示收到 quit 信号，需要退出
   */
  bool shouldExit() const {
    return state_ == LifecycleState::kExiting;
  }

  /**
   * @brief 获取当前状态
   * @return 当前生命周期状态
   */
  LifecycleState state() const {
    return state_;
  }

 private:
  LifecycleState state_ = LifecycleState::kWaitingForReady;
};

}  // namespace runtime
}  // namespace leju
