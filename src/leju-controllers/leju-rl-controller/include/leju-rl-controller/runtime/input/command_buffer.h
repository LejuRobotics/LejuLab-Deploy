#pragma once

#include <array>
#include <atomic>
#include <optional>

#include "leju-rl-controller/runtime/data_types.hpp"

namespace leju {
namespace runtime {

/**
 * @brief 线程安全的命令缓冲区（单来源）
 *
 * 每个输入源（TeleopAdapter、ExternalInterface 等）有自己的实例
 * 内部实现双缓冲 + 原子索引
 */
class CommandBuffer {
public:
  struct Snapshot {
    MotionCommand cmd_vel;

    std::optional<ExternalJointTarget> arm_target;
    std::optional<ExternalJointTarget> head_target;
    std::optional<ExternalJointTarget> waist_target;
  };

  /**
   * @brief 写入速度指令
   */
  void writeCmdVel(const MotionCommand& cmd);

  /**
   * @brief 写入手臂目标
   */
  void writeArmTarget(const ExternalJointTarget& target);

  /**
   * @brief 写入头部目标
   */
  void writeHeadTarget(const ExternalJointTarget& target);

  /**
   * @brief 写入腰部目标
   */
  void writeWaistTarget(const ExternalJointTarget& target);

  /**
   * @brief 获取命令快照
   */
  Snapshot getSnapshot() const;

  /**
   * @brief 重置所有命令
   */
  void reset();

private:
  std::array<Snapshot, 2> buffers_;
  std::atomic<size_t> active_index_{0};  // 0 或 1，当前可读的 buffer
};

}  // namespace runtime
}  // namespace leju
