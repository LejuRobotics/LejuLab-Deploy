#include "leju-rl-controller/runtime/input/command_buffer.h"

namespace leju {
namespace runtime {

// ============================================================================
// 写入操作
// ============================================================================

void CommandBuffer::writeCmdVel(const MotionCommand& cmd) {
  size_t current = active_index_.load(std::memory_order_relaxed);
  size_t write_index = 1 - current;

  buffers_[write_index] = buffers_[current];
  buffers_[write_index].cmd_vel = cmd;
  active_index_.store(write_index, std::memory_order_release);
}

void CommandBuffer::writeArmTarget(const ExternalJointTarget& target) {
  size_t current = active_index_.load(std::memory_order_relaxed);
  size_t write_index = 1 - current;

  buffers_[write_index] = buffers_[current];
  buffers_[write_index].arm_target = target;
  active_index_.store(write_index, std::memory_order_release);
}

void CommandBuffer::writeHeadTarget(const ExternalJointTarget& target) {
  size_t current = active_index_.load(std::memory_order_relaxed);
  size_t write_index = 1 - current;

  buffers_[write_index] = buffers_[current];
  buffers_[write_index].head_target = target;
  active_index_.store(write_index, std::memory_order_release);
}

void CommandBuffer::writeWaistTarget(const ExternalJointTarget& target) {
  size_t current = active_index_.load(std::memory_order_relaxed);
  size_t write_index = 1 - current;

  buffers_[write_index] = buffers_[current];
  buffers_[write_index].waist_target = target;
  active_index_.store(write_index, std::memory_order_release);
}

// ============================================================================
// 读取操作
// ============================================================================

CommandBuffer::Snapshot CommandBuffer::getSnapshot() const {
  size_t index = active_index_.load(std::memory_order_acquire);
  return buffers_[index];
}

// ============================================================================
// 重置操作
// ============================================================================

void CommandBuffer::reset() {
  size_t current = active_index_.load(std::memory_order_relaxed);
  size_t write_index = 1 - current;

  buffers_[write_index] = Snapshot();  // 默认构造
  active_index_.store(write_index, std::memory_order_release);
}

}  // namespace runtime
}  // namespace leju
