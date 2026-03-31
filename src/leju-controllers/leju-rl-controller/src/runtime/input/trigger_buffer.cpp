#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/runtime/data_types.hpp"

namespace leju {
namespace runtime {

void TriggerBuffer::push(const ActionTrigger& trigger) {
  std::lock_guard<std::mutex> lock(mutex_);
  write_buffer_.push_back(trigger);
}

void TriggerBuffer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  write_buffer_.clear();
  read_buffer_.clear();
}

std::vector<ActionTrigger> TriggerBuffer::drainAll() {
  std::lock_guard<std::mutex> lock(mutex_);
  // 双缓冲交换：消费者每个 tick 只锁一次
  write_buffer_.swap(read_buffer_);
  write_buffer_.clear();
  return std::move(read_buffer_);
}

}  // namespace runtime
}  // namespace leju
