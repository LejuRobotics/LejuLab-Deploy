#pragma once

#include <mutex>
#include <vector>

#include "leju-rl-controller/runtime/input/action_trigger.h"

namespace leju {
namespace runtime {
class TriggerBuffer {
 public:
  /**
   * @brief 添加触发器
   * @param trigger 动作触发器
   *
   * 线程安全，可在任意线程调用
   */
  void push(const ActionTrigger& trigger);

  /**
   * @brief 清空所有触发器
   *
   * 在控制循环末尾调用，清除本周期已处理的触发器
   * 线程安全
   */
  void clear();

  /**
   * @brief Drain（取出并清空）所有触发器
   * @return 所有触发器列表
   *
   * - 取出所有触发器
   * - 清空缓冲区
   * - 用于 ControlLoop 每拍消费触发器
   * 线程安全
   */
  std::vector<ActionTrigger> drainAll();

 private:
  mutable std::mutex mutex_;                    ///< 保护写缓冲区的互斥锁
  std::vector<ActionTrigger> write_buffer_;     ///< 生产者写入缓冲区
  std::vector<ActionTrigger> read_buffer_;      ///< 消费者读取缓冲区（交换后无锁访问）
};

}  // namespace runtime
}  // namespace leju
