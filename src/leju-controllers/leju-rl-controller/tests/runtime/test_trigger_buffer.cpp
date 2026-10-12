/**
 * @file test_trigger_buffer.cpp
 * @brief TriggerBuffer 模块单元测试
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "leju-rl-controller/runtime/input/action_trigger.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"

using namespace leju::runtime;

// ============================================================================
// 基本功能测试
// ============================================================================

TEST(TriggerBufferTest, DefaultConstructor) {
  TriggerBuffer buffer;
  // 默认构造后 drain 应该返回空
  auto triggers = buffer.drainAll();
  EXPECT_TRUE(triggers.empty());
}

TEST(TriggerBufferTest, PushAndDrain) {
  TriggerBuffer buffer;

  // 添加触发器
  buffer.push(ActionTrigger(ActionType::Start));
  buffer.push(ActionTrigger(ActionType::Quit));

  // 取出所有触发器
  auto triggers = buffer.drainAll();
  EXPECT_EQ(triggers.size(), 2);
  EXPECT_EQ(triggers[0].type, ActionType::Start);
  EXPECT_EQ(triggers[1].type, ActionType::Quit);

  // 再次 drain 应该为空
  triggers = buffer.drainAll();
  EXPECT_TRUE(triggers.empty());
}

// ============================================================================
// drainAll 测试 - 双缓冲机制
// ============================================================================

TEST(TriggerBufferTest, DrainAllEmptiesBuffer) {
  TriggerBuffer buffer;

  buffer.push(ActionTrigger(ActionType::SwitchController));
  EXPECT_EQ(buffer.drainAll().size(), 1);

  // 再次 drain 应该为空
  EXPECT_TRUE(buffer.drainAll().empty());
}

TEST(TriggerBufferTest, DrainAllReturnsCorrectOrder) {
  TriggerBuffer buffer;

  // 按顺序添加多个触发器
  buffer.push(MakeSwitchControllerTrigger("amp"));
  buffer.push(MakeSwitchControllerTrigger("mimic"));
  buffer.push(MakeSetArmModeTrigger("auto"));

  auto triggers = buffer.drainAll();
  EXPECT_EQ(triggers.size(), 3);
  EXPECT_EQ(triggers[0].type, ActionType::SwitchController);
  EXPECT_EQ(triggers[1].type, ActionType::SwitchController);
  EXPECT_EQ(triggers[2].type, ActionType::SetArmMode);
}

// ============================================================================
// clear 测试
// ============================================================================

TEST(TriggerBufferTest, ClearEmptiesBuffer) {
  TriggerBuffer buffer;

  buffer.push(ActionTrigger(ActionType::Start));
  buffer.push(ActionTrigger(ActionType::Quit));

  buffer.clear();

  auto triggers = buffer.drainAll();
  EXPECT_TRUE(triggers.empty());
}

TEST(TriggerBufferTest, ClearOnEmptyBuffer) {
  TriggerBuffer buffer;
  // 清空空缓冲区不应该出错
  buffer.clear();

  auto triggers = buffer.drainAll();
  EXPECT_TRUE(triggers.empty());
}

// ============================================================================
// 线程安全测试（基本）
// ============================================================================

TEST(TriggerBufferTest, MultiplePushAndDrain) {
  TriggerBuffer buffer;

  // 多次 push-drain 循环
  for (int i = 0; i < 5; ++i) {
    buffer.push(MakeSwitchControllerTrigger("amp"));
    auto triggers = buffer.drainAll();
    EXPECT_EQ(triggers.size(), 1);
  }

  // 最后应该为空
  EXPECT_TRUE(buffer.drainAll().empty());
}

// ============================================================================
// 复杂场景测试
// ============================================================================

TEST(TriggerBufferTest, MultipleDrainsWithoutPush) {
  TriggerBuffer buffer;

  buffer.push(ActionTrigger(ActionType::Start));

  // 第一次 drain
  auto triggers1 = buffer.drainAll();
  EXPECT_EQ(triggers1.size(), 1);

  // 第二次 drain（没有新 push）
  auto triggers2 = buffer.drainAll();
  EXPECT_TRUE(triggers2.empty());

  // 再次 push 后再 drain
  buffer.push(ActionTrigger(ActionType::Quit));
  auto triggers3 = buffer.drainAll();
  EXPECT_EQ(triggers3.size(), 1);
}

TEST(TriggerBufferTest, PushAfterDrain) {
  TriggerBuffer buffer;

  // 第一轮
  buffer.push(ActionTrigger(ActionType::Start));
  auto triggers1 = buffer.drainAll();
  EXPECT_EQ(triggers1.size(), 1);

  // 第二轮
  buffer.push(ActionTrigger(ActionType::Quit));
  auto triggers2 = buffer.drainAll();
  EXPECT_EQ(triggers2.size(), 1);
  EXPECT_EQ(triggers2[0].type, ActionType::Quit);
}

TEST(TriggerBufferTest, LargeNumberOfTriggers) {
  TriggerBuffer buffer;
  const int count = 1000;

  for (int i = 0; i < count; ++i) {
    buffer.push(MakeSwitchControllerTrigger("controller_" + std::to_string(i)));
  }

  auto triggers = buffer.drainAll();
  EXPECT_EQ(triggers.size(), count);
}

TEST(TriggerBufferTest, DrainAllReturnsByMove) {
  TriggerBuffer buffer;

  buffer.push(MakeSwitchControllerTrigger("test"));
  buffer.push(MakeSetArmModeTrigger("auto"));

  // drainAll 应该返回 vector，不应该是引用
  auto triggers = buffer.drainAll();
  EXPECT_EQ(triggers.size(), 2);

  // 再次 drain 应该为空，证明是移动语义
  EXPECT_TRUE(buffer.drainAll().empty());
}

// ============================================================================
// 多线程并发测试
// ============================================================================

TEST(TriggerBufferTest, ConcurrentPush) {
  TriggerBuffer buffer;
  const int kNumThreads = 4;
  const int kPushesPerThread = 1000;

  std::vector<std::thread> threads;

  // 多个生产者线程同时 push
  for (int i = 0; i < kNumThreads; ++i) {
    threads.emplace_back([&buffer, i]() {
      for (int j = 0; j < kPushesPerThread; ++j) {
        buffer.push(MakeSwitchControllerTrigger(
            "ctrl_" + std::to_string(i) + "_" + std::to_string(j)));
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }

  // 验证所有 push 都成功
  auto triggers = buffer.drainAll();
  EXPECT_EQ(triggers.size(), kNumThreads * kPushesPerThread);
}

TEST(TriggerBufferTest, ConcurrentPushAndDrain) {
  TriggerBuffer buffer;
  const int kNumProducers = 3;
  const int kNumConsumers = 2;
  const int kPushesPerThread = 500;

  std::atomic<size_t> total_drained{0};
  std::atomic<bool> stop_consumers{false};
  std::vector<std::thread> threads;

  // 生产者线程
  for (int i = 0; i < kNumProducers; ++i) {
    threads.emplace_back([&buffer, i]() {
      for (int j = 0; j < kPushesPerThread; ++j) {
        buffer.push(MakeSwitchControllerTrigger(
            "prod_" + std::to_string(i) + "_" + std::to_string(j)));
      }
    });
  }

  // 消费者线程
  for (int i = 0; i < kNumConsumers; ++i) {
    threads.emplace_back([&buffer, &total_drained, &stop_consumers]() {
      while (!stop_consumers) {
        auto triggers = buffer.drainAll();
        total_drained += triggers.size();
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
      // 最后 drain 剩余的所有
      auto triggers = buffer.drainAll();
      total_drained += triggers.size();
    });
  }

  // 等待所有生产者完成
  for (int i = 0; i < kNumProducers; ++i) {
    threads[i].join();
  }

  // 给消费者一点时间处理剩余数据
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  stop_consumers = true;

  // 等待所有消费者完成
  for (int i = kNumProducers; i < kNumProducers + kNumConsumers; ++i) {
    threads[i].join();
  }

  // 最终 drain 可能剩余的
  auto remaining = buffer.drainAll();
  total_drained += remaining.size();

  EXPECT_EQ(total_drained, kNumProducers * kPushesPerThread);
}

TEST(TriggerBufferTest, ConcurrentPushAndClear) {
  TriggerBuffer buffer;
  const int kNumThreads = 4;
  const int kOperationsPerThread = 100;

  std::vector<std::thread> threads;

  // 一半线程 push，一半线程 clear
  for (int i = 0; i < kNumThreads / 2; ++i) {
    threads.emplace_back([&buffer]() {
      for (int j = 0; j < kOperationsPerThread; ++j) {
        buffer.push(MakeSwitchControllerTrigger("op_" + std::to_string(j)));
      }
    });
  }

  for (int i = kNumThreads / 2; i < kNumThreads; ++i) {
    threads.emplace_back([&buffer]() {
      for (int j = 0; j < kOperationsPerThread / 10; ++j) {
        buffer.clear();
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }

  // 主要验证没有崩溃和数据竞争
  // 最终结果不确定（取决于 clear 的时机），但应该能正常 drain
  buffer.drainAll();
  SUCCEED();
}

TEST(TriggerBufferTest, HighContentionScenario) {
  TriggerBuffer buffer;
  const int kNumThreads = 8;
  const int kIterations = 100;

  std::atomic<size_t> total_pushed{0};
  std::atomic<size_t> total_drained{0};
  std::vector<std::thread> threads;

  // 所有线程交替 push 和 drain
  for (int i = 0; i < kNumThreads; ++i) {
    threads.emplace_back([&buffer, &total_pushed, &total_drained, i]() {
      for (int j = 0; j < kIterations; ++j) {
        // push
        buffer.push(MakeSwitchControllerTrigger("thread_" + std::to_string(i)));
        total_pushed++;

        // 偶尔 drain
        if (j % 10 == 0) {
          auto triggers = buffer.drainAll();
          total_drained += triggers.size();
        }
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }

  // 最终 drain 剩余
  auto remaining = buffer.drainAll();
  total_drained += remaining.size();

  EXPECT_EQ(total_drained, total_pushed);
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
