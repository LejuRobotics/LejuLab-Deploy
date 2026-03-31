#include <gtest/gtest.h>
#include <thread>
#include <vector>

#include "leju-rl-controller/robot_data.h"

using namespace leju;

class RobotDataTest : public ::testing::Test {
 protected:
  RobotData robot_data_;
};

TEST_F(RobotDataTest, InitialState) {
  // 初始状态下，数据应该都是无效的
  EXPECT_FALSE(robot_data_.hasRobotState());
  EXPECT_FALSE(robot_data_.hasImuData());
  EXPECT_FALSE(robot_data_.isDataReady());
}

TEST_F(RobotDataTest, GetRobotStateWhenInvalid) {
  RobotState state;
  // 数据无效时，getRobotState 应该返回 false
  EXPECT_FALSE(robot_data_.getRobotState(state));
}

TEST_F(RobotDataTest, GetImuDataWhenInvalid) {
  ImuData imu;
  // 数据无效时，getImuData 应该返回 false
  EXPECT_FALSE(robot_data_.getImuData(imu));
}

TEST_F(RobotDataTest, IsDataReadyRequiresBoth) {
  // isDataReady 需要 RobotState 和 ImuData 都有效
  EXPECT_FALSE(robot_data_.isDataReady());

  // 注意：由于回调是 private 的，无法直接测试数据到达后的状态
  // 需要通过 initialize() 和实际的数据订阅来测试
  // 或者添加 friend 声明允许测试访问私有成员
}

TEST_F(RobotDataTest, ConcurrentGetRobotState) {
  // 多线程同时调用 getRobotState 不应该崩溃
  std::vector<std::thread> threads;

  for (int i = 0; i < 10; ++i) {
    threads.emplace_back([this]() {
      RobotState state;
      for (int j = 0; j < 100; ++j) {
        robot_data_.getRobotState(state);
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }

  SUCCEED();
}

TEST_F(RobotDataTest, ConcurrentGetImuData) {
  // 多线程同时调用 getImuData 不应该崩溃
  std::vector<std::thread> threads;

  for (int i = 0; i < 10; ++i) {
    threads.emplace_back([this]() {
      ImuData imu;
      for (int j = 0; j < 100; ++j) {
        robot_data_.getImuData(imu);
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }

  SUCCEED();
}

TEST_F(RobotDataTest, ConcurrentMixedAccess) {
  // 多线程混合访问 getRobotState 和 getImuData
  std::vector<std::thread> threads;

  for (int i = 0; i < 5; ++i) {
    threads.emplace_back([this]() {
      RobotState state;
      for (int j = 0; j < 100; ++j) {
        robot_data_.getRobotState(state);
      }
    });

    threads.emplace_back([this]() {
      ImuData imu;
      for (int j = 0; j < 100; ++j) {
        robot_data_.getImuData(imu);
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }

  SUCCEED();
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
