#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "leju-rl-controller/trajectory/interpolator/interpolator_base.h"
#include "leju-rl-controller/trajectory/interpolator/minimum_jerk_interpolator.h"

using namespace leju;

class MinimumJerkInterpolatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 设置测试用的起点和终点
    start_pos_.resize(3);
    start_pos_ << 0.0, 1.0, 2.0;

    end_pos_.resize(3);
    end_pos_ << 1.0, 2.0, 3.0;

    duration_ = 2.0;
  }

  Eigen::VectorXd start_pos_;
  Eigen::VectorXd end_pos_;
  double duration_;
  MinimumJerkInterpolator interpolator_;
};

// ============================================================================
// 初始状态测试
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, InitialState) {
  EXPECT_FALSE(interpolator_.isInitialized());
  EXPECT_EQ(interpolator_.getDimension(), 0);
  EXPECT_DOUBLE_EQ(interpolator_.getDuration(), 0.0);
}

// ============================================================================
// Setup 测试
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, SetupValid) {
  EXPECT_TRUE(interpolator_.setup(start_pos_, end_pos_, duration_));
  EXPECT_TRUE(interpolator_.isInitialized());
  EXPECT_EQ(interpolator_.getDimension(), 3);
  EXPECT_DOUBLE_EQ(interpolator_.getDuration(), duration_);
}

TEST_F(MinimumJerkInterpolatorTest, SetupDimensionMismatch) {
  Eigen::VectorXd wrong_size(2);
  wrong_size << 0.0, 1.0;

  EXPECT_FALSE(interpolator_.setup(start_pos_, wrong_size, duration_));
  EXPECT_FALSE(interpolator_.isInitialized());
}

TEST_F(MinimumJerkInterpolatorTest, SetupZeroDuration) {
  EXPECT_FALSE(interpolator_.setup(start_pos_, end_pos_, 0.0));
  EXPECT_FALSE(interpolator_.isInitialized());
}

TEST_F(MinimumJerkInterpolatorTest, SetupNegativeDuration) {
  EXPECT_FALSE(interpolator_.setup(start_pos_, end_pos_, -1.0));
  EXPECT_FALSE(interpolator_.isInitialized());
}

// ============================================================================
// 边界条件测试
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, EvaluateAtStart) {
  interpolator_.setup(start_pos_, end_pos_, duration_);

  Eigen::VectorXd pos, vel;
  EXPECT_TRUE(interpolator_.evaluate(0.0, pos, vel));

  // t=0 时位置应为起点
  EXPECT_NEAR(pos[0], start_pos_[0], 1e-9);
  EXPECT_NEAR(pos[1], start_pos_[1], 1e-9);
  EXPECT_NEAR(pos[2], start_pos_[2], 1e-9);

  // t=0 时速度应为零
  EXPECT_NEAR(vel[0], 0.0, 1e-9);
  EXPECT_NEAR(vel[1], 0.0, 1e-9);
  EXPECT_NEAR(vel[2], 0.0, 1e-9);
}

TEST_F(MinimumJerkInterpolatorTest, EvaluateAtEnd) {
  interpolator_.setup(start_pos_, end_pos_, duration_);

  Eigen::VectorXd pos, vel;
  EXPECT_TRUE(interpolator_.evaluate(duration_, pos, vel));

  // t=duration 时位置应为终点
  EXPECT_NEAR(pos[0], end_pos_[0], 1e-9);
  EXPECT_NEAR(pos[1], end_pos_[1], 1e-9);
  EXPECT_NEAR(pos[2], end_pos_[2], 1e-9);

  // t=duration 时速度应为零
  EXPECT_NEAR(vel[0], 0.0, 1e-9);
  EXPECT_NEAR(vel[1], 0.0, 1e-9);
  EXPECT_NEAR(vel[2], 0.0, 1e-9);
}

TEST_F(MinimumJerkInterpolatorTest, EvaluateAtMiddle) {
  interpolator_.setup(start_pos_, end_pos_, duration_);

  Eigen::VectorXd pos, vel;
  EXPECT_TRUE(interpolator_.evaluate(duration_ / 2.0, pos, vel));

  // t=duration/2 时，tau=0.5, s(0.5) = 10*0.125 - 15*0.0625 + 6*0.03125 = 0.5
  // 位置应在起终点中间
  Eigen::VectorXd expected_pos = start_pos_ + 0.5 * (end_pos_ - start_pos_);
  EXPECT_NEAR(pos[0], expected_pos[0], 1e-9);
  EXPECT_NEAR(pos[1], expected_pos[1], 1e-9);
  EXPECT_NEAR(pos[2], expected_pos[2], 1e-9);

  // 中间点速度应大于零
  EXPECT_GT(vel.norm(), 0.0);
}

TEST_F(MinimumJerkInterpolatorTest, EvaluateNegativeTime) {
  interpolator_.setup(start_pos_, end_pos_, duration_);

  Eigen::VectorXd pos, vel;
  EXPECT_TRUE(interpolator_.evaluate(-1.0, pos, vel));

  // 负时间应 clamp 到起点
  EXPECT_NEAR(pos[0], start_pos_[0], 1e-9);
  EXPECT_NEAR(pos[1], start_pos_[1], 1e-9);
  EXPECT_NEAR(pos[2], start_pos_[2], 1e-9);
}

TEST_F(MinimumJerkInterpolatorTest, EvaluateBeyondDuration) {
  interpolator_.setup(start_pos_, end_pos_, duration_);

  Eigen::VectorXd pos, vel;
  EXPECT_TRUE(interpolator_.evaluate(duration_ + 1.0, pos, vel));

  // 超出时间应 clamp 到终点
  EXPECT_NEAR(pos[0], end_pos_[0], 1e-9);
  EXPECT_NEAR(pos[1], end_pos_[1], 1e-9);
  EXPECT_NEAR(pos[2], end_pos_[2], 1e-9);
}

// ============================================================================
// 未初始化测试
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, EvaluateBeforeSetup) {
  Eigen::VectorXd pos, vel;
  EXPECT_FALSE(interpolator_.evaluate(0.0, pos, vel));
}

// ============================================================================
// isFinished 测试
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, IsFinishedBeforeDuration) {
  interpolator_.setup(start_pos_, end_pos_, duration_);

  EXPECT_FALSE(interpolator_.isFinished(0.0));
  EXPECT_FALSE(interpolator_.isFinished(duration_ / 2.0));
  EXPECT_FALSE(interpolator_.isFinished(duration_ - 0.001));
}

TEST_F(MinimumJerkInterpolatorTest, IsFinishedAtDuration) {
  interpolator_.setup(start_pos_, end_pos_, duration_);

  EXPECT_TRUE(interpolator_.isFinished(duration_));
  EXPECT_TRUE(interpolator_.isFinished(duration_ + 1.0));
}

// ============================================================================
// Reset 测试
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, Reset) {
  interpolator_.setup(start_pos_, end_pos_, duration_);
  EXPECT_TRUE(interpolator_.isInitialized());

  interpolator_.reset();
  EXPECT_FALSE(interpolator_.isInitialized());
  EXPECT_EQ(interpolator_.getDimension(), 0);
  EXPECT_DOUBLE_EQ(interpolator_.getDuration(), 0.0);
}

// ============================================================================
// 单维度测试
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, SingleDimension) {
  Eigen::VectorXd start(1), end(1);
  start << 0.0;
  end << 10.0;

  interpolator_.setup(start, end, 1.0);

  Eigen::VectorXd pos, vel;
  interpolator_.evaluate(0.5, pos, vel);

  // t=0.5, tau=0.5, s(0.5)=0.5, pos = 0 + 0.5 * 10 = 5
  EXPECT_NEAR(pos[0], 5.0, 1e-9);
}

// ============================================================================
// 高维度多关节测试（模拟机械臂）
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, HighDimensionalArmTrajectory) {
  // 模拟 6-DOF 机械臂
  const int dof = 6;
  Eigen::VectorXd start(dof), end(dof);
  start << 0.0, -0.5, 1.57, 0.0, -0.78, 0.0;      // 典型初始位姿
  end << 0.5, 0.3, 0.78, -0.5, 0.0, 1.57;         // 目标位姿

  double duration = 2.0;
  EXPECT_TRUE(interpolator_.setup(start, end, duration));
  EXPECT_EQ(interpolator_.getDimension(), dof);

  // 验证各时刻插值
  Eigen::VectorXd pos, vel;
  interpolator_.evaluate(0.0, pos, vel);
  EXPECT_EQ(pos.size(), dof);
  EXPECT_EQ(vel.size(), dof);

  // 验证所有关节同时到达
  interpolator_.evaluate(duration, pos, vel);
  for (int i = 0; i < dof; ++i) {
    EXPECT_NEAR(pos[i], end[i], 1e-9);
    EXPECT_NEAR(vel[i], 0.0, 1e-9);
  }
}

// ============================================================================
// 数学性质验证：峰值速度在中点
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, PeakVelocityAtMidpoint) {
  Eigen::VectorXd start(1), end(1);
  start << 0.0;
  end << 1.0;
  double duration = 1.0;

  interpolator_.setup(start, end, duration);

  // ds/dtau = 30*tau^2 - 60*tau^3 + 30*tau^4
  // 在 tau=0.5 时取得最大值: 30*0.25 - 60*0.125 + 30*0.0625 = 1.875
  // 峰值速度 = 1.875 * (end - start) / T = 1.875

  Eigen::VectorXd pos, vel;
  double max_vel = 0.0;
  double max_vel_time = 0.0;

  const int samples = 100;
  for (int i = 0; i <= samples; ++i) {
    double t = duration * i / samples;
    interpolator_.evaluate(t, pos, vel);
    if (std::abs(vel[0]) > max_vel) {
      max_vel = std::abs(vel[0]);
      max_vel_time = t;
    }
  }

  // 峰值应该在中点附近
  EXPECT_NEAR(max_vel_time, duration / 2.0, duration / samples);
  // 峰值速度应为 1.875
  EXPECT_NEAR(max_vel, 1.875, 1e-3);
}

// ============================================================================
// 时长缩放测试：速度随时长反比缩放
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, DurationScalingVelocity) {
  Eigen::VectorXd start(1), end(1);
  start << 0.0;
  end << 1.0;

  // 两个不同时长
  double duration1 = 1.0;
  double duration2 = 2.0;

  Eigen::VectorXd pos1, vel1, pos2, vel2;

  // 测试 duration1
  interpolator_.setup(start, end, duration1);
  interpolator_.evaluate(duration1 / 2.0, pos1, vel1);

  // 测试 duration2
  interpolator_.setup(start, end, duration2);
  interpolator_.evaluate(duration2 / 2.0, pos2, vel2);

  // 位置应相同（都在中点）
  EXPECT_NEAR(pos1[0], pos2[0], 1e-9);
  EXPECT_NEAR(pos1[0], 0.5, 1e-9);

  // 速度应反比于时长: vel1 / vel2 = duration2 / duration1 = 2
  EXPECT_NEAR(vel1[0] / vel2[0], 2.0, 1e-9);
}

// ============================================================================
// 实时步进模拟（变步长）
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, RealTimeSteppingSimulation) {
  Eigen::VectorXd start(3), end(3);
  start << 0.0, 0.0, 0.0;
  end << 1.0, 2.0, 3.0;
  double duration = 1.0;

  interpolator_.setup(start, end, duration);

  // 模拟变步长控制循环 (1ms ~ 10ms)
  double t = 0.0;
  Eigen::VectorXd prev_pos = start;
  Eigen::VectorXd pos, vel;

  std::vector<double> dt_list = {0.001, 0.005, 0.01, 0.002, 0.008, 0.003};

  for (double dt : dt_list) {
    t += dt;
    if (t > duration) break;

    interpolator_.evaluate(t, pos, vel);

    // 位置应单调递增（对于递增轨迹）
    for (int i = 0; i < 3; ++i) {
      EXPECT_GE(pos[i], prev_pos[i] - 1e-9);
    }
    prev_pos = pos;
  }

  // 最终应能到达终点
  interpolator_.evaluate(duration, pos, vel);
  EXPECT_NEAR(pos[0], end[0], 1e-9);
  EXPECT_NEAR(pos[1], end[1], 1e-9);
  EXPECT_NEAR(pos[2], end[2], 1e-9);
}

// ============================================================================
// 极短时长边界测试
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, VeryShortDuration) {
  Eigen::VectorXd start(1), end(1);
  start << 0.0;
  end << 1.0;
  double duration = 0.001;  // 1ms

  EXPECT_TRUE(interpolator_.setup(start, end, duration));

  Eigen::VectorXd pos, vel;

  // 中点速度会非常大
  interpolator_.evaluate(duration / 2.0, pos, vel);
  EXPECT_NEAR(pos[0], 0.5, 1e-9);
  EXPECT_NEAR(vel[0], 1875.0, 1e-3);  // 1.875 / 0.001 = 1875
}

// ============================================================================
// 起终点相同测试
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, SameStartAndEnd) {
  Eigen::VectorXd start(3), end(3);
  start << 1.0, 2.0, 3.0;
  end << 1.0, 2.0, 3.0;  // 相同位置
  double duration = 1.0;

  EXPECT_TRUE(interpolator_.setup(start, end, duration));

  Eigen::VectorXd pos, vel;

  // 任意时刻位置都应该是起点/终点
  interpolator_.evaluate(0.0, pos, vel);
  EXPECT_NEAR(pos[0], 1.0, 1e-9);
  EXPECT_NEAR(vel[0], 0.0, 1e-9);

  interpolator_.evaluate(0.5, pos, vel);
  EXPECT_NEAR(pos[0], 1.0, 1e-9);
  EXPECT_NEAR(vel[0], 0.0, 1e-9);

  interpolator_.evaluate(1.0, pos, vel);
  EXPECT_NEAR(pos[0], 1.0, 1e-9);
  EXPECT_NEAR(vel[0], 0.0, 1e-9);
}

// ============================================================================
// 大位移关节运动测试
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, LargeDisplacement) {
  // 模拟大角度关节运动 (e.g., -π to +π)
  Eigen::VectorXd start(1), end(1);
  start << -M_PI;
  end << M_PI;
  double duration = 3.0;

  EXPECT_TRUE(interpolator_.setup(start, end, duration));

  Eigen::VectorXd pos, vel;

  // 验证全程平滑
  double prev_pos = start[0];
  for (int i = 1; i <= 100; ++i) {
    double t = duration * i / 100.0;
    interpolator_.evaluate(t, pos, vel);

    // 位置应单调递增
    EXPECT_GT(pos[0], prev_pos - 1e-9);
    prev_pos = pos[0];
  }

  // 终点验证
  interpolator_.evaluate(duration, pos, vel);
  EXPECT_NEAR(pos[0], M_PI, 1e-9);
}

// ============================================================================
// 连续重新设置测试
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, ConsecutiveSetup) {
  Eigen::VectorXd start1(2), end1(2);
  start1 << 0.0, 0.0;
  end1 << 1.0, 1.0;

  Eigen::VectorXd start2(3), end2(3);
  start2 << 0.0, 0.0, 0.0;
  end2 << 2.0, 2.0, 2.0;

  // 第一次设置
  EXPECT_TRUE(interpolator_.setup(start1, end1, 1.0));
  EXPECT_EQ(interpolator_.getDimension(), 2);

  // 第二次设置（不同维度）
  EXPECT_TRUE(interpolator_.setup(start2, end2, 2.0));
  EXPECT_EQ(interpolator_.getDimension(), 3);
  EXPECT_DOUBLE_EQ(interpolator_.getDuration(), 2.0);

  // 验证使用新参数
  Eigen::VectorXd pos, vel;
  interpolator_.evaluate(2.0, pos, vel);
  EXPECT_NEAR(pos[0], 2.0, 1e-9);
  EXPECT_NEAR(pos[1], 2.0, 1e-9);
  EXPECT_NEAR(pos[2], 2.0, 1e-9);
}

// ============================================================================
// 基类指针多态测试
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, PolymorphismThroughBasePointer) {
  // 通过基类指针使用插值器
  std::unique_ptr<InterpolatorBase> base_ptr =
      std::make_unique<MinimumJerkInterpolator>();

  // 验证类型名称
  EXPECT_STREQ(base_ptr->getName(), "MinimumJerk");

  // 验证初始状态
  EXPECT_FALSE(base_ptr->isInitialized());
  EXPECT_EQ(base_ptr->getDimension(), 0);
  EXPECT_DOUBLE_EQ(base_ptr->getDuration(), 0.0);

  // 通过基类指针设置
  EXPECT_TRUE(base_ptr->setup(start_pos_, end_pos_, duration_));
  EXPECT_TRUE(base_ptr->isInitialized());
  EXPECT_EQ(base_ptr->getDimension(), 3);
  EXPECT_DOUBLE_EQ(base_ptr->getDuration(), duration_);

  // 通过基类指针计算插值
  Eigen::VectorXd pos, vel;
  EXPECT_TRUE(base_ptr->evaluate(0.0, pos, vel));
  EXPECT_NEAR(pos[0], start_pos_[0], 1e-9);
  EXPECT_NEAR(pos[1], start_pos_[1], 1e-9);
  EXPECT_NEAR(pos[2], start_pos_[2], 1e-9);

  EXPECT_TRUE(base_ptr->evaluate(duration_, pos, vel));
  EXPECT_NEAR(pos[0], end_pos_[0], 1e-9);
  EXPECT_NEAR(pos[1], end_pos_[1], 1e-9);
  EXPECT_NEAR(pos[2], end_pos_[2], 1e-9);

  // 通过基类指针检查完成状态
  EXPECT_FALSE(base_ptr->isFinished(duration_ / 2.0));
  EXPECT_TRUE(base_ptr->isFinished(duration_));

  // 通过基类指针重置
  base_ptr->reset();
  EXPECT_FALSE(base_ptr->isInitialized());
}

TEST_F(MinimumJerkInterpolatorTest, GetNameReturnsCorrectType) {
  EXPECT_STREQ(interpolator_.getName(), "MinimumJerk");
}

// ============================================================================
// 可视化数据导出测试
// ============================================================================

TEST_F(MinimumJerkInterpolatorTest, ExportDataForVisualization) {
  // 7-DOF 机械臂轨迹，模拟典型机械臂运动
  const int num_joints = 7;
  Eigen::VectorXd start(num_joints), end(num_joints);
  // 典型 7-DOF 机械臂关节角度范围 (rad)
  start << 0.0, -0.5, 0.3, -1.2, 0.0, 0.8, 0.0;
  end << 0.8, 0.3, -0.5, -0.3, 1.0, -0.2, 0.5;
  double duration = 2.0;

  interpolator_.setup(start, end, duration);

  // 输出 CSV 文件
  std::ofstream file("/tmp/interpolator_data.csv");

  // 写入表头
  file << "time";
  for (int j = 0; j < num_joints; ++j) {
    file << ",j" << j << "_linear,j" << j << "_mjerk"
         << ",j" << j << "_vel_linear,j" << j << "_vel_mjerk";
  }
  file << "\n";

  const int samples = 200;
  for (int i = 0; i <= samples; ++i) {
    double t = duration * i / samples;
    double tau = t / duration;

    // 最小急动度插值
    Eigen::VectorXd pos, vel;
    interpolator_.evaluate(t, pos, vel);

    file << t;
    for (int j = 0; j < num_joints; ++j) {
      // 线性插值
      double linear_pos = start[j] + tau * (end[j] - start[j]);
      double linear_vel = (end[j] - start[j]) / duration;

      file << "," << linear_pos << "," << pos[j]
           << "," << linear_vel << "," << vel[j];
    }
    file << "\n";
  }
  file.close();

  std::cout << "Data exported to /tmp/interpolator_data.csv\n";
  std::cout << "7-DOF arm trajectory:\n";
  for (int j = 0; j < num_joints; ++j) {
    std::cout << "  Joint " << j << ": " << start[j] << " -> " << end[j] << "\n";
  }
  std::cout << "Duration: " << duration << "s\n";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
