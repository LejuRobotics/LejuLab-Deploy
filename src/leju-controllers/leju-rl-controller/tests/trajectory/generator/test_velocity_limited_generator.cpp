#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "leju-rl-controller/trajectory/generator/trajectory_generator_base.h"
#include "leju-rl-controller/trajectory/generator/velocity_limited_generator.h"

using namespace leju;

class VelocityLimitedGeneratorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 设置测试用的目标和最大速度
    target_pos_.resize(3);
    target_pos_ << 1.0, 2.0, 3.0;

    current_pos_.resize(3);
    current_pos_ << 0.0, 0.0, 0.0;

    current_vel_.resize(3);
    current_vel_ << 0.0, 0.0, 0.0;

    max_velocity_.resize(3);
    max_velocity_ << 0.5, 0.5, 0.5;

    dt_ = 0.01;  // 10ms
  }

  Eigen::VectorXd target_pos_;
  Eigen::VectorXd current_pos_;
  Eigen::VectorXd current_vel_;
  Eigen::VectorXd max_velocity_;
  double dt_;
  VelocityLimitedGenerator generator_;
};

// ============================================================================
// 初始状态测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, InitialState) {
  EXPECT_EQ(generator_.getDimension(), 0);
  EXPECT_STREQ(generator_.getName(), "VelocityLimited");
  EXPECT_EQ(generator_.getLimitMode(), VelocityLimitMode::kPerJoint);
}

TEST_F(VelocityLimitedGeneratorTest, ConstructWithSynchronizedMode) {
  VelocityLimitedGenerator sync_gen(VelocityLimitMode::kSynchronized);
  EXPECT_EQ(sync_gen.getLimitMode(), VelocityLimitMode::kSynchronized);
}

// ============================================================================
// setMaxVelocity 测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, SetMaxVelocity) {
  generator_.setMaxVelocity(max_velocity_);
  generator_.setTarget(target_pos_);
  generator_.update(current_pos_, current_vel_, dt_);

  // 验证期望速度不超过最大速度
  const Eigen::VectorXd& vel = generator_.getDesiredVel();
  for (int i = 0; i < 3; ++i) {
    EXPECT_LE(std::abs(vel[i]), max_velocity_[i] + 1e-9);
  }
}

// ============================================================================
// 静止目标到达测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, ReachStaticTarget) {
  generator_.setMaxVelocity(max_velocity_);
  generator_.setTarget(target_pos_);

  Eigen::VectorXd pos = current_pos_;
  Eigen::VectorXd vel = current_vel_;

  // 初始化 error_
  generator_.update(pos, vel, dt_);
  pos = generator_.getDesiredPos();
  vel = generator_.getDesiredVel();

  // 模拟运动直到到达目标
  const int max_steps = 10000;
  int steps = 0;
  while (!generator_.isReached(1e-6) && steps < max_steps) {
    generator_.update(pos, vel, dt_);
    pos = generator_.getDesiredPos();
    vel = generator_.getDesiredVel();
    ++steps;
  }

  // 应该能够到达目标
  EXPECT_TRUE(generator_.isReached(1e-6));
  EXPECT_NEAR(pos[0], target_pos_[0], 1e-6);
  EXPECT_NEAR(pos[1], target_pos_[1], 1e-6);
  EXPECT_NEAR(pos[2], target_pos_[2], 1e-6);
}

// ============================================================================
// 速度限制有效性测试 (kPerJoint)
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, VelocityLimitPerJoint) {
  generator_.setLimitMode(VelocityLimitMode::kPerJoint);
  generator_.setMaxVelocity(max_velocity_);
  generator_.setTarget(target_pos_);
  generator_.update(current_pos_, current_vel_, dt_);

  const Eigen::VectorXd& vel = generator_.getDesiredVel();

  // 每个关节速度应不超过对应的最大速度
  for (int i = 0; i < 3; ++i) {
    EXPECT_LE(std::abs(vel[i]), max_velocity_[i] + 1e-9);
  }

  // 位置增量应等于 max_velocity * dt
  const Eigen::VectorXd& pos = generator_.getDesiredPos();
  for (int i = 0; i < 3; ++i) {
    double delta = pos[i] - current_pos_[i];
    EXPECT_NEAR(delta, max_velocity_[i] * dt_, 1e-9);
  }
}

// ============================================================================
// 速度限制有效性测试 (kSynchronized)
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, VelocityLimitSynchronized) {
  generator_.setLimitMode(VelocityLimitMode::kSynchronized);
  generator_.setMaxVelocity(max_velocity_);
  generator_.setTarget(target_pos_);
  generator_.update(current_pos_, current_vel_, dt_);

  const Eigen::VectorXd& vel = generator_.getDesiredVel();

  // 速度向量范数应不超过 max_velocity 范数
  double max_vel_norm = max_velocity_.norm();
  EXPECT_LE(vel.norm(), max_vel_norm + 1e-9);

  // 运动方向应与误差方向一致
  Eigen::VectorXd error = target_pos_ - current_pos_;
  double dot = vel.dot(error);
  EXPECT_GT(dot, 0);  // 同向
}

TEST_F(VelocityLimitedGeneratorTest, SynchronizedModeProportionalMovement) {
  generator_.setLimitMode(VelocityLimitMode::kSynchronized);
  generator_.setMaxVelocity(max_velocity_);

  // 设置不同幅度的目标
  Eigen::VectorXd target(3);
  target << 1.0, 2.0, 4.0;  // 第三个关节误差最大
  generator_.setTarget(target);
  generator_.update(current_pos_, current_vel_, dt_);

  const Eigen::VectorXd& pos = generator_.getDesiredPos();
  Eigen::VectorXd delta = pos - current_pos_;

  // 各关节增量应保持与误差相同的比例
  Eigen::VectorXd error = target - current_pos_;
  double ratio_01 = delta[0] / delta[1];
  double expected_ratio_01 = error[0] / error[1];
  EXPECT_NEAR(ratio_01, expected_ratio_01, 1e-9);

  double ratio_02 = delta[0] / delta[2];
  double expected_ratio_02 = error[0] / error[2];
  EXPECT_NEAR(ratio_02, expected_ratio_02, 1e-9);
}

// ============================================================================
// 目标动态更新测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, DynamicTargetUpdate) {
  generator_.setMaxVelocity(max_velocity_);
  generator_.setTarget(target_pos_);

  Eigen::VectorXd pos = current_pos_;
  Eigen::VectorXd vel = current_vel_;

  // 运动几步
  for (int i = 0; i < 50; ++i) {
    generator_.update(pos, vel, dt_);
    pos = generator_.getDesiredPos();
    vel = generator_.getDesiredVel();
  }

  // 更新目标到新位置
  Eigen::VectorXd new_target(3);
  new_target << -1.0, -2.0, -3.0;
  generator_.setTarget(new_target);

  // 误差应更新
  generator_.update(pos, vel, dt_);
  const Eigen::VectorXd& error = generator_.getError();

  Eigen::VectorXd expected_error = new_target - pos;
  EXPECT_NEAR(error[0], expected_error[0], 1e-9);
  EXPECT_NEAR(error[1], expected_error[1], 1e-9);
  EXPECT_NEAR(error[2], expected_error[2], 1e-9);
}

// ============================================================================
// isReached 阈值判断测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, IsReachedThreshold) {
  generator_.setMaxVelocity(max_velocity_);

  // 设置很近的目标
  Eigen::VectorXd close_target(3);
  close_target << 0.001, 0.001, 0.001;
  generator_.setTarget(close_target);
  generator_.update(current_pos_, current_vel_, dt_);

  // 误差范数约为 sqrt(3) * 0.001 ≈ 0.00173
  double error_norm = generator_.getError().norm();
  EXPECT_LT(error_norm, 0.002);

  // 大阈值应判定为到达
  EXPECT_TRUE(generator_.isReached(0.01));
  // 小阈值应判定为未到达
  EXPECT_FALSE(generator_.isReached(0.001));
}

// ============================================================================
// 多维关节协调测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, HighDimensionalCoordination) {
  // 模拟 7-DOF 机械臂
  const int dof = 7;
  Eigen::VectorXd start(dof), target(dof), max_vel(dof);
  start << 0.0, -0.5, 1.57, 0.0, -0.78, 0.0, 0.5;
  target << 0.5, 0.3, 0.78, -0.5, 0.0, 1.57, -0.5;
  max_vel.setConstant(0.5);

  generator_.setMaxVelocity(max_vel);
  generator_.setTarget(target);

  Eigen::VectorXd pos = start;
  Eigen::VectorXd vel = Eigen::VectorXd::Zero(dof);

  // 初始化 error_
  generator_.update(pos, vel, dt_);
  pos = generator_.getDesiredPos();
  vel = generator_.getDesiredVel();

  // 运动直到到达
  const int max_steps = 10000;
  int steps = 0;
  while (!generator_.isReached(1e-6) && steps < max_steps) {
    generator_.update(pos, vel, dt_);
    pos = generator_.getDesiredPos();
    vel = generator_.getDesiredVel();
    ++steps;
  }

  // 所有关节应到达目标
  EXPECT_TRUE(generator_.isReached(1e-6));
  for (int i = 0; i < dof; ++i) {
    EXPECT_NEAR(pos[i], target[i], 1e-6);
  }
}

// ============================================================================
// 边界条件测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, ZeroMovement) {
  generator_.setMaxVelocity(max_velocity_);

  // 目标等于当前位置
  generator_.setTarget(current_pos_);
  generator_.update(current_pos_, current_vel_, dt_);

  const Eigen::VectorXd& pos = generator_.getDesiredPos();
  const Eigen::VectorXd& vel = generator_.getDesiredVel();

  // 应保持在原位
  EXPECT_NEAR(pos[0], current_pos_[0], 1e-9);
  EXPECT_NEAR(pos[1], current_pos_[1], 1e-9);
  EXPECT_NEAR(pos[2], current_pos_[2], 1e-9);

  // 速度应为零
  EXPECT_NEAR(vel[0], 0.0, 1e-9);
  EXPECT_NEAR(vel[1], 0.0, 1e-9);
  EXPECT_NEAR(vel[2], 0.0, 1e-9);

  EXPECT_TRUE(generator_.isReached(1e-9));
}

TEST_F(VelocityLimitedGeneratorTest, LargeTarget) {
  generator_.setMaxVelocity(max_velocity_);

  // 设置很远的目标
  Eigen::VectorXd large_target(3);
  large_target << 1000.0, 2000.0, 3000.0;
  generator_.setTarget(large_target);
  generator_.update(current_pos_, current_vel_, dt_);

  const Eigen::VectorXd& vel = generator_.getDesiredVel();

  // 速度应被限制
  for (int i = 0; i < 3; ++i) {
    EXPECT_LE(std::abs(vel[i]), max_velocity_[i] + 1e-9);
  }
}

TEST_F(VelocityLimitedGeneratorTest, NegativeTarget) {
  generator_.setMaxVelocity(max_velocity_);

  Eigen::VectorXd negative_target(3);
  negative_target << -5.0, -10.0, -15.0;
  generator_.setTarget(negative_target);
  generator_.update(current_pos_, current_vel_, dt_);

  const Eigen::VectorXd& vel = generator_.getDesiredVel();

  // 速度应为负方向
  for (int i = 0; i < 3; ++i) {
    EXPECT_LT(vel[i], 0.0);
    EXPECT_GE(vel[i], -max_velocity_[i] - 1e-9);
  }
}

// ============================================================================
// Reset 测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, Reset) {
  generator_.setMaxVelocity(max_velocity_);
  generator_.setTarget(target_pos_);
  generator_.update(current_pos_, current_vel_, dt_);

  EXPECT_GT(generator_.getDimension(), 0);

  generator_.reset();

  EXPECT_EQ(generator_.getDimension(), 0);
  EXPECT_TRUE(generator_.isReached(1e-9));  // 无目标时视为到达
}

// ============================================================================
// 基类指针多态测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, PolymorphismThroughBasePointer) {
  std::unique_ptr<TrajectoryGeneratorBase> base_ptr =
      std::make_unique<VelocityLimitedGenerator>();

  EXPECT_STREQ(base_ptr->getName(), "VelocityLimited");

  base_ptr->setMaxVelocity(max_velocity_);
  base_ptr->setTarget(target_pos_);
  base_ptr->update(current_pos_, current_vel_, dt_);

  const Eigen::VectorXd& pos = base_ptr->getDesiredPos();
  EXPECT_EQ(pos.size(), 3);

  // 多次更新直到到达
  Eigen::VectorXd p = current_pos_;
  Eigen::VectorXd v = current_vel_;
  for (int i = 0; i < 10000 && !base_ptr->isReached(1e-6); ++i) {
    base_ptr->update(p, v, dt_);
    p = base_ptr->getDesiredPos();
    v = base_ptr->getDesiredVel();
  }

  EXPECT_TRUE(base_ptr->isReached(1e-6));

  base_ptr->reset();
  EXPECT_EQ(base_ptr->getDimension(), 0);
}

// ============================================================================
// 无目标时的行为测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, UpdateWithoutTarget) {
  // 未设置目标时调用 update
  generator_.update(current_pos_, current_vel_, dt_);

  const Eigen::VectorXd& pos = generator_.getDesiredPos();
  const Eigen::VectorXd& vel = generator_.getDesiredVel();

  // 应保持当前位置
  EXPECT_NEAR(pos[0], current_pos_[0], 1e-9);
  EXPECT_NEAR(pos[1], current_pos_[1], 1e-9);
  EXPECT_NEAR(pos[2], current_pos_[2], 1e-9);

  // 速度应为零
  EXPECT_NEAR(vel[0], 0.0, 1e-9);
  EXPECT_NEAR(vel[1], 0.0, 1e-9);
  EXPECT_NEAR(vel[2], 0.0, 1e-9);
}

// ============================================================================
// 维度不匹配测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, DimensionMismatch) {
  generator_.setMaxVelocity(max_velocity_);

  // 设置不同维度的目标
  Eigen::VectorXd wrong_dim_target(2);
  wrong_dim_target << 1.0, 2.0;
  generator_.setTarget(wrong_dim_target);

  // 用 3 维当前位置更新
  generator_.update(current_pos_, current_vel_, dt_);

  const Eigen::VectorXd& pos = generator_.getDesiredPos();

  // 维度不匹配时应保持当前位置
  EXPECT_NEAR(pos[0], current_pos_[0], 1e-9);
  EXPECT_NEAR(pos[1], current_pos_[1], 1e-9);
  EXPECT_NEAR(pos[2], current_pos_[2], 1e-9);
}

// ============================================================================
// 默认最大速度测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, DefaultMaxVelocity) {
  // 不设置 max_velocity，使用默认值
  generator_.setTarget(target_pos_);
  generator_.update(current_pos_, current_vel_, dt_);

  const Eigen::VectorXd& vel = generator_.getDesiredVel();

  // 默认最大速度为 1.0 rad/s
  for (int i = 0; i < 3; ++i) {
    EXPECT_LE(std::abs(vel[i]), 1.0 + 1e-9);
  }
}

// ============================================================================
// 模式切换测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, ModeSwitching) {
  generator_.setMaxVelocity(max_velocity_);
  generator_.setTarget(target_pos_);

  // kPerJoint 模式
  generator_.setLimitMode(VelocityLimitMode::kPerJoint);
  generator_.update(current_pos_, current_vel_, dt_);
  Eigen::VectorXd vel_per_joint = generator_.getDesiredVel();

  // 切换到 kSynchronized 模式
  generator_.setLimitMode(VelocityLimitMode::kSynchronized);
  generator_.update(current_pos_, current_vel_, dt_);
  Eigen::VectorXd vel_sync = generator_.getDesiredVel();

  // 两种模式应产生不同的速度（除非特殊情况）
  // 由于目标 [1, 2, 3] 各分量不同，结果应该不同
  bool different = false;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(vel_per_joint[i] - vel_sync[i]) > 1e-9) {
      different = true;
      break;
    }
  }
  EXPECT_TRUE(different);
}

// ============================================================================
// 可视化数据导出测试
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, ExportDataForVisualization) {
  // 7-DOF 机械臂轨迹
  const int num_joints = 7;
  Eigen::VectorXd start(num_joints), target(num_joints), max_vel(num_joints);
  start << 0.0, -0.5, 1.57, 0.0, -0.78, 0.0, 0.5;
  target << 0.8, 0.3, 0.78, -0.5, 0.0, 1.57, -0.5;
  max_vel << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;

  // 输出 CSV 文件
  std::ofstream file("/tmp/trajectory_generator_data.csv");

  // 写入表头
  file << "time";
  for (int j = 0; j < num_joints; ++j) {
    file << ",j" << j << "_pos_pj,j" << j << "_pos_sync"
         << ",j" << j << "_vel_pj,j" << j << "_vel_sync";
  }
  file << "\n";

  VelocityLimitedGenerator gen_pj(VelocityLimitMode::kPerJoint);
  VelocityLimitedGenerator gen_sync(VelocityLimitMode::kSynchronized);

  gen_pj.setMaxVelocity(max_vel);
  gen_sync.setMaxVelocity(max_vel);
  gen_pj.setTarget(target);
  gen_sync.setTarget(target);

  Eigen::VectorXd pos_pj = start;
  Eigen::VectorXd pos_sync = start;
  Eigen::VectorXd vel = Eigen::VectorXd::Zero(num_joints);

  const double dt = 0.01;
  const double duration = 10.0;
  const int samples = static_cast<int>(duration / dt);

  for (int i = 0; i <= samples; ++i) {
    double t = i * dt;

    gen_pj.update(pos_pj, vel, dt);
    gen_sync.update(pos_sync, vel, dt);

    file << t;
    for (int j = 0; j < num_joints; ++j) {
      file << "," << gen_pj.getDesiredPos()[j]
           << "," << gen_sync.getDesiredPos()[j]
           << "," << gen_pj.getDesiredVel()[j]
           << "," << gen_sync.getDesiredVel()[j];
    }
    file << "\n";

    pos_pj = gen_pj.getDesiredPos();
    pos_sync = gen_sync.getDesiredPos();
  }
  file.close();

  std::cout << "Data exported to /tmp/trajectory_generator_data.csv\n";
  std::cout << num_joints << "-DOF trajectory:\n";
  for (int j = 0; j < num_joints; ++j) {
    std::cout << "  Joint " << j << ": " << start[j] << " -> " << target[j] << "\n";
  }
  std::cout << "Max velocity: " << max_vel.transpose() << "\n";
}

// ============================================================================
// 目标动态变化可视化数据导出
// ============================================================================

TEST_F(VelocityLimitedGeneratorTest, ExportDynamicTargetData) {
  // 3-DOF 简化示例，展示目标变化响应
  const int num_joints = 3;
  Eigen::VectorXd start(num_joints), max_vel(num_joints);
  start << 0.5, -0.3, 0.8;              // 不同的初始位置
  max_vel << 0.5, 0.6, 0.4;             // 不同的速度限制

  // 目标序列: 在运动过程中切换目标（未到达就切换）
  std::vector<std::pair<double, Eigen::VectorXd>> target_sequence;
  Eigen::VectorXd t1(3), t2(3), t3(3);
  t1 << 2.0, 1.2, -0.5;   // 目标1: 各关节方向不同
  t2 << -0.8, 2.5, 1.0;   // 目标2: 在 1.5s 时切换
  t3 << 1.5, 0.0, 2.0;    // 目标3: 在 3.5s 时切换
  target_sequence.push_back({0.0, t1});   // t=0: 目标1
  target_sequence.push_back({1.5, t2});   // t=1.5: 中途切换到目标2
  target_sequence.push_back({3.5, t3});   // t=3.5: 中途切换到目标3

  // 输出 CSV 文件
  std::ofstream file("/tmp/trajectory_generator_dynamic.csv");

  // 写入表头
  file << "time";
  for (int j = 0; j < num_joints; ++j) {
    file << ",j" << j << "_pos,j" << j << "_vel,j" << j << "_target";
  }
  file << "\n";

  VelocityLimitedGenerator gen(VelocityLimitMode::kPerJoint);
  gen.setMaxVelocity(max_vel);

  Eigen::VectorXd pos = start;
  Eigen::VectorXd vel = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd current_target = t1;
  gen.setTarget(current_target);

  const double dt = 0.01;
  const double duration = 8.0;
  const int samples = static_cast<int>(duration / dt);
  size_t target_idx = 0;

  for (int i = 0; i <= samples; ++i) {
    double t = i * dt;

    // 检查是否需要切换目标
    while (target_idx + 1 < target_sequence.size() &&
           t >= target_sequence[target_idx + 1].first) {
      target_idx++;
      current_target = target_sequence[target_idx].second;
      gen.setTarget(current_target);
    }

    gen.update(pos, vel, dt);

    file << t;
    for (int j = 0; j < num_joints; ++j) {
      file << "," << gen.getDesiredPos()[j]
           << "," << gen.getDesiredVel()[j]
           << "," << current_target[j];
    }
    file << "\n";

    pos = gen.getDesiredPos();
    vel = gen.getDesiredVel();
  }
  file.close();

  std::cout << "Dynamic target data exported to /tmp/trajectory_generator_dynamic.csv\n";
  std::cout << "Target changes at: ";
  for (const auto& ts : target_sequence) {
    std::cout << "t=" << ts.first << "s ";
  }
  std::cout << "\n";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
