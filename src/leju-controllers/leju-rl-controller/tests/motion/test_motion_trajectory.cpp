#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include <fstream>

#include "leju-rl-controller/motion/motion_trajectory.h"

using namespace leju;

class MotionTrajectoryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 创建临时测试文件（标准语义格式）
    test_file_path_ = "/tmp/test_motion_data.csv";
    createTestFile();
  }

  void TearDown() override {
    // 删除临时测试文件
    std::remove(test_file_path_.c_str());
  }

  void createTestFile() {
    std::ofstream file(test_file_path_);
    // 使用标准语义格式: body_pos(3) + body_quat(4) + joint_pos(2) + joint_vel(2)
    file << "body_pos_x,body_pos_y,body_pos_z,body_quat_w,body_quat_x,body_quat_y,body_quat_z,joint_pos00,joint_pos01,joint_vel_00,joint_vel_01\n";
    file << "0.0,0.0,0.8,1.0,0.0,0.0,0.0,0.1,0.2,0.01,0.02\n";
    file << "0.1,0.0,0.81,0.999,0.0,0.0,0.044,0.11,0.21,0.011,0.021\n";
    file << "0.2,0.0,0.82,0.996,0.0,0.0,0.087,0.12,0.22,0.012,0.022\n";
    file << "0.3,0.0,0.83,0.991,0.0,0.0,0.131,0.13,0.23,0.013,0.023\n";
    file << "0.4,0.0,0.84,0.984,0.0,0.0,0.174,0.14,0.24,0.014,0.024\n";
    file.close();
  }

  std::string test_file_path_;
  MotionTrajectory loader_;
};

// ============================================================================
// 基础加载测试
// ============================================================================

TEST_F(MotionTrajectoryTest, InitialState) {
  EXPECT_FALSE(loader_.isLoaded());
  EXPECT_EQ(loader_.getNumFrames(), 0);
  EXPECT_EQ(loader_.getCurrentFrame(), 0);
}

TEST_F(MotionTrajectoryTest, LoadValidFile) {
  EXPECT_TRUE(loader_.load(test_file_path_));
  EXPECT_TRUE(loader_.isLoaded());
  EXPECT_EQ(loader_.getNumFrames(), 5);
}

TEST_F(MotionTrajectoryTest, LoadInvalidFile) {
  EXPECT_FALSE(loader_.load("/nonexistent/path/file.csv"));
  EXPECT_FALSE(loader_.isLoaded());
}

TEST_F(MotionTrajectoryTest, LoadTabDelimited) {
  // 创建 tab 分隔的测试文件
  std::string tab_path = "/tmp/test_motion_tab.csv";
  {
    std::ofstream file(tab_path);
    file << "body_pos_x\tbody_pos_y\tbody_pos_z\tbody_quat_w\tbody_quat_x\tbody_quat_y\tbody_quat_z\tjoint_pos00\tjoint_pos01\tjoint_vel_00\tjoint_vel_01\n";
    file << "0.0\t0.0\t0.9\t1.0\t0.0\t0.0\t0.0\t0.5\t0.6\t0.05\t0.06\n";
    file << "0.1\t0.0\t0.91\t0.999\t0.0\t0.0\t0.044\t0.51\t0.61\t0.051\t0.061\n";
    file.close();
  }

  MotionTrajectory tab_loader;
  EXPECT_TRUE(tab_loader.load(tab_path));
  EXPECT_EQ(tab_loader.getNumFrames(), 2);

  // 验证数据正确解析
  array_t joint_pos = tab_loader.getJointPos();
  EXPECT_DOUBLE_EQ(joint_pos[0], 0.5);
  EXPECT_DOUBLE_EQ(joint_pos[1], 0.6);

  std::remove(tab_path.c_str());
}

// ============================================================================
// 语义数据访问测试
// ============================================================================

TEST_F(MotionTrajectoryTest, GetJointPos) {
  loader_.load(test_file_path_);

  array_t joint_pos = loader_.getJointPos();
  ASSERT_EQ(joint_pos.size(), 2);
  EXPECT_DOUBLE_EQ(joint_pos[0], 0.1);
  EXPECT_DOUBLE_EQ(joint_pos[1], 0.2);
}

TEST_F(MotionTrajectoryTest, GetJointVel) {
  loader_.load(test_file_path_);

  array_t joint_vel = loader_.getJointVel();
  ASSERT_EQ(joint_vel.size(), 2);
  EXPECT_DOUBLE_EQ(joint_vel[0], 0.01);
  EXPECT_DOUBLE_EQ(joint_vel[1], 0.02);
}

TEST_F(MotionTrajectoryTest, GetBodyPos) {
  loader_.load(test_file_path_);

  Eigen::Vector3d body_pos = loader_.getBodyPos();
  EXPECT_DOUBLE_EQ(body_pos[0], 0.0);  // x
  EXPECT_DOUBLE_EQ(body_pos[1], 0.0);  // y
  EXPECT_DOUBLE_EQ(body_pos[2], 0.8);  // z
}

TEST_F(MotionTrajectoryTest, GetBodyQuat) {
  loader_.load(test_file_path_);

  Eigen::Quaterniond body_quat = loader_.getBodyQuat();
  EXPECT_DOUBLE_EQ(body_quat.w(), 1.0);
  EXPECT_DOUBLE_EQ(body_quat.x(), 0.0);
  EXPECT_DOUBLE_EQ(body_quat.y(), 0.0);
  EXPECT_DOUBLE_EQ(body_quat.z(), 0.0);
}

TEST_F(MotionTrajectoryTest, GetCurrentCommand) {
  loader_.load(test_file_path_);

  Eigen::VectorXd cmd = loader_.getCurrentCommand();
  ASSERT_EQ(cmd.size(), 4);  // joint_pos(2) + joint_vel(2)
  EXPECT_DOUBLE_EQ(cmd[0], 0.1);   // joint_pos[0]
  EXPECT_DOUBLE_EQ(cmd[1], 0.2);   // joint_pos[1]
  EXPECT_DOUBLE_EQ(cmd[2], 0.01);  // joint_vel[0]
  EXPECT_DOUBLE_EQ(cmd[3], 0.02);  // joint_vel[1]
}

TEST_F(MotionTrajectoryTest, GetReferenceYaw) {
  loader_.load(test_file_path_);

  // 第一帧四元数是 (1, 0, 0, 0)，yaw = 0
  double ref_yaw = loader_.getReferenceYaw();
  EXPECT_NEAR(ref_yaw, 0.0, 1e-6);
}

// ============================================================================
// 帧迭代测试
// ============================================================================

TEST_F(MotionTrajectoryTest, NextFrame) {
  loader_.load(test_file_path_);

  EXPECT_EQ(loader_.getCurrentFrame(), 0);

  // 前进一帧
  loader_.next();
  EXPECT_EQ(loader_.getCurrentFrame(), 1);

  // 验证数据是第二帧的
  array_t joint_pos = loader_.getJointPos();
  EXPECT_DOUBLE_EQ(joint_pos[0], 0.11);
  EXPECT_DOUBLE_EQ(joint_pos[1], 0.21);
}

TEST_F(MotionTrajectoryTest, HasNext) {
  loader_.load(test_file_path_);

  EXPECT_TRUE(loader_.hasNext());

  // 遍历到最后一帧
  for (int i = 0; i < 4; ++i) {
    loader_.next();
  }

  // 在最后一帧时，hasNext 应该返回 false
  EXPECT_FALSE(loader_.hasNext());
  EXPECT_EQ(loader_.getCurrentFrame(), 4);
}

TEST_F(MotionTrajectoryTest, Reset) {
  loader_.load(test_file_path_);

  // 前进几帧
  loader_.next();
  loader_.next();
  EXPECT_EQ(loader_.getCurrentFrame(), 2);

  // 重置
  loader_.reset();
  EXPECT_EQ(loader_.getCurrentFrame(), 0);

  // 验证数据是第一帧的
  array_t joint_pos = loader_.getJointPos();
  EXPECT_DOUBLE_EQ(joint_pos[0], 0.1);
}

// ============================================================================
// 边界条件测试
// ============================================================================

TEST_F(MotionTrajectoryTest, NextAtLastFrame) {
  loader_.load(test_file_path_);

  // 前进到最后一帧
  for (int i = 0; i < 10; ++i) {  // 超过帧数
    loader_.next();
  }

  // 应该停留在最后一帧
  EXPECT_EQ(loader_.getCurrentFrame(), 4);

  // 数据仍然可以访问
  array_t joint_pos = loader_.getJointPos();
  EXPECT_DOUBLE_EQ(joint_pos[0], 0.14);
}

TEST_F(MotionTrajectoryTest, GetDataBeforeLoad) {
  // 未加载时获取数据应返回空/默认值
  array_t joint_pos = loader_.getJointPos();
  EXPECT_EQ(joint_pos.size(), 0);

  array_t joint_vel = loader_.getJointVel();
  EXPECT_EQ(joint_vel.size(), 0);

  Eigen::Vector3d body_pos = loader_.getBodyPos();
  EXPECT_DOUBLE_EQ(body_pos.norm(), 0.0);

  Eigen::Quaterniond body_quat = loader_.getBodyQuat();
  EXPECT_DOUBLE_EQ(body_quat.w(), 1.0);  // Identity quaternion

  Eigen::VectorXd cmd = loader_.getCurrentCommand();
  EXPECT_EQ(cmd.size(), 0);
}

TEST_F(MotionTrajectoryTest, InvalidColumnCount) {
  // 创建列数不符合语义格式的文件
  std::string invalid_path = "/tmp/test_motion_invalid.csv";
  {
    std::ofstream file(invalid_path);
    file << "a,b,c,d,e\n";  // 5列，不符合 7 + 2*N 格式
    file << "1,2,3,4,5\n";
    file.close();
  }

  MotionTrajectory invalid_loader;
  EXPECT_FALSE(invalid_loader.load(invalid_path));

  std::remove(invalid_path.c_str());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
