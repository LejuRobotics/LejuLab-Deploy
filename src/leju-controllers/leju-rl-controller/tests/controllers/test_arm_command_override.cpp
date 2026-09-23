/**
 * @file test_arm_command_override.cpp
 * @brief ControllerBase::updateArmCommand 覆盖行为测试
 *
 * 背景：CST 配置（actuator_control_mode=0）下 updateRobotCmd 会把手臂关节的
 * modes/kp/kd 置 0，而 updateArmCommand 覆盖时又会把 tau 改写为重力补偿力矩
 * （pinocchio 不可用时为 0）。若覆盖时不强制 CSP 并恢复 kp/kd，手臂将处于
 * 零增益零力矩状态（站立时手臂完全松软）。
 *
 * 验收用例：
 *   1. kAuto 站立覆盖（模拟 CST 上游输出）→ 手臂 modes=2、kp/kd 恢复为配置值
 *   2. kAuto 行走（不覆盖）→ cmd 原样保留，RL 摆臂不受影响
 *   3. 非手臂关节在任何情况下不被 updateArmCommand 修改
 */

#include <gtest/gtest.h>

#include "leju-rl-controller/controllers/controller_base.h"

namespace leju {
namespace {

constexpr int kMotorCount = 4;      // 0,1 = 腿；2,3 = 手臂
constexpr double kArmKp1 = 30.0;
constexpr double kArmKp2 = 20.0;
constexpr double kArmKd = 3.0;

/// 暴露 protected 成员与 updateArmCommand 的测试桩
class TestableController : public ControllerBase {
 public:
  TestableController() {
    // 关节布局：policy 顺序与电机索引一致
    arm_joint_names_ = {"zarm_1", "zarm_2"};
    arm_joint_ids_ = {2, 3};
    arm_policy_start_idx_ = 2;

    joint_direction_ = array_t::Ones(kMotorCount);
    joint_kp_ = array_t(kMotorCount);
    joint_kp_ << 100.0, 100.0, kArmKp1, kArmKp2;
    joint_kd_ = array_t(kMotorCount);
    joint_kd_ << 4.0, 4.0, kArmKd, kArmKd;

    current_state_.resize(kMotorCount);

    MultiModeArmControllerConfig config;
    arm_controller_ = std::make_unique<MultiModeArmController>(config);
    Eigen::VectorXd default_arm_pos(2);
    default_arm_pos << 0.1, -0.2;
    arm_controller_->init(2, default_arm_pos, 0.001);
  }

  void setStanding() { velocity_cmd_.setZero(); }
  void setWalking() { velocity_cmd_.linear_x = 0.5; }

  using ControllerBase::updateArmCommand;

  // 纯虚接口桩实现（本测试不使用）
  bool initialize() override { return true; }

 protected:
  bool updateImpl(double, const RobotState&, const ImuData&, RobotCmd&) override {
    return true;
  }
  bool loadPolicy(const std::string&) override { return true; }
  void computeObservation() override {}
  void computeActions() override {}
  void updateRobotCmd(RobotCmd&) override {}
};

/// 模拟 CST 配置下 updateRobotCmd 的输出：modes=0、kp=kd=0、tau=软件 PD 力矩
RobotCmd MakeCstUpstreamCmd() {
  RobotCmd cmd(kMotorCount);
  for (int i = 0; i < kMotorCount; ++i) {
    cmd.q[i] = 0.0;
    cmd.v[i] = 0.0;
    cmd.tau[i] = 5.0;
    cmd.kp[i] = 0.0;
    cmd.kd[i] = 0.0;
    cmd.modes[i] = 0;  // CST
  }
  return cmd;
}

// 用例 1：kAuto 站立覆盖时，手臂必须被强制 CSP 且恢复 kp/kd
TEST(ArmCommandOverride, KAutoStandingForcesCspAndRestoresGains) {
  TestableController controller;
  controller.setStanding();

  RobotCmd cmd = MakeCstUpstreamCmd();
  controller.updateArmCommand(cmd);

  // 手臂关节：CSP + 配置 kp/kd
  EXPECT_EQ(cmd.modes[2], 2);
  EXPECT_EQ(cmd.modes[3], 2);
  EXPECT_DOUBLE_EQ(cmd.kp[2], kArmKp1);
  EXPECT_DOUBLE_EQ(cmd.kp[3], kArmKp2);
  EXPECT_DOUBLE_EQ(cmd.kd[2], kArmKd);
  EXPECT_DOUBLE_EQ(cmd.kd[3], kArmKd);
  // 重力补偿未初始化（无 pinocchio/URDF）→ tau 被覆盖为 0，
  // 此时若无上面的 kp 恢复，手臂就是零力矩
  EXPECT_DOUBLE_EQ(cmd.tau[2], 0.0);
  EXPECT_DOUBLE_EQ(cmd.tau[3], 0.0);
}

// 用例 2：kAuto 行走（cmd_stance=0）不覆盖，RL 输出原样保留
TEST(ArmCommandOverride, KAutoWalkingLeavesCmdUntouched) {
  TestableController controller;
  controller.setWalking();

  RobotCmd cmd = MakeCstUpstreamCmd();
  controller.updateArmCommand(cmd);

  for (int i = 0; i < kMotorCount; ++i) {
    EXPECT_EQ(cmd.modes[i], 0) << "joint " << i;
    EXPECT_DOUBLE_EQ(cmd.kp[i], 0.0) << "joint " << i;
    EXPECT_DOUBLE_EQ(cmd.kd[i], 0.0) << "joint " << i;
    EXPECT_DOUBLE_EQ(cmd.tau[i], 5.0) << "joint " << i;
  }
}

// 用例 3：站立覆盖时非手臂关节（腿）不受影响
TEST(ArmCommandOverride, LegJointsNeverModified) {
  TestableController controller;
  controller.setStanding();

  RobotCmd cmd = MakeCstUpstreamCmd();
  controller.updateArmCommand(cmd);

  for (int i : {0, 1}) {
    EXPECT_EQ(cmd.modes[i], 0) << "joint " << i;
    EXPECT_DOUBLE_EQ(cmd.kp[i], 0.0) << "joint " << i;
    EXPECT_DOUBLE_EQ(cmd.kd[i], 0.0) << "joint " << i;
    EXPECT_DOUBLE_EQ(cmd.tau[i], 5.0) << "joint " << i;
  }
}

}  // namespace
}  // namespace leju
