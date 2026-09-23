#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "leju-rl-controller/controllers/controller_base.h"
#include "leju-rl-controller/rl/rl_controller_types.h"
#include "lejusdk-utils/robot_version.hpp"

namespace leju {

/**
 * HoldPoseController（搬运模式 LOCK 用）
 * - 激活后首拍捕获当前全身关节反馈位，之后全电机位控冻结，不跑策略
 * - 机器人变为刚性"雕像"，可被安全搬运；退出由外部切换控制器完成
 */
class HoldPoseController : public ControllerBase {
 public:
  HoldPoseController(const RobotVersion& version, const std::string& name);
  ~HoldPoseController() override = default;

  bool initialize() override;
  void reset() override;

  bool setConfigPath(const std::string& config_path) override {
    config_path_ = config_path;
    return true;
  }

 protected:
  bool updateImpl(double time, const RobotState& state, const ImuData& imu,
                  RobotCmd& cmd) override;
  bool loadPolicy(const std::string&) override { return true; }
  void computeObservation() override {}
  void computeActions() override {}
  void updateRobotCmd(RobotCmd&) override {}

 private:
  bool loadHoldConfig(const std::string& config_path);
  bool buildJointMapping();

  RobotVersion robot_version_;
  std::string config_path_;
  int motor_count_ = 0;
  std::vector<std::string> motor_names_;

  std::vector<std::string> joint_names_;
  std::vector<int> policy_joint_ids_;
  int policy_joint_count_ = 0;
  array_t joint_kp_;
  array_t joint_kd_;

  double fallback_kp_ = 50.0;
  double fallback_kd_ = 2.0;

  bool pose_captured_ = false;
  std::vector<double> hold_q_;
};

void ForceLinkHoldPoseController();

}  // namespace leju
