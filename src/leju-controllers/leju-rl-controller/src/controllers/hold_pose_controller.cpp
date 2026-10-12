#include "leju-rl-controller/controllers/hold_pose_controller.h"

#include <filesystem>

#include <yaml-cpp/yaml.h>

#include "leju-rl-controller/controllers/controller_registry.h"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/time_utils.hpp"

namespace leju {

HoldPoseController::HoldPoseController(const RobotVersion& version,
                                       const std::string& name)
    : robot_version_(version) {
  name_ = name;
}

bool HoldPoseController::loadHoldConfig(const std::string& config_path) {
  if (config_path.empty() || !std::filesystem::exists(config_path)) {
    RL_LOG_FAILURE("HoldPoseController: config not found: %s", config_path.c_str());
    return false;
  }

  try {
    YAML::Node root = YAML::LoadFile(config_path);
    if (root["loop_dt"]) {
      loop_dt_ = root["loop_dt"].as<double>();
    }
    if (root["kp"] && !root["kp"].IsSequence()) {
      fallback_kp_ = root["kp"].as<double>();
    }
    if (root["kd"] && !root["kd"].IsSequence()) {
      fallback_kd_ = root["kd"].as<double>();
    }

    // 支持直接写在根节点，或从 HumanoidRobotCfg.env.robot 读取（便于复用 amp 字段）
    YAML::Node robot = root["robot"];
    if (!robot && root["HumanoidRobotCfg"] && root["HumanoidRobotCfg"]["env"]) {
      robot = root["HumanoidRobotCfg"]["env"]["robot"];
      if (root["HumanoidRobotCfg"]["loop_dt"]) {
        loop_dt_ = root["HumanoidRobotCfg"]["loop_dt"].as<double>();
      }
    }
    if (!robot) {
      RL_LOG_FAILURE("HoldPoseController: missing robot / HumanoidRobotCfg.env.robot");
      return false;
    }

    joint_names_ = robot["joint_names"].as<std::vector<std::string>>();
    auto kp_vec = robot["actuator_kp"].as<std::vector<double>>();
    auto kd_vec = robot["actuator_kd"].as<std::vector<double>>();

    if (joint_names_.size() != kp_vec.size() ||
        joint_names_.size() != kd_vec.size()) {
      RL_LOG_FAILURE("HoldPoseController: joint_* array size mismatch");
      return false;
    }

    joint_kp_ = Eigen::Map<const Eigen::ArrayXd>(kp_vec.data(), kp_vec.size());
    joint_kd_ = Eigen::Map<const Eigen::ArrayXd>(kd_vec.data(), kd_vec.size());
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("HoldPoseController config error: %s", e.what());
    return false;
  }
  return true;
}

bool HoldPoseController::buildJointMapping() {
  auto& robot = GlobalRobot::getInstance();
  motor_count_ = robot.getMotorNumber();
  motor_names_ = robot.getMotorNames();
  policy_joint_ids_.clear();
  for (const auto& joint_name : joint_names_) {
    int joint_id = -1;
    for (int j = 0; j < motor_count_; ++j) {
      if (joint_name == motor_names_[j]) {
        joint_id = j;
        break;
      }
    }
    if (joint_id < 0) {
      RL_LOG_FAILURE("HoldPoseController: joint not found: %s", joint_name.c_str());
      return false;
    }
    policy_joint_ids_.push_back(joint_id);
  }
  policy_joint_count_ = static_cast<int>(policy_joint_ids_.size());
  return true;
}

bool HoldPoseController::initialize() {
  RL_LOGI("Initializing HoldPoseController '%s'...", name_.c_str());
  if (!loadHoldConfig(config_path_)) return false;
  if (!buildJointMapping()) return false;

  pose_captured_ = false;
  hold_q_.assign(static_cast<size_t>(motor_count_), 0.0);
  state_ = ControllerState::kPaused;
  RL_LOG_SUCCESS("HoldPoseController ready (motors=%d, policy_joints=%d)",
                 motor_count_, policy_joint_count_);
  return true;
}

void HoldPoseController::reset() {
  ControllerBase::reset();
  pose_captured_ = false;
}

bool HoldPoseController::updateImpl(double, const RobotState& state,
                                    const ImuData&, RobotCmd& cmd) {
  if (static_cast<int>(state.q.size()) < motor_count_) {
    return false;
  }
  if (!pose_captured_) {
    hold_q_.assign(state.q.begin(), state.q.begin() + motor_count_);
    pose_captured_ = true;
    RL_LOGI("HoldPoseController: captured hold pose from sensor feedback");
  }

  if (cmd.q.size() != static_cast<size_t>(motor_count_)) {
    cmd.resize(static_cast<size_t>(motor_count_));
  }
  for (int i = 0; i < motor_count_; ++i) {
    cmd.q[i] = hold_q_[static_cast<size_t>(i)];
    cmd.v[i] = 0.0;
    cmd.tau[i] = 0.0;
    cmd.kp[i] = fallback_kp_;
    cmd.kd[i] = fallback_kd_;
    // modes=2=位控；fallback_kp_/kd_(50/2) 冻结未配置关节（手臂/头），策略关节被 joint_kp_/kd_ 覆盖
    cmd.modes[i] = 2;
  }
  for (int i = 0; i < policy_joint_count_; ++i) {
    int motor_idx = policy_joint_ids_[i];
    if (motor_idx >= 0 && motor_idx < motor_count_) {
      cmd.kp[motor_idx] = joint_kp_[i];
      cmd.kd[motor_idx] = joint_kd_[i];
    }
  }
  cmd.timestamp = leju::common::GetUnixTimestampS();
  return true;
}

REGISTER_CONTROLLER("HoldPoseController", HoldPoseController, hold_pose);
void ForceLinkHoldPoseController() {}

}  // namespace leju
