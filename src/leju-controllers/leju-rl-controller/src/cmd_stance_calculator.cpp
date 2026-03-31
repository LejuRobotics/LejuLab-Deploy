/**
 * @file cmd_stance_calculator.cpp
 * @brief cmdStance 计算器实现
 */

#include "leju-rl-controller/cmd_stance_calculator.h"
#include <cmath>
#include <iostream>

namespace leju {
namespace rl_demo {

CmdStanceCalculator::CmdStanceCalculator(const CmdStanceConfig& config)
    : config_(config) {}

void CmdStanceCalculator::setConfig(const CmdStanceConfig& config) {
  config_ = config;
}

double CmdStanceCalculator::compute(
    const Eigen::Vector3d& velocity_commands,
    const Eigen::VectorXd& torso_state,
    const Eigen::VectorXd& feet_positions) {
  // 1. 速度大 → cmdStance=0（行走）
  double vel_mag = std::sqrt(
      velocity_commands(0) * velocity_commands(0) +
      velocity_commands(1) * velocity_commands(1) +
      0.01 * velocity_commands(2) * velocity_commands(2));  // 角速度权重
  if (vel_mag > config_.velocity_magnitude_threshold) {
    return 0.0;
  }

  // 2. 速度小 + 智能停止条件满足 → cmdStance=1（站立）
  if (config_.smart_stop_enabled &&
      shouldSmartStop(torso_state, feet_positions)) {
    return 1.0;
  }

  // 3. 速度小：若无 torso/feet 数据（computeSimple），默认站立；否则保持行走等待停止
  // 用户期望：松开摇杆即站立，推摇杆才走路
  if (torso_state.size() < 12 || feet_positions.size() < 24) {
    return 1.0;  // 无智能停止数据时，速度小即站立
  }
  return 0.0;  // 有数据但不满足停止条件，保持行走
}

double CmdStanceCalculator::computeSimple(
    const Eigen::Vector3d& velocity_commands) {
  Eigen::VectorXd empty_torso;
  Eigen::VectorXd empty_feet;
  return compute(velocity_commands, empty_torso, empty_feet);
}

bool CmdStanceCalculator::shouldSmartStop(
    const Eigen::VectorXd& torso_state,
    const Eigen::VectorXd& feet_positions) {
  if (torso_state.size() < 12 || feet_positions.size() < 24) {
    return false;
  }

  // torsostate: [x, y, z, yaw, pitch, roll, vx, vy, vz, wx, wy, wz]
  Eigen::Vector3d global_linear_vel = torso_state.segment<3>(6);
  Eigen::Vector3d orientation = torso_state.segment<3>(3);
  Eigen::Vector3d torso_position = torso_state.segment<3>(0);

  double yaw = orientation(0);
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  // 1. 躯干前向速度（局部坐标系）
  double forward_velocity =
      global_linear_vel(0) * cos_yaw + global_linear_vel(1) * sin_yaw;
  bool torso_slow =
      std::abs(forward_velocity) < config_.torso_velocity_threshold;

  // 2. 双脚对齐（局部 x 方向）
  Eigen::Vector3d lf_pos_w = Eigen::Vector3d::Zero();
  for (int i = 0; i < 4; i++) {
    lf_pos_w.head<2>() += feet_positions.segment<2>(i * 3) / 4.0;
  }
  Eigen::Vector3d rf_pos_w = Eigen::Vector3d::Zero();
  for (int i = 0; i < 4; i++) {
    rf_pos_w.head<2>() += feet_positions.segment<2>(i * 3 + 12) / 4.0;
  }

  Eigen::Vector3d lf_pos_local = lf_pos_w - torso_position;
  Eigen::Vector3d rf_pos_local = rf_pos_w - torso_position;

  double lf_x_body = lf_pos_local(0) * cos_yaw + lf_pos_local(1) * sin_yaw;
  double rf_x_body = rf_pos_local(0) * cos_yaw + rf_pos_local(1) * sin_yaw;
  double feet_x_diff = std::abs(lf_x_body - rf_x_body);
  bool feet_aligned = feet_x_diff < config_.feet_alignment_threshold;

  return torso_slow && feet_aligned;
}

}  // namespace rl_demo
}  // namespace leju
