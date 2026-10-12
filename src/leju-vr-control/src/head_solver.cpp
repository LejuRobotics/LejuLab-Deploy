/**
 * @file head_solver.cpp
 * @brief Compute head yaw/pitch from Quest bone poses
 */

#include "leju-vr-control/head_solver.h"

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace leju {
namespace vr_control {

namespace {
constexpr int POSE_INDEX_CHEST = 23;
constexpr int POSE_INDEX_HEAD = 25;

void quatToEulerXYZ(const Eigen::Quaterniond& q, double& yaw, double& pitch, double& roll) {
  // 头部旋转使用外旋XYZ顺序
  Eigen::Matrix3d R = q.toRotationMatrix();
  double X, Y, Z;
  Y = std::asin(R(0, 2));
  if (std::abs(std::cos(pitch)) > 1e-6) {
    Z  = std::atan2(-R(0, 1), R(0, 0));
    X = std::atan2(-R(1, 2), R(2, 2));
  } else { // 奇异点 (Y ≈ ±π/2)，自由度退化，设定 yaw = 0
    Z  = 0.0;
    X = std::atan2(R(1, 0), R(1, 1));
  }
  // Extract pitch (around X-axis) and yaw (around Y-axis)
  pitch = X;
  yaw = Y;
  roll = Z;
}
}  // namespace

bool computeHeadFromBones(const QuestBonePosesData& quest_poses,
                          std::vector<double>& out_q) {
  const auto& poses = quest_poses.poses;
  if (poses.size() <= static_cast<size_t>(POSE_INDEX_HEAD)) {
    return false;
  }

  const auto& chest = poses[POSE_INDEX_CHEST];
  const auto& head = poses[POSE_INDEX_HEAD];

  Eigen::Quaterniond q_chest(static_cast<double>(chest.qw),
                             static_cast<double>(chest.qx),
                             static_cast<double>(chest.qy),
                             static_cast<double>(chest.qz));
  Eigen::Quaterniond q_head(static_cast<double>(head.qw),
                            static_cast<double>(head.qx),
                            static_cast<double>(head.qy),
                            static_cast<double>(head.qz));

  Eigen::Quaterniond q_rel = q_chest.inverse() * q_head;
  double yaw, pitch, roll;
  quatToEulerXYZ(q_rel, yaw, pitch, roll);
  constexpr double pitch_max = static_cast<double>(M_PI/6.0);
  constexpr double pitch_min = -static_cast<double>(M_PI/9.0);
  yaw = std::clamp(yaw, -static_cast<double>(M_PI_2), static_cast<double>(M_PI_2));
  pitch = std::clamp(pitch, pitch_min, pitch_max);
  out_q.resize(2);
  out_q[0] = yaw;
  out_q[1] = pitch;
  return true;
}

}  // namespace vr_control
}  // namespace leju
