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

void quatToEulerZYX(const Eigen::Quaterniond& q, double& yaw, double& pitch, double& roll) {
  Eigen::Matrix3d R = q.toRotationMatrix();
  pitch = std::asin(-R(2, 0));
  if (std::abs(std::cos(pitch)) > 1e-6) {
    yaw = std::atan2(R(1, 0), R(0, 0));
    roll = std::atan2(R(2, 1), R(2, 2));
  } else {
    yaw = 0.0;
    roll = std::atan2(-R(0, 1), R(1, 1));
  }
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
  quatToEulerZYX(q_rel, yaw, pitch, roll);

  out_q.resize(2);
  out_q[0] = pitch;
  out_q[1] = yaw;
  return true;
}

}  // namespace vr_control
}  // namespace leju
