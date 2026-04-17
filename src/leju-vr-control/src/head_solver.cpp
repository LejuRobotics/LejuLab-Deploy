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

// Quest bone poses here behave like a Y-up frame:
// - yaw   : rotation around Y
// - pitch : rotation around X
// We therefore extract Y-X-Z Tait-Bryan angles instead of the previous Z-Y-X
// decomposition, which caused left/right head yaw to appear on the "pitch" term.
void quatToEulerYXZ(const Eigen::Quaterniond& q, double& yaw, double& pitch, double& roll) {
  Eigen::Matrix3d R = q.toRotationMatrix();
  pitch = std::asin(-R(1, 2));
  if (std::abs(std::cos(pitch)) > 1e-6) {
    yaw = std::atan2(R(0, 2), R(2, 2));
    roll = std::atan2(R(1, 0), R(1, 1));
  } else {
    yaw = std::atan2(-R(2, 0), R(0, 0));
    roll = 0.0;
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
  quatToEulerYXZ(q_rel, yaw, pitch, roll);

  out_q.resize(2);
  out_q[0] = yaw;
  out_q[1] = pitch;
  return true;
}

}  // namespace vr_control
}  // namespace leju
