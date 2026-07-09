/**
 * @file quest_to_ik_converter.cpp
 * @brief Convert Quest data to leju-ik types
 */

#include "leju-vr-control/quest_to_ik_converter.h"

namespace leju {
namespace vr_control {

leju::ik::PoseInfoList toPoseInfoList(const QuestBonePosesData& quest_poses) {
  leju::ik::PoseInfoList out;
  out.timestamp_ms = quest_poses.timestamp_ms;
  out.is_high_confidence = quest_poses.is_high_confidence;
  out.is_hand_tracking = quest_poses.is_hand_tracking;

  const auto& poses = quest_poses.poses;
  out.poses.resize(poses.size());

  for (size_t i = 0; i < poses.size(); ++i) {
    const auto& p = poses[i];
    out.poses[i].position << static_cast<double>(p.x),
        static_cast<double>(p.y),
        static_cast<double>(p.z);
    // Eigen::Quaterniond(w, x, y, z)
    out.poses[i].orientation = Eigen::Quaterniond(
        static_cast<double>(p.qw),
        static_cast<double>(p.qx),
        static_cast<double>(p.qy),
        static_cast<double>(p.qz));
  }

  return out;
}

leju::ik::JoyStickData toJoyStickData(const QuestJoystickData& quest_joy) {
  leju::ik::JoyStickData out;
  out.left_x = quest_joy.left_x;
  out.left_y = quest_joy.left_y;
  out.left_trigger = quest_joy.left_trigger;
  out.left_grip = quest_joy.left_grip;
  out.right_x = quest_joy.right_x;
  out.right_y = quest_joy.right_y;
  out.right_trigger = quest_joy.right_trigger;
  out.right_grip = quest_joy.right_grip;
  return out;
}

}  // namespace vr_control
}  // namespace leju
