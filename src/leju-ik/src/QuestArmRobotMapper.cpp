#include "leju-ik/QuestArmRobotMapper.h"
#include <iostream>
namespace leju {
namespace ik {

QuestArmRobotMapperConfig QuestArmRobotMapperConfig::fromQuestVrCalibration(
    const QuestVrCalibration& cal) {
  QuestArmRobotMapperConfig c = bipedArmDefault();
  c.upper_arm_length = cal.upper_arm_length;
  c.lower_arm_length = cal.lower_arm_length;
  c.shoulder_width = cal.shoulder_width;
  c.base_height_offset = cal.base_height_offset;
  c.base_chest_offset_x = cal.base_chest_offset_x;
  c.eef_z_offset = cal.eef_z_bias;
  return c;
}

QuestArmRobotMapperConfig QuestArmRobotMapperConfig::bipedArmDefault() {
  QuestArmRobotMapperConfig c;
  c.shoulder_frame_left = "zarm_l1_link";
  c.shoulder_frame_right = "zarm_r1_link";
  c.eef_visual_stl_left = "l_hand_roll.STL";
  c.eef_visual_stl_right = "r_hand_roll.STL";
  c.upper_arm_length = 0.20533;
  c.lower_arm_length = 0.2175;
  c.shoulder_width = 0.181;
  c.eef_z_offset = -0.2175;
  c.hand_ref_length = 0.0;
  c.base_height_offset = 0.24;
  c.base_chest_offset_x = 0.0;
  return c;
}

QuestArmRobotMapper::QuestArmRobotMapper(const QuestArmRobotMapperConfig& cfg)
    : cfg_(cfg), transformer_("kuavo_45") {
  transformer_.updateUpperArmLength(cfg_.upper_arm_length);
  transformer_.updateLowerArmLength(cfg_.lower_arm_length);
  transformer_.updateShoulderWidth(cfg_.shoulder_width);
  transformer_.updateBaseHeightOffset(cfg_.base_height_offset);
  transformer_.updateBaseChestOffsetX(cfg_.base_chest_offset_x);
  // 绝对式工作流靠摇杆手势（OK/Shot）解锁与停止，需要 transformer 自动驱动 isRunning_。
  transformer_.setGestureRunningEnabled(true);
}

bool QuestArmRobotMapper::mapBonesToRobotTargets(const PoseInfoList& bones, RobotArmIkTargets& out) {
  out.valid = false;
  PoseInfoList scratch;
  if (!transformer_.updateHandPoseAndElbowPosition(bones, scratch)) {
    return false;
  }

  const auto& lh = transformer_.getLeftHandPose();
  const auto& rh = transformer_.getRightHandPose();
  const auto& le = transformer_.getLeftElbowPose();
  const auto& re = transformer_.getRightElbowPose();

  if (!lh.isValid() || !rh.isValid() || !le.isValid() || !re.isValid()) {
    return false;
  }

  out.left_hand = lh.position;
  out.right_hand = rh.position;
  out.left_elbow = le.position;
  out.right_elbow = re.position;
  out.left_hand_quat = lh.quaternion;
  out.right_hand_quat = rh.quaternion;
  out.valid = true;
  return true;
}

}  // namespace ik
}  // namespace leju
