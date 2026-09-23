/**
 * @file QuestVrCalibration.h
 * @brief Quest VR + ArmAbsoluteIK 几何标定（按 ROBOT_VERSION 硬编码，与 ik_ros_uni + kuavo.json 对齐）
 */

#pragma once

#include <string>

namespace leju {
namespace ik {

/// 与 motion_capture_ik ik_ros_uni.py + kuavo.json 一致的 Quest/IK 标定
struct QuestVrCalibration {
  double upper_arm_length{};
  double lower_arm_length{};
  double shoulder_width{};
  double base_height_offset{};
  double base_chest_offset_x{};
  double eef_z_bias{};
};

/// @param ver 与目录名 biped_s{ver} 一致，如 "17"、"14"
inline QuestVrCalibration calibrationForRobotVersion(const std::string& ver) {
  QuestVrCalibration c;
  if (ver == "17" || ver == "14") {
    // leju-hardware/config/roban_v14/kuavo.json（Roban / biped_s17）
    c.upper_arm_length = 0.188;
    c.lower_arm_length = 0.17;
    c.shoulder_width = 0.16;
    c.base_height_offset = 0.18;
    c.base_chest_offset_x = 0.0;
    c.eef_z_bias = -0.17;
    return c;
  }
  // 其它版本：原 quest 节点默认 + Quest3ArmInfoTransformer 胸高默认
  c.upper_arm_length = 0.29;
  c.lower_arm_length = 0.29;
  c.shoulder_width = 0.15;
  c.base_height_offset = 0.4245;
  c.base_chest_offset_x = 0.0;
  c.eef_z_bias = 0.0;
  return c;
}

}  // namespace ik
}  // namespace leju
