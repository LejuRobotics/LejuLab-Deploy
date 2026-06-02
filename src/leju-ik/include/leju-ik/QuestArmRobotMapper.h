/**
 * @file QuestArmRobotMapper.h
 * @brief 将 Quest3 骨骼 PoseInfoList 映射为机器人 IK 用手/肘目标（封装 Quest3ArmInfoTransformer + 固定标定）
 *
 * 逻辑与 motion_capture_ik/scripts/tools/quest3_utils.py::Quest3ArmInfoTransformer 一致；
 * 下列参数可按机型硬编码，默认值为 biped 单臂链长度与胸→基座偏置（与 URDF 段长一致时常用组合）。
 */

#pragma once

#include "leju-ik/Quest3ArmInfoTransformer.h"
#include "leju-ik/QuestVrCalibration.h"
#include "leju-ik/ik_types.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

namespace leju {
namespace ik {

/// 与 kuavo.json 中 VR/IK 相关字段对应（用于文档与其它模块读名）
struct QuestArmRobotMapperConfig {
  std::string shoulder_frame_left = "zarm_l1_link";
  std::string shoulder_frame_right = "zarm_r1_link";
  std::string eef_visual_stl_left = "l_hand_roll.STL";
  std::string eef_visual_stl_right = "r_hand_roll.STL";

  double upper_arm_length = 0.20533;
  double lower_arm_length = 0.2175;
  double shoulder_width = 0.181;
  /// 传给 ArmAbsoluteIKConfig::eef_z_bias（末端在名义 EEF 上的额外 z 平移，米）
  double eef_z_offset = -0.2175;
  /// 与 Python 两阶段 IK 中 hand_ref_length 对应；本映射器仅保存，不参与当前单阶段映射
  double hand_ref_length = 0.0;
  double base_height_offset = 0.24;
  double base_chest_offset_x = 0.0;

  /// 当前工程约定的 biped 默认标定（可整体替换为其它机型）
  static QuestArmRobotMapperConfig bipedArmDefault();

  /// 用 ROBOT_VERSION 对应的 QuestVrCalibration 覆盖臂长/肩宽/胸偏置/eef_z，肩架与 STL 名保留 biped 默认
  static QuestArmRobotMapperConfig fromQuestVrCalibration(const QuestVrCalibration& cal);
};

/// 躯干/世界系下、供 ArmAbsoluteIK::computeIK 使用的目标（位置 + 手部四元数可选）
struct RobotArmIkTargets {
  bool valid = false;
  Eigen::Vector3d left_hand = Eigen::Vector3d::Zero();
  Eigen::Vector3d right_hand = Eigen::Vector3d::Zero();
  Eigen::Vector3d left_elbow = Eigen::Vector3d::Zero();
  Eigen::Vector3d right_elbow = Eigen::Vector3d::Zero();
  Eigen::Quaterniond left_hand_quat = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond right_hand_quat = Eigen::Quaterniond::Identity();
};

/**
 * @brief Quest 骨骼 → 机器人手/肘 IK 输入的薄封装
 *
 * 内部使用 Quest3ArmInfoTransformer；构造时写入 upper/lower/shoulder/base/eef 等标定。
 * shoulder_frame_* / eef_visual_stl_* 仅保存在 config() 中供上层（可视化、肩位 FK 裁剪等）使用。
 */
class QuestArmRobotMapper final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit QuestArmRobotMapper(const QuestArmRobotMapperConfig& cfg = QuestArmRobotMapperConfig::bipedArmDefault());

  const QuestArmRobotMapperConfig& config() const { return cfg_; }

  /// 与 ArmAbsoluteIK 的 eef_z_bias 一致
  double eefZBiasForAbsoluteIk() const { return cfg_.eef_z_offset; }

  Quest3ArmInfoTransformer& transformer() { return transformer_; }
  const Quest3ArmInfoTransformer& transformer() const { return transformer_; }

  /**
   * @param bones 与 quest_udp_to_dds → toPoseInfoList 相同约定的 PoseInfoList（≥24 骨骼）
   * @param out   手/肘位置与姿态；胸减+yaw 去除+bias 后肩位固定（y=±肩宽），肘/手由
   *              quest3_utils.scale_arm_positions 同款比例映射：scaled_elbow = 肩 + radi1*(肘-人体肩)，
   *              scaled_hand = scaled_elbow + radi2*(手-肘)。
   * @return 是否与 Quest3ArmInfoTransformer::updateHandPoseAndElbowPosition 同样成功
   */
  bool mapBonesToRobotTargets(const PoseInfoList& bones, RobotArmIkTargets& out);

  void setRunning(bool running) { transformer_.setRunning(running); }
  bool isRunning() const { return transformer_.isRunning(); }
  void updateJoystickForGesture(float left_trigger, float left_grip, float right_trigger,
                                float right_grip) {
    transformer_.updateJoystickData(left_trigger, left_grip, right_trigger, right_grip);
  }

 private:
  QuestArmRobotMapperConfig cfg_;
  Quest3ArmInfoTransformer transformer_;
};

}  // namespace ik
}  // namespace leju
