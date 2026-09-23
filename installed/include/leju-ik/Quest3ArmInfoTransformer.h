#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <functional>
#include <string>
#include <vector>

#include "leju-ik/ArmLengthMeasurement.h"
#include "leju-ik/ik_types.h"

namespace leju {
namespace ik {

class Quest3ArmInfoTransformer final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Quest3ArmInfoTransformer(const std::string& robotModel = "kuavo_45",
                           const Eigen::Vector3d& deltaScale = Eigen::Vector3d(1.0, 1.0, 1.0));

  ~Quest3ArmInfoTransformer() = default;

  bool updateHandPoseAndElbowPosition(const PoseInfoList& input, PoseInfoList& output);

  const ArmPose& getLeftHandPose() const { return leftHandPose_; }
  const ArmPose& getRightHandPose() const { return rightHandPose_; }
  const ArmPose& getLeftElbowPose() const { return leftElbowPose_; }
  const ArmPose& getRightElbowPose() const { return rightElbowPose_; }
  const ArmPose& getLeftShoulderPose() const { return leftShoulderPose_; }
  const ArmPose& getRightShoulderPose() const { return rightShoulderPose_; }
  const HeadBodyPose& getHeadBodyPose() const { return headBodyPose_; }

  void updateHandElbowPoseInfoList(PoseInfoList& output);

  void updateBaseHeightOffset(double baseHeightOffset);
  void updateBaseChestOffsetX(double baseChestOffsetX);
  void updateShoulderWidth(double shoulderWidth);
  void updateUpperArmLength(double upperArmLength);
  void updateLowerArmLength(double lowerArmLength);

  void setMeasureArmLength(bool measureArmLength) { armLengthMeasurement_.setMeasureArmLength(measureArmLength); }
  bool isMeasureArmLength() const { return armLengthMeasurement_.isMeasureArmLength(); }
  void resetArmLengthMeasurement() { armLengthMeasurement_.reset(); }
  void completeArmLengthMeasurement() { armLengthMeasurement_.completeMeasurement(); }
  void setDeltaScale(const Eigen::Vector3d& deltaScale) { deltaScale_ = deltaScale; }

  double getAvgLeftUpperArmLength() const { return armLengthMeasurement_.getAvgLeftUpperArmLength(); }
  double getAvgLeftLowerArmLength() const { return armLengthMeasurement_.getAvgLeftLowerArmLength(); }
  double getAvgRightUpperArmLength() const { return armLengthMeasurement_.getAvgRightUpperArmLength(); }
  double getAvgRightLowerArmLength() const { return armLengthMeasurement_.getAvgRightLowerArmLength(); }

  struct VisualizationData {
    /// 缩放后（IK 输入，躯干/ base_link 约定系）
    Eigen::Vector3d leftHandPos, rightHandPos, leftElbowPos, rightElbowPos;
    Eigen::Vector3d leftShoulderPos, rightShoulderPos, chestPos;
    Eigen::Quaterniond leftHandQuat, rightHandQuat;
    /// 胸减+yaw 去除+bias+肩宽后、缩放前（与 IK 目标同躯干系）
    Eigen::Vector3d leftHandPreScale, rightHandPreScale, leftElbowPreScale, rightElbowPreScale;
    bool leftSideReady = false, rightSideReady = false, isValid = false;
  };

  const VisualizationData& getVisualizationData() const { return visualizationData_; }
  bool hasVisualizationData() const { return visualizationData_.isValid; }

  using VisualizationCallback =
      std::function<void(const std::string& side, const std::vector<PoseData>& poses)>;
  void setVisualizationCallback(VisualizationCallback callback) { visualizationCallback_ = callback; }

  bool isRunning() const { return isRunning_; }
  void setRunning(bool running) { isRunning_ = running; }
  void updateJoystickData(float leftTrigger, float leftGrip, float rightTrigger, float rightGrip);
  void resetInitChestYaw();
 private:
  ArmPose leftHandPose_, rightHandPose_, leftElbowPose_, rightElbowPose_;
  ArmPose leftShoulderPose_, rightShoulderPose_;
  Eigen::Quaterniond qLeftHandW_, qRightHandW_;
  Eigen::Quaterniond qInitChest_;
  Eigen::Vector3d chest_axis_agl_;
  bool isInitialized_;
  Eigen::Quaterniond yawOnlyQuat_;
  double bodyYaw_ = std::numeric_limits<double>::quiet_NaN();  // 当前帧的 body yaw（肩部估算），NaN=回退到 chest
  double initialShoulderYaw_ = 0.0;                            // 初始肩部方向角
  bool shoulderYawInitialized_ = false;                         // 是否已记录初始朝向
  bool firstCompute_ = true;  // 是否是第一次计算身体 yaw（肩部估算）
  double bodyYawFilter = 0.0;  // 低通滤波后的 body yaw（肩部估算），NaN=回退到 chest
  Eigen::Vector3d chestPosition_;
  double bodyPitch_;
  Eigen::Vector3d leftShoulderRpyInRobot_, rightShoulderRpyInRobot_;
  double shoulderWidth_;
  Eigen::Vector3d biasChestToBaseLink_;
  ArmLengthMeasurement armLengthMeasurement_;
  double robotUpperArmLength_, robotLowerArmLength_;
  VisualizationData visualizationData_;
  VisualizationCallback visualizationCallback_;
  bool visPub_ = true;
  bool isRunning_ = false;
  Eigen::Vector3d deltaScale_;

  struct JoystickState {
    float trigger = 0.0f, grip = 0.0f;
  };
  JoystickState leftJoystick_, rightJoystick_;
  HeadBodyPose headBodyPose_;

  bool computeHandPose(const PoseInfoList& input, const std::string& side);
  /// 用左右肩部骨骼位置（index 0, 2）估算身体 yaw；返回 NaN 表示数据不可用
  double computeBodyYawFromShoulders(const PoseInfoList& input);
  Eigen::Vector3d extractPosition(const PoseInfo& poseInfo) const;
  Eigen::Matrix3d poseInfo2Transform(const PoseInfo& poseInfo) const;
  /// 与 quest3_utils.scale_arm_positions 一致：固定肩位，上臂/前臂按机器人与人体段长比例缩放
  std::pair<Eigen::Vector3d, Eigen::Vector3d> scaleArmPositions(
      const Eigen::Vector3d& shoulderAdaptivePos, const Eigen::Vector3d& elbowPos,
      const Eigen::Vector3d& handPos, const Eigen::Vector3d& humanShoulderOriginPos,
      const std::string& side);
  bool validateInput(const PoseInfoList& input) const;
  Eigen::Quaterniond vrQuat2RobotQuat(const Eigen::Quaterniond& vrQuat, const std::string& side,
                                      double biasAngle = 0.0) const;
  /// 与 motion_capture_ik / quest3_utils：肩宽自适应（含越过身体中线时的内收等）
  bool isOverChest(const Eigen::Vector3d& handPos, const std::string& side) const;
  void adaptShoulderWidthAdvanced(Eigen::Vector3d& shoulderPos, const Eigen::Vector3d& elbowPos,
                                  const Eigen::Vector3d& handPos,
                                  const Eigen::Vector3d& humanShoulderPos, const std::string& side,
                                  bool overChest) const;
  void applyLateralPositionAdjustment(Eigen::Vector3d& handPos, const std::string& side) const;
  void computeShoulderAngle(const Eigen::Matrix3d& R_wS, const std::string& side);
  Eigen::Matrix3d extractRotationMatrix(const PoseInfo& poseInfo) const;
  void quatToAxisAngle(const Eigen::Quaterniond& quat, Eigen::Vector3d& axis, double& angle) const;
  Eigen::Vector3d matrixToRPY(const Eigen::Matrix3d& R) const;
  Eigen::Matrix3d axisAngleToMatrix(const Eigen::Vector3d& axisAngle) const;
  void updateVisualizationDataForSide(const PoseInfoList& input, const std::string& side,
                                      const Eigen::Vector3d& handPos,
                                      const Eigen::Quaterniond& handQuat,
                                      const Eigen::Vector3d& elbowPos,
                                      const Eigen::Vector3d& shoulderPos,
                                      const Eigen::Matrix3d& shoulderRot);

  /// 与 motion_capture_ik/scripts/tools/quest3_utils.py 中手柄分支一致：
  /// OK：双手扳机均 > 0.5，连续约 50 帧；停止(Shot)：双手扳机 < 0.5 且握把 > 0.8，连续约 50 帧。
  void updateRunningGestureState();

  int ok_gesture_counts_ = 0;
  int shot_gesture_counts_ = 0;
};

}  // namespace ik
}  // namespace leju
