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
    Eigen::Vector3d leftHandPos, rightHandPos, leftElbowPos, rightElbowPos;
    Eigen::Vector3d leftShoulderPos, rightShoulderPos, chestPos;
    Eigen::Quaterniond leftHandQuat, rightHandQuat;
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

 private:
  ArmPose leftHandPose_, rightHandPose_, leftElbowPose_, rightElbowPose_;
  ArmPose leftShoulderPose_, rightShoulderPose_;
  Eigen::Quaterniond qLeftHandW_, qRightHandW_;
  Eigen::Quaterniond qInitChest_;
  Eigen::Vector3d chest_axis_agl_;
  bool isInitialized_;
  Eigen::Quaterniond yawOnlyQuat_;
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
  Eigen::Vector3d extractPosition(const PoseInfo& poseInfo) const;
  Eigen::Matrix3d poseInfo2Transform(const PoseInfo& poseInfo) const;
  std::pair<Eigen::Vector3d, Eigen::Vector3d> scaleArmPositions(
      const Eigen::Vector3d& shoulderAdaptivePos, const Eigen::Vector3d& elbowPos,
      const Eigen::Vector3d& handPos, const Eigen::Vector3d& humanShoulderOriginPos,
      const std::string& side);
  bool validateInput(const PoseInfoList& input) const;
  Eigen::Quaterniond vrQuat2RobotQuat(const Eigen::Quaterniond& vrQuat, const std::string& side,
                                      double biasAngle = 0.0) const;
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

};

}  // namespace ik
}  // namespace leju
