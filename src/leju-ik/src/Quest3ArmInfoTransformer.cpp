#include "leju-ik/Quest3ArmInfoTransformer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

namespace leju {
namespace ik {

namespace {
void setHandPoseInfo(PoseInfo& poseInfo, const ArmPose& armPose) {
  if (armPose.isValid()) {
    poseInfo.position = armPose.position;
    poseInfo.orientation = armPose.quaternion;
  } else {
    poseInfo = PoseInfo();
  }
}

void setElbowPoseInfo(PoseInfo& poseInfo, const ArmPose& armPose) {
  if (armPose.isValid()) {
    poseInfo.position = armPose.position;
    poseInfo.orientation = Eigen::Quaterniond::Identity();
  } else {
    poseInfo = PoseInfo();
  }
}
}  // namespace

Quest3ArmInfoTransformer::Quest3ArmInfoTransformer(const std::string& robotModel,
                                                   const Eigen::Vector3d& deltaScale)
    : qInitChest_(0.5, 0.5, 0.5, 0.5),
      chest_axis_agl_(Eigen::Vector3d::Zero()),
      isInitialized_(true),
      leftHandPose_(robotModel, true),
      rightHandPose_(robotModel, false),
      leftShoulderPose_(),
      rightShoulderPose_(),
      leftShoulderRpyInRobot_(Eigen::Vector3d::Zero()),
      rightShoulderRpyInRobot_(Eigen::Vector3d::Zero()),
      shoulderWidth_(0.15),
      biasChestToBaseLink_(0.0, 0.0, 0.4245),
      armLengthMeasurement_(),
      robotUpperArmLength_(0.2844),
      robotLowerArmLength_(0.45),
      deltaScale_(deltaScale) {
  armLengthMeasurement_.setMeasureArmLength(true);
}

bool Quest3ArmInfoTransformer::updateHandPoseAndElbowPosition(const PoseInfoList& input,
                                                             PoseInfoList& output) {
  if (!validateInput(input)) return false;
  if (!computeHandPose(input, "Left")) return false;
  if (!computeHandPose(input, "Right")) return false;
  updateHandElbowPoseInfoList(output);
  return true;
}

bool Quest3ArmInfoTransformer::computeHandPose(const PoseInfoList& input, const std::string& side) {
  std::map<std::string, ArmData> sideMap = {
      {"Left", ArmData(qLeftHandW_, leftHandPose_, leftElbowPose_, POSE_INDEX_LEFT_ELBOW,
                      POSE_INDEX_LEFT_ARM_UPPER, POSE_INDEX_LEFT_HAND)},
      {"Right", ArmData(qRightHandW_, rightHandPose_, rightElbowPose_, POSE_INDEX_RIGHT_ELBOW,
                       POSE_INDEX_RIGHT_ARM_UPPER, POSE_INDEX_RIGHT_HAND)}};
  auto it = sideMap.find(side);
  if (it == sideMap.end()) return false;

  ArmData& armData = it->second;
  const auto& chestPose = input.poses[POSE_INDEX_CHEST];
  const auto& elbowPose = input.poses[armData.elbowIndex];
  const auto& shoulderPose = input.poses[armData.shoulderIndex];
  const auto& handPose = input.poses[armData.handIndex];

  Eigen::Quaterniond vrQuat(handPose.orientation.w(), handPose.orientation.x(),
                            handPose.orientation.y(), handPose.orientation.z());
  double biasAngle = 15.0 * M_PI / 180.0;
  Eigen::Quaterniond handQuatInW = vrQuat2RobotQuat(vrQuat, side, biasAngle);
  armData.handQuatInW = handQuatInW;

  chestPosition_ = chestPose.position;
  const Eigen::Quaterniond qCurrentChest(chestPose.orientation.w(), chestPose.orientation.x(),
                                        chestPose.orientation.y(), chestPose.orientation.z());
  const Eigen::Quaterniond qRelativeChest = (qInitChest_.inverse() * qCurrentChest).normalized();

  Eigen::Vector3d axis;
  double angle;
  quatToAxisAngle(qRelativeChest, axis, angle);

  Eigen::Matrix3d initRwC = qInitChest_.toRotationMatrix();
  Eigen::Matrix3d currentChestRotation = qCurrentChest.toRotationMatrix();
  Eigen::Matrix3d relativeChestRotation = initRwC.transpose() * currentChestRotation;
  Eigen::Vector3d chestRpy = matrixToRPY(relativeChestRotation);

  chest_axis_agl_ = Eigen::Vector3d(0, 0, axis[1]);
  headBodyPose_.body_yaw = axis[1];

  Eigen::Matrix3d RwChestRmYaw = axisAngleToMatrix(chest_axis_agl_).transpose() * currentChestRotation;
  Eigen::Vector3d bodyRpyAfterYawRemoval = matrixToRPY(initRwC.transpose() * RwChestRmYaw);
  headBodyPose_.body_pitch = bodyRpyAfterYawRemoval[0];
  headBodyPose_.body_roll = chestRpy[2];
  headBodyPose_.body_x = chestPose.position.x();
  headBodyPose_.body_y = chestPose.position.y();
  headBodyPose_.body_height = chestPose.position.z();

  yawOnlyQuat_ = Eigen::Quaterniond(Eigen::AngleAxisd(axis[1], Eigen::Vector3d::UnitZ()));
  handQuatInW = yawOnlyQuat_.inverse() * handQuatInW;

  auto handPos = extractPosition(handPose);
  handPos -= chestPosition_;
  handPos = yawOnlyQuat_.inverse() * handPos;
  handPos += biasChestToBaseLink_;

  auto elbowPos = extractPosition(elbowPose);
  elbowPos -= chestPosition_;
  elbowPos = yawOnlyQuat_.inverse() * elbowPos;
  elbowPos.x() += biasChestToBaseLink_.x();
  elbowPos.z() += biasChestToBaseLink_.z();

  auto shoulderPos = extractPosition(shoulderPose);
  shoulderPos -= chestPosition_;
  shoulderPos = yawOnlyQuat_.inverse() * shoulderPos;
  shoulderPos += biasChestToBaseLink_;

  Eigen::Matrix3d R_wS = extractRotationMatrix(shoulderPose);
  computeShoulderAngle(R_wS, side);

  shoulderPos.x() = biasChestToBaseLink_.x();
  shoulderPos.z() = biasChestToBaseLink_.z();
  Eigen::Vector3d humanShoulderPos = shoulderPos;

  bool overChest = isOverChest(handPos, side);
  adaptShoulderWidthAdvanced(shoulderPos, elbowPos, handPos, humanShoulderPos, side, overChest);

  auto scaledPositions = scaleArmPositions(shoulderPos, elbowPos, handPos, humanShoulderPos, side);
  elbowPos = scaledPositions.first;
  handPos = scaledPositions.second;

  applyLateralPositionAdjustment(handPos, side);

  if (handPos.x() < 0.1) handPos.x() = 0.1;

  armData.handPose = ArmPose(handPos, handQuatInW);
  armData.elbowPose = ArmPose(elbowPos, Eigen::Quaterniond::Identity());

  Eigen::Quaterniond shoulderQuat(R_wS);
  shoulderQuat.normalize();
  if (side == "Left") {
    leftShoulderPose_ = ArmPose(shoulderPos, shoulderQuat);
  } else if (side == "Right") {
    rightShoulderPose_ = ArmPose(shoulderPos, shoulderQuat);
  }

  if (!armData.handPose.isValid() || !armData.elbowPose.isValid()) return false;

  updateVisualizationDataForSide(input, side, handPos, handQuatInW, elbowPos, shoulderPos, R_wS);

  if (side == "Left") {
    leftHandPose_.position = input.poses[armData.handIndex].position;
  }
  if (side == "Right") {
    rightHandPose_.position = input.poses[armData.handIndex].position;
  }

  return true;
}

Eigen::Vector3d Quest3ArmInfoTransformer::extractPosition(const PoseInfo& poseInfo) const {
  return poseInfo.position;
}

void Quest3ArmInfoTransformer::updateHandElbowPoseInfoList(PoseInfoList& output) {
  output.timestamp_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
          .count();
  output.is_high_confidence = true;
  output.is_hand_tracking = false;
  output.poses.resize(4);
  setHandPoseInfo(output.poses[0], leftHandPose_);
  setElbowPoseInfo(output.poses[1], leftElbowPose_);
  setHandPoseInfo(output.poses[2], rightHandPose_);
  setElbowPoseInfo(output.poses[3], rightElbowPose_);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> Quest3ArmInfoTransformer::scaleArmPositions(
    const Eigen::Vector3d& shoulderAdaptivePos, const Eigen::Vector3d& elbowPos,
    const Eigen::Vector3d& handPos, const Eigen::Vector3d& humanShoulderOriginPos,
    const std::string& side) {
  double humanUpperArmLength = (elbowPos - humanShoulderOriginPos).norm();
  double humanLowerArmLength = (handPos - elbowPos).norm();

  if (armLengthMeasurement_.isMeasureArmLength()) {
    armLengthMeasurement_.updateMeasurement(humanUpperArmLength, humanLowerArmLength, side);
    if (armLengthMeasurement_.getLeftDataCount() >= 30 &&
        armLengthMeasurement_.getRightDataCount() >= 30) {
      armLengthMeasurement_.completeMeasurement();
    }
  }

  double radi1, radi2;
  if (armLengthMeasurement_.isMeasureArmLength()) {
    radi1 = robotUpperArmLength_ / humanUpperArmLength;
    radi2 = (robotLowerArmLength_ + robotUpperArmLength_) /
            (humanLowerArmLength + humanUpperArmLength);
  } else {
    if (side == "Left") {
      if (armLengthMeasurement_.getLeftDataCount() > 0) {
        radi1 = robotUpperArmLength_ / armLengthMeasurement_.getAvgLeftUpperArmLength();
        radi2 = robotLowerArmLength_ / armLengthMeasurement_.getAvgLeftLowerArmLength();
      } else {
        radi1 = robotUpperArmLength_ / humanUpperArmLength;
        radi2 = robotLowerArmLength_ / humanLowerArmLength;
      }
    } else {
      if (armLengthMeasurement_.getRightDataCount() > 0) {
        radi1 = robotUpperArmLength_ / armLengthMeasurement_.getAvgRightUpperArmLength();
        radi2 = robotLowerArmLength_ / armLengthMeasurement_.getAvgRightLowerArmLength();
      } else {
        radi1 = robotUpperArmLength_ / humanUpperArmLength;
        radi2 = robotLowerArmLength_ / humanLowerArmLength;
      }
    }
  }

  Eigen::Vector3d scaledElbowPos = shoulderAdaptivePos + radi1 * (elbowPos - humanShoulderOriginPos);
  Eigen::Vector3d scaledHandPos = scaledElbowPos + radi2 * (handPos - elbowPos);
  return {scaledElbowPos, scaledHandPos};
}

bool Quest3ArmInfoTransformer::validateInput(const PoseInfoList& input) const {
  if (input.poses.empty()) return false;
  if (input.poses.size() < 24) return false;

  std::vector<size_t> keyPoseIndices = {POSE_INDEX_LEFT_HAND, POSE_INDEX_LEFT_ELBOW,
                                        POSE_INDEX_RIGHT_HAND, POSE_INDEX_RIGHT_ELBOW};
  for (size_t i : keyPoseIndices) {
    const auto& pose = input.poses[i];
    if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite()) return false;
  }
  return true;
}

Eigen::Quaterniond Quest3ArmInfoTransformer::vrQuat2RobotQuat(const Eigen::Quaterniond& vrQuat,
                                                              const std::string& side,
                                                              double biasAngle) const {
  Eigen::Quaterniond qHand = vrQuat.normalized();
  double xHandBias = side == "Right" ? biasAngle : -biasAngle;
  double xAxisBias = -M_PI / 2;
  double zAxisBias = side == "Right" ? 0.0 : -M_PI;
  Eigen::Quaterniond qZX =
      Eigen::AngleAxisd(zAxisBias, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(xAxisBias, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond qBias(Eigen::AngleAxisd(xHandBias, Eigen::Vector3d::UnitX()));
  return qHand * qZX * qBias;
}

bool Quest3ArmInfoTransformer::isOverChest(const Eigen::Vector3d& handPos,
                                          const std::string& side) const {
  if (side == "Left" && handPos.y() < 0) return true;
  if (side == "Right" && handPos.y() > 0) return true;
  return false;
}

void Quest3ArmInfoTransformer::adaptShoulderWidthAdvanced(
    Eigen::Vector3d& shoulderPos, const Eigen::Vector3d& elbowPos,
    const Eigen::Vector3d& handPos, const Eigen::Vector3d& humanShoulderPos,
    const std::string& side, bool overChest) const {
  double yDistance = std::abs(handPos.y());
  Eigen::Vector3d elbowRelativeToShoulder = elbowPos - humanShoulderPos;
  double elbowAngleHorizontal = std::atan2(elbowRelativeToShoulder.y(), elbowRelativeToShoulder.x());

  double shoulderRotationFactor = 0.0;
  if (side == "Right") {
    if (elbowAngleHorizontal > -M_PI / 2 && elbowAngleHorizontal < M_PI / 4) {
      shoulderRotationFactor = (elbowAngleHorizontal + M_PI / 2) / (3 * M_PI / 4);
    }
  } else {
    if (elbowAngleHorizontal < M_PI / 2 && elbowAngleHorizontal > -M_PI / 4) {
      shoulderRotationFactor = (M_PI / 2 - elbowAngleHorizontal) / (3 * M_PI / 4);
    }
  }
  shoulderRotationFactor = std::clamp(shoulderRotationFactor, 0.0, 1.0);

  double shoulderForwardOffset = shoulderRotationFactor * 0.08;
  double shoulderInwardOffset = shoulderRotationFactor * 0.15;
  double crossBodyFactor = 0.0;
  if (overChest) {
    crossBodyFactor = std::min(yDistance / shoulderWidth_, 1.0) * 0.08;
  }

  shoulderPos.x() += shoulderForwardOffset;
  if (side == "Right") {
    shoulderPos.y() = -shoulderWidth_ + shoulderInwardOffset + crossBodyFactor;
  } else if (side == "Left") {
    shoulderPos.y() = shoulderWidth_ - shoulderInwardOffset - crossBodyFactor;
  }
}

void Quest3ArmInfoTransformer::applyLateralPositionAdjustment(Eigen::Vector3d& handPos,
                                                             const std::string& side) const {
  double handToCenterline = std::abs(handPos.y());
  if (handPos.x() > 0.15 && handToCenterline < 0.2) {
    double pullToCenterFactor = (0.2 - handToCenterline) / 0.2;
    double pullAmount = pullToCenterFactor * 0.05;
    if (side == "Right") {
      handPos.y() = handPos.y() + pullAmount;
    } else {
      handPos.y() = handPos.y() - pullAmount;
    }
  }
}

void Quest3ArmInfoTransformer::updateBaseHeightOffset(double baseHeightOffset) {
  biasChestToBaseLink_.z() = baseHeightOffset;
}
void Quest3ArmInfoTransformer::updateBaseChestOffsetX(double baseChestOffsetX) {
  biasChestToBaseLink_.x() = baseChestOffsetX;
}
void Quest3ArmInfoTransformer::updateShoulderWidth(double shoulderWidth) {
  shoulderWidth_ = shoulderWidth;
}
void Quest3ArmInfoTransformer::updateUpperArmLength(double upperArmLength) {
  robotUpperArmLength_ = upperArmLength;
}
void Quest3ArmInfoTransformer::updateLowerArmLength(double lowerArmLength) {
  robotLowerArmLength_ = lowerArmLength;
}

void Quest3ArmInfoTransformer::computeShoulderAngle(const Eigen::Matrix3d& R_wS,
                                                     const std::string& side) {
  static Eigen::Matrix3d initR_wLS = Eigen::Matrix3d::Identity();
  static Eigen::Matrix3d initR_wRS = Eigen::Matrix3d::Identity();
  static bool leftInitialized = false;
  static bool rightInitialized = false;

  if (side == "Left" && !leftInitialized) {
    initR_wLS = R_wS;
    leftInitialized = true;
    return;
  }
  if (side == "Right" && !rightInitialized) {
    initR_wRS = R_wS;
    rightInitialized = true;
    return;
  }

  Eigen::Matrix3d R_01;
  Eigen::Vector3d rpy;

  if (side == "Left") {
    R_01 = initR_wLS.transpose() * R_wS;
    rpy = matrixToRPY(R_01);
    leftShoulderRpyInRobot_[0] = rpy[2];
    leftShoulderRpyInRobot_[1] = -rpy[0];
    leftShoulderRpyInRobot_[2] = -rpy[1];
  } else if (side == "Right") {
    R_01 = initR_wRS.transpose() * R_wS;
    rpy = matrixToRPY(R_01);
    rightShoulderRpyInRobot_[0] = -rpy[2];
    rightShoulderRpyInRobot_[1] = -rpy[0];
    rightShoulderRpyInRobot_[2] = rpy[1];
  }
}

Eigen::Matrix3d Quest3ArmInfoTransformer::extractRotationMatrix(const PoseInfo& poseInfo) const {
  return poseInfo.orientation.toRotationMatrix();
}

void Quest3ArmInfoTransformer::quatToAxisAngle(const Eigen::Quaterniond& quat,
                                               Eigen::Vector3d& axis, double& angle) const {
  double x = quat.x(), y = quat.y(), z = quat.z(), w = quat.w();
  double norm_xyz = std::sqrt(x * x + y * y + z * z);
  angle = 2.0 * std::atan2(norm_xyz, w);
  if (angle <= 1e-3) {
    double angle2 = angle * angle;
    double scale = 2.0 + angle2 / 12.0 + 7.0 * angle2 * angle2 / 2880.0;
    axis = Eigen::Vector3d(x, y, z) * scale;
  } else {
    double scale = angle / std::sin(angle / 2.0);
    axis = Eigen::Vector3d(x, y, z) * scale;
  }
}

Eigen::Vector3d Quest3ArmInfoTransformer::matrixToRPY(const Eigen::Matrix3d& R) const {
  double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
  bool singular = sy < 1e-6;
  double roll, pitch, yaw;
  if (!singular) {
    roll = std::atan2(R(2, 1), R(2, 2));
    pitch = std::atan2(-R(2, 0), sy);
    yaw = std::atan2(R(1, 0), R(0, 0));
  } else {
    roll = std::atan2(-R(1, 2), R(1, 1));
    pitch = std::atan2(-R(2, 0), sy);
    yaw = 0;
  }
  return Eigen::Vector3d(roll, pitch, yaw);
}

void Quest3ArmInfoTransformer::updateVisualizationDataForSide(
    const PoseInfoList& input, const std::string& side, const Eigen::Vector3d& handPos,
    const Eigen::Quaterniond& handQuat, const Eigen::Vector3d& elbowPos,
    const Eigen::Vector3d& shoulderPos, const Eigen::Matrix3d& shoulderRot) {
  if (side == "Left") {
    visualizationData_.leftHandPos = handPos;
    visualizationData_.leftElbowPos = elbowPos;
    visualizationData_.leftShoulderPos = shoulderPos;
    visualizationData_.leftHandQuat = qLeftHandW_;
    visualizationData_.leftSideReady = true;
  } else if (side == "Right") {
    visualizationData_.rightHandPos = handPos;
    visualizationData_.rightElbowPos = elbowPos;
    visualizationData_.rightShoulderPos = shoulderPos;
    visualizationData_.rightHandQuat = qRightHandW_;
    visualizationData_.rightSideReady = true;
  }

  Eigen::Vector3d originalChestPos;
  Eigen::Matrix3d chestRot = Eigen::Matrix3d::Identity();
  if (input.poses.size() > POSE_INDEX_CHEST) {
    const auto& chestPose = input.poses[POSE_INDEX_CHEST];
    originalChestPos = chestPose.position;
    visualizationData_.chestPos = originalChestPos;
    chestRot = chestPose.orientation.toRotationMatrix();
  }

  visualizationData_.isValid = visualizationData_.leftSideReady || visualizationData_.rightSideReady;
  if (visPub_ && visualizationCallback_) {
    std::vector<PoseData> poses;
    poses.push_back(PoseData(handQuat.toRotationMatrix(), handPos));
    poses.push_back(PoseData(Eigen::Matrix3d::Identity(), elbowPos));
    poses.push_back(PoseData(shoulderRot, shoulderPos));
    poses.push_back(PoseData(chestRot, originalChestPos));
    visualizationCallback_(side, poses);
  }
}

void Quest3ArmInfoTransformer::updateJoystickData(float leftTrigger, float leftGrip,
                                                 float rightTrigger, float rightGrip) {
  leftJoystick_.trigger = leftTrigger;
  leftJoystick_.grip = leftGrip;
  rightJoystick_.trigger = rightTrigger;
  rightJoystick_.grip = rightGrip;
}

Eigen::Matrix3d Quest3ArmInfoTransformer::axisAngleToMatrix(const Eigen::Vector3d& axisAngle) const {
  double angle = axisAngle.norm();
  if (angle < 1e-8) return Eigen::Matrix3d::Identity();
  Eigen::Vector3d axis = axisAngle / angle;
  return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

}  // namespace ik
}  // namespace leju
