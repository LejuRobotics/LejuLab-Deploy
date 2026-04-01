#include "leju-ik/IncrementalControlModule.h"

#include <cmath>

namespace leju {
namespace ik {

// ============ HandStatus ============
void HandStatus::ready(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  activated = true;
  moving = false;
  lastPosition = position;
  lastOrientation = orientation;
  isFirstFrame = true;
}

void HandStatus::unready(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  activated = false;
  moving = false;
  lastPosition = position;
  lastOrientation = orientation;
  isFirstFrame = true;
}

bool HandStatus::detectMovement(const Eigen::Vector3d& currentPosition, double threshold) {
  if (moving) return true;
  if (isFirstFrame) {
    lastPosition = currentPosition;
    isFirstFrame = false;
    return false;
  }
  lastMovementDistance = (currentPosition - lastPosition).norm();
  lastPosition = currentPosition;
  if (lastMovementDistance > threshold) {
    moving = true;
    return true;
  }
  return false;
}

void HandStatus::reset() {
  moving = false;
  isFirstFrame = true;
  lastPosition.setZero();
  lastOrientation = Eigen::Quaterniond::Identity();
  lastMovementDistance = 0.0;
}

// ============ IncrementalPoseResult ============
void IncrementalPoseResult::saveLastOnExit(const std::vector<PoseData>& latestPoseConstraintList) {
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    robotLeftHandPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
    robotLeftHandQuatAnchor_ =
        Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix).normalized();
    robotLeftHandQuatTarget_ = robotLeftHandQuatAnchor_;
  }
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    robotRightHandPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
    robotRightHandQuatAnchor_ =
        Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix).normalized();
    robotRightHandQuatTarget_ = robotRightHandQuatAnchor_;
  }
}

void IncrementalPoseResult::resetDelta() {
  robotLeftHandDeltaPos_.setZero();
  robotRightHandDeltaPos_.setZero();
  dotLeftHandDeltaPos_.setZero();
  dotRightHandDeltaPos_.setZero();
  leftHandDeltaQuatLast_.setIdentity();
  rightHandDeltaQuatLast_.setIdentity();
}

void IncrementalPoseResult::resetLeftHandDelta() {
  robotLeftHandDeltaPos_.setZero();
  dotLeftHandDeltaPos_.setZero();
  leftSlerpQuatT_ = 0.0;
  leftSlerpQuatDt_ = 0.0;
  leftHandDeltaQuatLast_.setIdentity();
}

void IncrementalPoseResult::resetRightHandDelta() {
  robotRightHandDeltaPos_.setZero();
  dotRightHandDeltaPos_.setZero();
  rightSlerpQuatT_ = 0.0;
  rightSlerpQuatDt_ = 0.0;
  rightHandDeltaQuatLast_.setIdentity();
}

void IncrementalPoseResult::resetSlerpFactor() {
  leftSlerpQuatT_ = 0.0;
  leftSlerpQuatDt_ = 0.0;
  rightSlerpQuatT_ = 0.0;
  rightSlerpQuatDt_ = 0.0;
}

void IncrementalPoseResult::slerpQuat(const Eigen::Quaterniond& leftHandQuat,
                                      const Eigen::Quaterniond& rightHandQuat,
                                      bool isLeftActive,
                                      bool isRightActive) {
  if (isLeftActive) {
    robotLeftHandQuatSlerpDes_ = robotLeftHandQuatAnchor_.slerp(leftSlerpQuatT_, leftHandQuat).normalized();
    if (!usePythonIncrementalOrientation_) robotLeftHandQuatTarget_ = robotLeftHandQuatSlerpDes_;
  }
  if (isRightActive) {
    robotRightHandQuatSlerpDes_ = robotRightHandQuatAnchor_.slerp(rightSlerpQuatT_, rightHandQuat).normalized();
    if (!usePythonIncrementalOrientation_) robotRightHandQuatTarget_ = robotRightHandQuatSlerpDes_;
  }
}

std::pair<Eigen::Quaterniond, Eigen::Quaterniond> IncrementalPoseResult::getLatestRobotLeftHandQuatInc(bool /*smoothRotation*/) const {
  Eigen::Quaterniond qRobotTarget = robotLeftHandQuatTarget_;
  qRobotTarget = robotLeftHandQuatMeasEERealTime_.conjugate() * qRobotTarget;
  qRobotTarget = robotLeftHandQuatMeasEERealTime_ * limitQuaternionAngleEulerZYX(qRobotTarget, zyxLimitsFinal_);
  return {qRobotTarget, leftHandDeltaQuatLast_};
}

std::pair<Eigen::Quaterniond, Eigen::Quaterniond> IncrementalPoseResult::getLatestRobotRightHandQuatInc(bool /*smoothRotation*/) const {
  Eigen::Quaterniond qRobotTarget = robotRightHandQuatTarget_;
  qRobotTarget = robotRightHandQuatMeasEERealTime_.conjugate() * qRobotTarget;
  qRobotTarget = robotRightHandQuatMeasEERealTime_ * limitQuaternionAngleEulerZYX(qRobotTarget, zyxLimitsFinal_);
  return {qRobotTarget, rightHandDeltaQuatLast_};
}

std::tuple<Eigen::Quaterniond, Eigen::Quaterniond, Eigen::Vector3d, Eigen::Vector3d>
IncrementalPoseResult::getLatestIncrementalHandPose(bool /*incrementalPos*/,
                                                    bool incrementalQuat,
                                                    bool smoothRotation) const {
  Eigen::Vector3d leftPos = robotLeftHandPosAnchor_ + robotLeftHandDeltaPos_;
  Eigen::Vector3d rightPos = robotRightHandPosAnchor_ + robotRightHandDeltaPos_;
  if (incrementalQuat) {
    return std::make_tuple(getLatestRobotLeftHandQuatInc(smoothRotation).first,
                           getLatestRobotRightHandQuatInc(smoothRotation).first, leftPos, rightPos);
  }
  return std::make_tuple(robotLeftHandQuatSlerpDes_, robotRightHandQuatSlerpDes_, leftPos, rightPos);
}

Eigen::Vector3d IncrementalPoseResult::getLeftAnchorPos() const { return humanLeftHandPosAnchor_; }
Eigen::Vector3d IncrementalPoseResult::getRightAnchorPos() const { return humanRightHandPosAnchor_; }
Eigen::Vector3d IncrementalPoseResult::getRobotLeftHandAnchorPos() const { return robotLeftHandPosAnchor_; }
Eigen::Vector3d IncrementalPoseResult::getRobotRightHandAnchorPos() const { return robotRightHandPosAnchor_; }
Eigen::Quaterniond IncrementalPoseResult::getRobotLeftHandAnchorQuat() const { return robotLeftHandQuatAnchor_; }
Eigen::Quaterniond IncrementalPoseResult::getRobotRightHandAnchorQuat() const { return robotRightHandQuatAnchor_; }

// ============ IncrementalControlModule ============
IncrementalControlModule::IncrementalControlModule(const IncrementalControlConfig& config)
    : config_(config),
      initialized_(true),
      zyxLimitsEE_(config.zyxLimitsEE),
      zyxLimitsLink4_(config.zyxLimitsLink4),
      defaultLeftHandPos_(0.05, 0.32, -0.05),
      defaultRightHandPos_(0.05, -0.32, -0.05),
      posAnchorZeroThreshold_(1e-3),
      slerpQuatFactorThreshold_(1e-6) {
  result_.usePythonIncrementalOrientation_ = config_.usePythonIncrementalOrientation;
  result_.pythonOrientationThresholdRad_ = config_.pythonOrientationThresholdRad;
  result_.zyxLimitsFinal_ = config_.zyxLimitsFinal;
}

void IncrementalControlModule::updateLeftArmPoseAnchor(const ArmPose& vrLeftPose,
                                                       const std::vector<PoseData>& latestPoseConstraintList,
                                                       const Eigen::Vector3d& /*pEndEffector*/,
                                                       const Eigen::Quaterniond& qEndEffector,
                                                       const Eigen::Quaterniond& qLink4) {
  result_.humanLeftHandPosAnchor_ = vrLeftPose.position;
  result_.humanLeftHandQuatAnchor_ = vrLeftPose.quaternion.normalized();
  result_.humanLeftHandQuatMeas_ = vrLeftPose.quaternion.normalized();
  result_.humanLeftHandQuatAnchorPython_ = result_.humanLeftHandQuatAnchor_;

  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    Eigen::Quaterniond qTargetQuatAnchor =
        Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix).normalized();
    Eigen::Quaterniond qTargetQuatAnchorRelEE = qEndEffector.conjugate() * qTargetQuatAnchor;
    qTargetQuatAnchorRelEE = limitQuaternionAngleEulerZYX(qTargetQuatAnchorRelEE, zyxLimitsEE_);
    Eigen::Quaterniond qTargetQuatAnchorLimited = qEndEffector * qTargetQuatAnchorRelEE;
    Eigen::Quaterniond qTargetQuatAnchorRelLink4 = qLink4.conjugate() * qTargetQuatAnchorLimited;
    qTargetQuatAnchorRelLink4 = limitQuaternionAngleEulerZYX(qTargetQuatAnchorRelLink4, zyxLimitsLink4_);
    result_.robotLeftHandQuatAnchor_ = (qLink4 * qTargetQuatAnchorRelLink4).normalized();
    result_.robotLeftHandQuatTarget_ = qTargetQuatAnchor;
    result_.robotLeftHandPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
    if (result_.robotLeftHandPosAnchor_.norm() < posAnchorZeroThreshold_) {
      result_.robotLeftHandPosAnchor_ = defaultLeftHandPos_;
    }
  }
  result_.robotLeftHandQuatMeasEE_ = qEndEffector.normalized();
  result_.robotLeftHandQuatMeasEERealTime_ = qEndEffector.normalized();
  result_.robotLeftHandQuatMeasLink4_ = qLink4.normalized();
  result_.resetLeftHandDelta();
}

void IncrementalControlModule::updateRightArmPoseAnchor(const ArmPose& vrRightPose,
                                                        const std::vector<PoseData>& latestPoseConstraintList,
                                                        const Eigen::Vector3d& /*pEndEffector*/,
                                                        const Eigen::Quaterniond& qEndEffector,
                                                        const Eigen::Quaterniond& qLink4) {
  result_.humanRightHandPosAnchor_ = vrRightPose.position;
  result_.humanRightHandQuatAnchor_ = vrRightPose.quaternion.normalized();
  result_.humanRightHandQuatMeas_ = vrRightPose.quaternion.normalized();
  result_.humanRightHandQuatAnchorPython_ = result_.humanRightHandQuatAnchor_;

  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    Eigen::Quaterniond qTargetQuatAnchor =
        Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix).normalized();
    Eigen::Quaterniond qTargetQuatAnchorRelEE = qEndEffector.conjugate() * qTargetQuatAnchor;
    qTargetQuatAnchorRelEE = limitQuaternionAngleEulerZYX(qTargetQuatAnchorRelEE, zyxLimitsEE_);
    Eigen::Quaterniond qTargetQuatAnchorLimited = qEndEffector * qTargetQuatAnchorRelEE;
    Eigen::Quaterniond qTargetQuatAnchorRelLink4 = qLink4.conjugate() * qTargetQuatAnchorLimited;
    qTargetQuatAnchorRelLink4 = limitQuaternionAngleEulerZYX(qTargetQuatAnchorRelLink4, zyxLimitsLink4_);
    result_.robotRightHandQuatAnchor_ = (qLink4 * qTargetQuatAnchorRelLink4).normalized();
    result_.robotRightHandQuatTarget_ = qTargetQuatAnchor;
    result_.robotRightHandPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
    if (result_.robotRightHandPosAnchor_.norm() < posAnchorZeroThreshold_) {
      result_.robotRightHandPosAnchor_ = defaultRightHandPos_;
    }
  }
  result_.robotRightHandQuatMeasEE_ = qEndEffector.normalized();
  result_.robotRightHandQuatMeasEERealTime_ = qEndEffector.normalized();
  result_.robotRightHandQuatMeasLink4_ = qLink4.normalized();
  result_.resetRightHandDelta();
}

void IncrementalControlModule::computeFhanFiltering(const ArmPose& vrLeftPose,
                                                    const ArmPose& vrRightPose,
                                                    bool isLeftActive,
                                                    bool isRightActive,
                                                    double slerpQuatFactor) {
  if (slerpQuatFactor - 1.0 > slerpQuatFactorThreshold_) return;
  double fhanH = 1.0 / config_.publishRate;
  double fhanH0 = fhanH * config_.fhanKh0;
  double slerptR = config_.fhanR / 10;

  if (isLeftActive) {
    fhanStepForward(result_.leftSlerpQuatT_, result_.leftSlerpQuatDt_, slerpQuatFactor, slerptR, fhanH, fhanH0);
  }
  if (isRightActive) {
    fhanStepForward(result_.rightSlerpQuatT_, result_.rightSlerpQuatDt_, slerpQuatFactor, slerptR, fhanH, fhanH0);
  }

  for (int i = 0; i < 3; i++) {
    if (isLeftActive) {
      double rawLeftPosDelta = (vrLeftPose.position[i] - result_.humanLeftHandPosAnchor_[i]) * config_.deltaScale[i];
      fhanStepForward(result_.robotLeftHandDeltaPos_[i], result_.dotLeftHandDeltaPos_[i], rawLeftPosDelta,
                      config_.fhanR, fhanH, fhanH0);
    }
    if (isRightActive) {
      double rawRightPosDelta = (vrRightPose.position[i] - result_.humanRightHandPosAnchor_[i]) * config_.deltaScale[i];
      fhanStepForward(result_.robotRightHandDeltaPos_[i], result_.dotRightHandDeltaPos_[i], rawRightPosDelta,
                      config_.fhanR, fhanH, fhanH0);
    }
  }
}

void IncrementalControlModule::updateLastOnExit(const std::vector<PoseData>& latestPoseConstraintList) {
  result_.saveLastOnExit(latestPoseConstraintList);
}

void IncrementalControlModule::enterIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                                           const std::vector<PoseData>& latestPoseConstraintList,
                                                           const Eigen::Vector3d& pEndEffector,
                                                           const Eigen::Quaterniond& qEndEffector,
                                                           const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return;
  leftHandStatus_.ready(vrLeftPose.position, vrLeftPose.quaternion);
  updateLeftArmPoseAnchor(vrLeftPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);
}

void IncrementalControlModule::enterIncrementalModeRightArm(const ArmPose& vrRightPose,
                                                            const std::vector<PoseData>& latestPoseConstraintList,
                                                            const Eigen::Vector3d& pEndEffector,
                                                            const Eigen::Quaterniond& qEndEffector,
                                                            const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return;
  rightHandStatus_.ready(vrRightPose.position, vrRightPose.quaternion);
  updateRightArmPoseAnchor(vrRightPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);
}

void IncrementalControlModule::exitIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                                         const std::vector<PoseData>& latestPoseConstraintList,
                                                         const Eigen::Vector3d& pEndEffector,
                                                         const Eigen::Quaterniond& qEndEffector,
                                                         const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return;
  updateLeftArmPoseAnchor(vrLeftPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);
  leftHandStatus_.unready(vrLeftPose.position, vrLeftPose.quaternion);
  if (!leftHandStatus_.activated && !rightHandStatus_.activated) {
    updateLastOnExit(latestPoseConstraintList);
    result_.resetDelta();
    result_.resetSlerpFactor();
  }
}

void IncrementalControlModule::exitIncrementalModeRightArm(const ArmPose& vrRightPose,
                                                          const std::vector<PoseData>& latestPoseConstraintList,
                                                          const Eigen::Vector3d& pEndEffector,
                                                          const Eigen::Quaterniond& qEndEffector,
                                                          const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return;
  updateRightArmPoseAnchor(vrRightPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);
  rightHandStatus_.unready(vrRightPose.position, vrRightPose.quaternion);
  if (!leftHandStatus_.activated && !rightHandStatus_.activated) {
    updateLastOnExit(latestPoseConstraintList);
    result_.resetDelta();
    result_.resetSlerpFactor();
  }
}

IncrementalPoseResult IncrementalControlModule::computeIncrementalPose(const ArmPose& vrLeftPose,
                                                                        const ArmPose& vrRightPose,
                                                                        bool isLeftActive,
                                                                        bool isRightActive,
                                                                        const Eigen::Quaterniond& qLeftEndEffector,
                                                                        const Eigen::Quaterniond& qRightEndEffector) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_ || (!leftHandStatus_.activated && !rightHandStatus_.activated)) {
    result_.isValid_ = false;
    return result_;
  }

  if (isLeftActive && qLeftEndEffector.norm() > 1e-6) {
    result_.robotLeftHandQuatMeasEERealTime_ = qLeftEndEffector.normalized();
  }
  if (isRightActive && qRightEndEffector.norm() > 1e-6) {
    result_.robotRightHandQuatMeasEERealTime_ = qRightEndEffector.normalized();
  }

  if (isLeftActive) result_.humanLeftHandQuatMeas_ = vrLeftPose.quaternion.normalized();
  if (isRightActive) result_.humanRightHandQuatMeas_ = vrRightPose.quaternion.normalized();

  computeFhanFiltering(vrLeftPose, vrRightPose, isLeftActive, isRightActive);
  result_.slerpQuat(vrLeftPose.quaternion, vrRightPose.quaternion, isLeftActive, isRightActive);

  if (result_.usePythonIncrementalOrientation_) {
    if (isLeftActive) {
      Eigen::Quaterniond qCur = result_.humanLeftHandQuatMeas_.normalized();
      Eigen::Quaterniond qAnchor = result_.humanLeftHandQuatAnchorPython_.normalized();
      Eigen::Quaterniond qDelta = (qCur * qAnchor.conjugate()).normalized();
      if (quaternionAngleRad(qDelta) < result_.pythonOrientationThresholdRad_) {
        qDelta.setIdentity();
      } else {
        result_.humanLeftHandQuatAnchorPython_ = qCur;
        result_.robotLeftHandQuatTarget_ = (qDelta * result_.robotLeftHandQuatTarget_).normalized();
      }
      result_.leftHandDeltaQuatLast_ = qDelta;
    }
    if (isRightActive) {
      Eigen::Quaterniond qCur = result_.humanRightHandQuatMeas_.normalized();
      Eigen::Quaterniond qAnchor = result_.humanRightHandQuatAnchorPython_.normalized();
      Eigen::Quaterniond qDelta = (qCur * qAnchor.conjugate()).normalized();
      if (quaternionAngleRad(qDelta) < result_.pythonOrientationThresholdRad_) {
        qDelta.setIdentity();
      } else {
        result_.humanRightHandQuatAnchorPython_ = qCur;
        result_.robotRightHandQuatTarget_ = (qDelta * result_.robotRightHandQuatTarget_).normalized();
      }
      result_.rightHandDeltaQuatLast_ = qDelta;
    }
  }

  result_.isValid_ = true;
  return result_;
}

IncrementalPoseResult IncrementalControlModule::computeIncrementalPoseLeftArm(const ArmPose& vrLeftPose,
                                                                              bool isLeftActive,
                                                                              const Eigen::Quaterniond& qLeftEndEffector) {
  ArmPose vrRightPose;
  vrRightPose.position = result_.humanRightHandPosAnchor_;
  vrRightPose.quaternion = result_.humanRightHandQuatMeas_;
  return computeIncrementalPose(vrLeftPose, vrRightPose, isLeftActive, false, qLeftEndEffector,
                               Eigen::Quaterniond::Identity());
}

IncrementalPoseResult IncrementalControlModule::computeIncrementalPoseRightArm(const ArmPose& vrRightPose,
                                                                               bool isRightActive,
                                                                               const Eigen::Quaterniond& qRightEndEffector) {
  ArmPose vrLeftPose;
  vrLeftPose.position = result_.humanLeftHandPosAnchor_;
  vrLeftPose.quaternion = result_.humanLeftHandQuatMeas_;
  return computeIncrementalPose(vrLeftPose, vrRightPose, false, isRightActive, Eigen::Quaterniond::Identity(),
                               qRightEndEffector);
}

IncrementalPoseResult IncrementalControlModule::getLatestIncrementalResult() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_;
}

Eigen::Vector3d IncrementalControlModule::getRobotLeftHandAnchorPos() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotLeftHandAnchorPos();
}
Eigen::Vector3d IncrementalControlModule::getRobotRightHandAnchorPos() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotRightHandAnchorPos();
}
Eigen::Quaterniond IncrementalControlModule::getRobotLeftHandAnchorQuat() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotLeftHandAnchorQuat();
}
Eigen::Quaterniond IncrementalControlModule::getRobotRightHandAnchorQuat() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotRightHandAnchorQuat();
}

void IncrementalControlModule::setHandQuatSeeds(const Eigen::Quaterniond& leftHandQuatSeed,
                                                const Eigen::Quaterniond& rightHandQuatSeed,
                                                bool isIncrementalOrientation) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!isIncrementalOrientation) {
    result_.robotLeftHandQuatAnchor_ = leftHandQuatSeed.normalized();
    result_.robotRightHandQuatAnchor_ = rightHandQuatSeed.normalized();
  }
  result_.robotLeftHandQuatSlerpDes_ = result_.robotLeftHandQuatAnchor_;
  result_.robotRightHandQuatSlerpDes_ = result_.robotRightHandQuatAnchor_;
  result_.robotLeftHandQuatTarget_ = result_.robotLeftHandQuatAnchor_;
  result_.robotRightHandQuatTarget_ = result_.robotRightHandQuatAnchor_;
}

bool IncrementalControlModule::detectLeftArmMove(const Eigen::Vector3d& currentLeftHandPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && leftHandStatus_.detectMovement(currentLeftHandPos, config_.armMoveThreshold);
}
bool IncrementalControlModule::detectRightArmMove(const Eigen::Vector3d& currentRightHandPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && rightHandStatus_.detectMovement(currentRightHandPos, config_.armMoveThreshold);
}

bool IncrementalControlModule::shouldEnterIncrementalModeLeftArm(bool isLeftGrip) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && !leftHandStatus_.activated && isLeftGrip;
}
bool IncrementalControlModule::shouldEnterIncrementalModeRightArm(bool isRightGrip) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && !rightHandStatus_.activated && isRightGrip;
}
bool IncrementalControlModule::shouldExitIncrementalModeLeftArm(bool isLeftArmCtrlModeActive) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_ || !leftHandStatus_.activated) return false;
  return !isLeftArmCtrlModeActive;
}
bool IncrementalControlModule::shouldExitIncrementalModeRightArm(bool isRightArmCtrlModeActive) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_ || !rightHandStatus_.activated) return false;
  return !isRightArmCtrlModeActive;
}

bool IncrementalControlModule::isIncrementalMode() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && (leftHandStatus_.activated || rightHandStatus_.activated);
}
bool IncrementalControlModule::isIncrementalModeLeftArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && leftHandStatus_.activated;
}
bool IncrementalControlModule::isIncrementalModeRightArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && rightHandStatus_.activated;
}
bool IncrementalControlModule::hasLeftArmMoved() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && leftHandStatus_.moving;
}
bool IncrementalControlModule::hasRightArmMoved() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && rightHandStatus_.moving;
}

void IncrementalControlModule::updateConfig(const IncrementalControlConfig& config) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  config_ = config;
  result_.usePythonIncrementalOrientation_ = config_.usePythonIncrementalOrientation;
  result_.pythonOrientationThresholdRad_ = config_.pythonOrientationThresholdRad;
}

const IncrementalControlConfig& IncrementalControlModule::getConfig() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return config_;
}

void IncrementalControlModule::reset() {
  std::lock_guard<std::mutex> lock(stateMutex_);
  result_ = IncrementalPoseResult();
  leftHandStatus_.reset();
  rightHandStatus_.reset();
}

}  // namespace ik
}  // namespace leju
