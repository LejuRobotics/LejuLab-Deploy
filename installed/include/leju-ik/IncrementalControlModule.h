#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <tuple>
#include <utility>
#include <vector>

#include "leju-ik/ik_math.h"
#include "leju-ik/ik_types.h"

namespace leju {
namespace ik {

struct IncrementalPoseResult {
  bool isValid() const { return isValid_; }

  std::tuple<Eigen::Quaterniond, Eigen::Quaterniond, Eigen::Vector3d, Eigen::Vector3d> getLatestIncrementalHandPose(
      bool incrementalPos = true,
      bool incrementalQuat = false,
      bool smoothRotation = true) const;

  Eigen::Vector3d getLeftAnchorPos() const;
  Eigen::Vector3d getRightAnchorPos() const;
  Eigen::Vector3d getRobotLeftHandAnchorPos() const;
  Eigen::Vector3d getRobotRightHandAnchorPos() const;
  Eigen::Quaterniond getRobotLeftHandAnchorQuat() const;
  Eigen::Quaterniond getRobotRightHandAnchorQuat() const;

  friend class IncrementalControlModule;

 private:
  bool isValid_ = false;
  Eigen::Vector3d robotLeftHandPosAnchor_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d humanLeftHandPosAnchor_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d robotLeftHandDeltaPos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dotLeftHandDeltaPos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d robotRightHandPosAnchor_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d humanRightHandPosAnchor_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d robotRightHandDeltaPos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dotRightHandDeltaPos_ = Eigen::Vector3d::Zero();

  Eigen::Quaterniond robotLeftHandQuatAnchor_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond robotLeftHandQuatSlerpDes_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond robotLeftHandQuatMeasEE_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond robotLeftHandQuatMeasEERealTime_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond robotLeftHandQuatMeasLink4_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond humanLeftHandQuatAnchor_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond humanLeftHandQuatMeas_ = Eigen::Quaterniond::Identity();

  Eigen::Quaterniond robotRightHandQuatAnchor_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond robotRightHandQuatSlerpDes_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond robotRightHandQuatMeasEE_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond robotRightHandQuatMeasEERealTime_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond robotRightHandQuatMeasLink4_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond humanRightHandQuatAnchor_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond humanRightHandQuatMeas_ = Eigen::Quaterniond::Identity();

  bool usePythonIncrementalOrientation_ = true;
  double pythonOrientationThresholdRad_ = 0.01;
  Eigen::Quaterniond robotLeftHandQuatTarget_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond robotRightHandQuatTarget_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond humanLeftHandQuatAnchorPython_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond humanRightHandQuatAnchorPython_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond leftHandDeltaQuatLast_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond rightHandDeltaQuatLast_ = Eigen::Quaterniond::Identity();

  double leftSlerpQuatT_ = 0.0, leftSlerpQuatDt_ = 0.0;
  double rightSlerpQuatT_ = 0.0, rightSlerpQuatDt_ = 0.0;
  Eigen::Vector3d zyxLimitsFinal_ = Eigen::Vector3d(0.95 * M_PI / 2.0, M_PI / 2.0, M_PI / 2.0);
  Eigen::Vector3d zyxLimitsEE_ = Eigen::Vector3d(M_PI / 2.0, M_PI / 2.0, M_PI / 2.0);
  Eigen::Vector3d zyxLimitsLink4_ = Eigen::Vector3d(M_PI / 2.0, 0.6, 0.6);

  void saveLastOnExit(const std::vector<PoseData>& latestPoseConstraintList);
  void resetDelta();
  void resetLeftHandDelta();
  void resetRightHandDelta();
  void resetSlerpFactor();
  void slerpQuat(const Eigen::Quaterniond& leftHandQuat,
                 const Eigen::Quaterniond& rightHandQuat,
                 bool isLeftActive,
                 bool isRightActive);
  std::pair<Eigen::Quaterniond, Eigen::Quaterniond> getLatestRobotLeftHandQuatInc(bool smoothRotation = true) const;
  std::pair<Eigen::Quaterniond, Eigen::Quaterniond> getLatestRobotRightHandQuatInc(bool smoothRotation = true) const;
};

struct IncrementalControlConfig {
  double fhanR = 900.0;
  double fhanKh0 = 6.0;
  Eigen::Vector3d deltaScale = Eigen::Vector3d(1.0, 1.0, 1.0);
  double maxPosDiff = 0.45;
  double armMoveThreshold = 0.01;
  double publishRate = 100.0;
  bool usePythonIncrementalOrientation = true;
  double pythonOrientationThresholdRad = 0.01;
  Eigen::Vector3d zyxLimitsFinal = Eigen::Vector3d(0.95 * M_PI / 2.0, M_PI / 2.0, M_PI / 2.0);
  Eigen::Vector3d zyxLimitsEE = Eigen::Vector3d(M_PI / 2.0, M_PI / 2.0, M_PI / 2.0);
  Eigen::Vector3d zyxLimitsLink4 = Eigen::Vector3d(M_PI / 2.0, 0.6, 0.6);
};

struct HandStatus {
  bool activated = false;
  bool moving = false;
  Eigen::Vector3d lastPosition = Eigen::Vector3d::Zero();
  Eigen::Quaterniond lastOrientation = Eigen::Quaterniond::Identity();
  bool isFirstFrame = true;
  double lastMovementDistance = 0.0;

  void ready(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
  void unready(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
  bool detectMovement(const Eigen::Vector3d& currentPosition, double threshold = 0.01);
  void reset();
};

class IncrementalControlModule {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit IncrementalControlModule(const IncrementalControlConfig& config = IncrementalControlConfig{});
  ~IncrementalControlModule() = default;

  IncrementalControlModule(const IncrementalControlModule&) = delete;
  IncrementalControlModule& operator=(const IncrementalControlModule&) = delete;

  void enterIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                  const std::vector<PoseData>& latestPoseConstraintList,
                                  const Eigen::Vector3d& pEndEffector,
                                  const Eigen::Quaterniond& qEndEffector,
                                  const Eigen::Quaterniond& qLink4);

  void enterIncrementalModeRightArm(const ArmPose& vrRightPose,
                                   const std::vector<PoseData>& latestPoseConstraintList,
                                   const Eigen::Vector3d& pEndEffector,
                                   const Eigen::Quaterniond& qEndEffector,
                                   const Eigen::Quaterniond& qLink4);

  void exitIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                 const std::vector<PoseData>& latestPoseConstraintList,
                                 const Eigen::Vector3d& pEndEffector,
                                 const Eigen::Quaterniond& qEndEffector,
                                 const Eigen::Quaterniond& qLink4);

  void exitIncrementalModeRightArm(const ArmPose& vrRightPose,
                                  const std::vector<PoseData>& latestPoseConstraintList,
                                  const Eigen::Vector3d& pEndEffector,
                                  const Eigen::Quaterniond& qEndEffector,
                                  const Eigen::Quaterniond& qLink4);

  IncrementalPoseResult computeIncrementalPose(const ArmPose& vrLeftPose,
                                               const ArmPose& vrRightPose,
                                               bool isLeftActive = true,
                                               bool isRightActive = true,
                                               const Eigen::Quaterniond& qLeftEndEffector = Eigen::Quaterniond::Identity(),
                                               const Eigen::Quaterniond& qRightEndEffector = Eigen::Quaterniond::Identity());

  IncrementalPoseResult computeIncrementalPoseLeftArm(const ArmPose& vrLeftPose,
                                                      bool isLeftActive = true,
                                                      const Eigen::Quaterniond& qLeftEndEffector = Eigen::Quaterniond::Identity());

  IncrementalPoseResult computeIncrementalPoseRightArm(const ArmPose& vrRightPose,
                                                       bool isRightActive = true,
                                                       const Eigen::Quaterniond& qRightEndEffector = Eigen::Quaterniond::Identity());

  IncrementalPoseResult getLatestIncrementalResult() const;

  Eigen::Vector3d getRobotLeftHandAnchorPos() const;
  Eigen::Vector3d getRobotRightHandAnchorPos() const;
  Eigen::Quaterniond getRobotLeftHandAnchorQuat() const;
  Eigen::Quaterniond getRobotRightHandAnchorQuat() const;

  void setHandQuatSeeds(const Eigen::Quaterniond& leftHandQuatSeed,
                        const Eigen::Quaterniond& rightHandQuatSeed,
                        bool isIncrementalOrientation = false);

  bool detectLeftArmMove(const Eigen::Vector3d& currentLeftHandPos);
  bool detectRightArmMove(const Eigen::Vector3d& currentRightHandPos);

  bool shouldEnterIncrementalModeLeftArm(bool isLeftGrip) const;
  bool shouldEnterIncrementalModeRightArm(bool isRightGrip) const;
  bool shouldExitIncrementalModeLeftArm(bool isLeftArmCtrlModeActive) const;
  bool shouldExitIncrementalModeRightArm(bool isRightArmCtrlModeActive) const;

  bool isIncrementalMode() const;
  bool isIncrementalModeLeftArm() const;
  bool isIncrementalModeRightArm() const;
  bool hasLeftArmMoved() const;
  bool hasRightArmMoved() const;

  void updateConfig(const IncrementalControlConfig& config);
  const IncrementalControlConfig& getConfig() const;
  void reset();

 private:
  IncrementalPoseResult result_;
  IncrementalControlConfig config_;
  bool initialized_ = false;
  HandStatus leftHandStatus_;
  HandStatus rightHandStatus_;
  mutable std::mutex stateMutex_;
  Eigen::Vector3d zyxLimitsEE_;
  Eigen::Vector3d zyxLimitsLink4_;
  Eigen::Vector3d defaultLeftHandPos_;
  Eigen::Vector3d defaultRightHandPos_;
  double posAnchorZeroThreshold_;
  double slerpQuatFactorThreshold_;

  void updateLeftArmPoseAnchor(const ArmPose& vrLeftPose,
                               const std::vector<PoseData>& latestPoseConstraintList,
                               const Eigen::Vector3d& pEndEffector,
                               const Eigen::Quaterniond& qEndEffector,
                               const Eigen::Quaterniond& qLink4);
  void updateRightArmPoseAnchor(const ArmPose& vrRightPose,
                                const std::vector<PoseData>& latestPoseConstraintList,
                                const Eigen::Vector3d& pEndEffector,
                                const Eigen::Quaterniond& qEndEffector,
                                const Eigen::Quaterniond& qLink4);
  void computeFhanFiltering(const ArmPose& vrLeftPose,
                            const ArmPose& vrRightPose,
                            bool isLeftActive,
                            bool isRightActive,
                            double slerpQuatFactor = 1.0);
  void updateLastOnExit(const std::vector<PoseData>& latestPoseConstraintList);
};

}  // namespace ik
}  // namespace leju
