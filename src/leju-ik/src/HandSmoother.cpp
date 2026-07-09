#include "leju-ik/HandSmoother.h"

#include <algorithm>

#include "leju-ik/ik_math.h"
#include "leju-ik/OneStageIKEndEffector.h"

namespace leju {
namespace ik {

HandSmoother::HandSmoother(const std::string& handName,
                           const std::string& fkFrameName,
                           const Eigen::Vector3d& defaultPosOnExit)
    : handName_(handName), fkFrameName_(fkFrameName), defaultPosOnExit_(defaultPosOnExit) {
  smootheIntermidiatePos_ = defaultPosOnExit_;
  dotSmootheIntermidiatePos_.setZero();
}

void HandSmoother::reset() {
  ctrlModeChangingMaintain_ = false;
  ctrlModeChangingInstant_ = false;
  smootheIntermidiatePos_ = defaultPosOnExit_;
  dotSmootheIntermidiatePos_.setZero();
}

void HandSmoother::resetSmoothState(const Eigen::Vector3d& currentPos) {
  smootheIntermidiatePos_ = currentPos;
  dotSmootheIntermidiatePos_.setZero();
}

void HandSmoother::setSmoothState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel) {
  smootheIntermidiatePos_ = pos;
  dotSmootheIntermidiatePos_ = vel;
}

void HandSmoother::setModeChangingState(bool changingMaintain, bool changingInstant) {
  ctrlModeChangingMaintain_ = changingMaintain;
  ctrlModeChangingInstant_ = changingInstant;
}

std::pair<bool, bool> HandSmoother::getModeChangingState() const {
  return {ctrlModeChangingMaintain_, ctrlModeChangingInstant_};
}

std::pair<bool, bool> HandSmoother::updateModeChangingStateIfNeeded(bool ctrlModeChanged) {
  if (!ctrlModeChangingMaintain_) {
    ctrlModeChangingMaintain_ = ctrlModeChanged;
    ctrlModeChangingInstant_ = ctrlModeChanged;
  }
  return getModeChangingState();
}

void HandSmoother::processActiveModeInterpolation(Eigen::Vector3d& scaledHandPos,
                                                  bool& isChangingInstant,
                                                  const Eigen::Vector3d& defaultPos,
                                                  const std::string& handName) {
  (void)handName;
  if (isChangingInstant) {
    resetSmoothState(scaledHandPos);
    isChangingInstant = false;
  }
  scaledHandPos = defaultPos;
}

void HandSmoother::processInactiveModeInterpolation(Eigen::Vector3d& scaledHandPos,
                                                    bool& isChangingInstant,
                                                    const Eigen::Vector3d& defaultPos,
                                                    const std::string& handName) {
  (void)handName;
  if (isChangingInstant) {
    resetSmoothState(scaledHandPos);
    isChangingInstant = false;
  }
  (void)handName;
  updateWithFhan(defaultPos, 0.3, 0.01, 0.04);
  scaledHandPos = smootheIntermidiatePos_;
}

void HandSmoother::updateWithFhan(const Eigen::Vector3d& targetPos,
                                  double r,
                                  double h,
                                  double h0) {
  for (int i = 0; i < 3; ++i) {
    fhanStepForward(smootheIntermidiatePos_(i), dotSmootheIntermidiatePos_(i), targetPos(i), r, h,
                    h0);
  }
}

void HandSmoother::cancelMaintain() {
  ctrlModeChangingMaintain_ = false;
  ctrlModeChangingInstant_ = false;
}

bool HandSmoother::updateChangingMode(const Eigen::Vector3d& targetPos,
                                      OneStageIKEndEffector* ikSolverPtr,
                                      const Eigen::VectorXd& armJoints,
                                      int jointStateSize,
                                      double threshold) {
  if (!ctrlModeChangingMaintain_ || !ikSolverPtr) {
    return false;
  }
  if (armJoints.size() != jointStateSize) {
    return false;
  }

  auto [posReached, quatTmp] = ikSolverPtr->FK(armJoints, fkFrameName_, jointStateSize);
  (void)quatTmp;
  const double err = (posReached - targetPos).norm();
  if (err < threshold) {
    ctrlModeChangingMaintain_ = false;
    ctrlModeChangingInstant_ = false;
    return true;
  }
  return false;
}

}  // namespace ik
}  // namespace leju
