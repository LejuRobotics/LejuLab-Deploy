#pragma once

#include <Eigen/Dense>

#include <chrono>
#include <string>
#include <utility>

namespace leju {
namespace ik {

class HandSmoother {
 public:
  HandSmoother(const std::string& handName,
               const std::string& fkFrameName,
               const Eigen::Vector3d& defaultPosOnExit);

  void reset();
  void resetSmoothState(const Eigen::Vector3d& currentPos);
  void setSmoothState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel);
  void setModeChangingState(bool changingMaintain, bool changingInstant);
  std::pair<bool, bool> getModeChangingState() const;
  std::pair<bool, bool> updateModeChangingStateIfNeeded(bool ctrlModeChanged);
  void processActiveModeInterpolation(Eigen::Vector3d& scaledHandPos,
                                      bool& isChangingInstant,
                                      const Eigen::Vector3d& defaultPos,
                                      const std::string& handName);
  void processInactiveModeInterpolation(Eigen::Vector3d& scaledHandPos,
                                        bool& isChangingInstant,
                                        const Eigen::Vector3d& defaultPos,
                                        const std::string& handName);
  void updateWithFhan(const Eigen::Vector3d& targetPos,
                      double r = 0.3,
                      double h = 0.01,
                      double h0 = 0.04);

  void cancelMaintain();
  bool updateChangingMode(const Eigen::Vector3d& targetPos,
                          class OneStageIKEndEffector* ikSolverPtr,
                          const Eigen::VectorXd& armJoints,
                          int jointStateSize,
                          double threshold = 0.02);

  bool isMaintaining() const { return ctrlModeChangingMaintain_; }

  const Eigen::Vector3d& getSmoothPosition() const { return smootheIntermidiatePos_; }
  const Eigen::Vector3d& getSmoothVelocity() const { return dotSmootheIntermidiatePos_; }
  const Eigen::Vector3d& getDefaultPosOnExit() const { return defaultPosOnExit_; }

 private:
  std::string handName_;
  std::string fkFrameName_;
  Eigen::Vector3d smootheIntermidiatePos_;
  Eigen::Vector3d dotSmootheIntermidiatePos_;
  bool ctrlModeChangingMaintain_ = false;
  bool ctrlModeChangingInstant_ = false;
  Eigen::Vector3d defaultPosOnExit_;
};

}  // namespace ik
}  // namespace leju
