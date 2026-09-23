#pragma once

#include <functional>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "leju-ik/DrakeElbowHandPointOpt.hpp"
#include "leju-ik/HandSmoother.h"
#include "leju-ik/IncrementalControlModule.h"
#include "leju-ik/OneStageIKEndEffector.h"
#include "leju-ik/Quest3ArmInfoTransformer.h"

namespace leju {
namespace ik {

struct DrakeVelocityIKBoundsConfig {
  double xUpperOffset = 0.6;
  double zLower = -0.3;
  double zUpperOffset = 0.1;
  double leftYLower = -0.2;
  double leftYUpperOffset = 0.5;
  double rightYLowerOffset = -0.5;
  double rightYUpper = 0.2;
};

class Quest3IkIncrementalAPI {
 public:
  using ArmJointCallback = std::function<void(const std::vector<double>& q_rad)>;
  using HeadBodyPoseCallback = std::function<void(const HeadBodyPose&)>;

  Quest3IkIncrementalAPI();
  ~Quest3IkIncrementalAPI();

  bool init(const std::string& urdf_path, const nlohmann::json& config);
  bool init(const std::string& assets_path, uint16_t major, uint16_t minor,
            const nlohmann::json& config = nlohmann::json());
  bool init(const std::string& models_base, const std::string& config_base,
            uint16_t major, uint16_t minor,
            const nlohmann::json& config = nlohmann::json(),
            const std::string& quest3_yaml_path = std::string());

  void onBonePoses(const PoseInfoList& bone_poses);
  void onJoystick(const JoyStickData& joystick);
  void onArmCtrlModeState(const ArmControlModeState& state);
  void onSensorArmJoints(const std::vector<double>& q_rad);

  void setArmJointCallback(ArmJointCallback cb) { armJointCallback_ = cb; }
  void setHeadBodyPoseCallback(HeadBodyPoseCallback cb) { headBodyPoseCallback_ = cb; }

  void setArmMode(int mode);
  void setPublishRate(double hz) { publishRate_ = hz; }
  void runOnce();

  bool isRunning() const { return transformer_ && transformer_->isRunning(); }
  bool isIncrementalMode() const {
    return incrementalController_ && incrementalController_->isIncrementalMode();
  }

 private:
  bool buildPlantAndIK(const std::string& urdf_path,
                       const nlohmann::json& config,
                       const std::string& robotModel);
  std::vector<std::string> loadFrameNamesFromConfig(const nlohmann::json& config);

  DrakeVelocityIKWeightConfig loadDrakeVelocityIKWeightsFromJson(const nlohmann::json& config);
  DrakeVelocityIKBoundsConfig loadDrakeVelocityIKBoundsFromJson(const nlohmann::json& config);
  void loadDrakeVelocityIKGeometryFromJson(const nlohmann::json& config);
  PointTrackIKSolverConfig loadPointTrackIKSolverConfigFromJson(const nlohmann::json& config);

  bool hasValidSensorJoints(Eigen::VectorXd& qForFK);
  void initializePoseConstraintList();

  bool computeLeftEndEffectorFK(const Eigen::VectorXd& q, Eigen::Vector3d& pOut,
                                Eigen::Quaterniond& qOut);
  bool computeRightEndEffectorFK(const Eigen::VectorXd& q, Eigen::Vector3d& pOut,
                                 Eigen::Quaterniond& qOut);
  bool computeLeftLink4FK(const Eigen::VectorXd& q, Eigen::Vector3d& pOut,
                          Eigen::Quaterniond& qOut);
  bool computeRightLink4FK(const Eigen::VectorXd& q, Eigen::Vector3d& pOut,
                           Eigen::Quaterniond& qOut);
  bool computeLeftLink6FK(const Eigen::VectorXd& q, Eigen::Vector3d& pOut,
                          Eigen::Quaterniond& qOut);
  bool computeRightLink6FK(const Eigen::VectorXd& q, Eigen::Vector3d& pOut,
                           Eigen::Quaterniond& qOut);

  void rebuildPoseConstraintListFromCurrentFk(const Eigen::Vector3d& leftLink4Pos,
                                              const Eigen::Quaterniond& leftLink4Quat,
                                              const Eigen::Vector3d& rightLink4Pos,
                                              const Eigen::Quaterniond& rightLink4Quat,
                                              const Eigen::Vector3d& leftLink6Pos,
                                              const Eigen::Quaterniond& leftLink6Quat,
                                              const Eigen::Vector3d& rightLink6Pos,
                                              const Eigen::Quaterniond& rightLink6Quat,
                                              const Eigen::Vector3d& leftEEPos,
                                              const Eigen::Quaterniond& leftEEQuat,
                                              const Eigen::Vector3d& rightEEPos,
                                              const Eigen::Quaterniond& rightEEQuat);
  void updateLeftConstraintList(const Eigen::Vector3d& leftHandPos,
                                const Eigen::Quaterniond& leftHandQuat,
                                const Eigen::Vector3d& leftElbowPos);
  void updateRightConstraintList(const Eigen::Vector3d& rightHandPos,
                                 const Eigen::Quaterniond& rightHandQuat,
                                 const Eigen::Vector3d& rightElbowPos);
  void buildPoseConstraintListFromIncrementalResult(const IncrementalPoseResult& incResult,
                                                    std::vector<PoseData>& out);
  bool detectLeftArmMove(const ArmPose& vrLeftPose);
  bool detectRightArmMove(const ArmPose& vrRightPose);
  bool updateLatestIncrementalResult(const ArmPose& vrLeftPose,
                                     const ArmPose& vrRightPose,
                                     bool leftCanProcess,
                                     bool rightCanProcess,
                                     const Eigen::Quaterniond& qLeftEndEffector,
                                     const Eigen::Quaterniond& qRightEndEffector);
  bool processChangingDataLeftArm(bool leftHandCtrlModeChanged);
  bool processChangingDataRightArm(bool rightHandCtrlModeChanged);
  bool processDataLeftArm();
  bool processDataRightArm();
  bool fsmChange(const ArmPose& vrLeftPose,
                 const ArmPose& vrRightPose,
                 const Eigen::Quaterniond& qLeftEndEffector,
                 const Eigen::Quaterniond& qRightEndEffector,
                 bool leftHandCtrlModeChanged,
                 bool rightHandCtrlModeChanged);
  bool fsmProcess(const ArmPose& vrLeftPose,
                  const ArmPose& vrRightPose,
                  const Eigen::Quaterniond& qLeftEndEffector,
                  const Eigen::Quaterniond& qRightEndEffector,
                  bool& hasIncrementalResult);
  void solveIk();
  bool updateLeftHandChangingMode(const Eigen::Vector3d& leftTargetPos);
  bool updateRightHandChangingMode(const Eigen::Vector3d& rightTargetPos);

  void syncRuntimeStateFromCurrentFk(const Eigen::Vector3d& leftLink4Pos,
                                     const Eigen::Quaterniond& leftLink4Quat,
                                     const Eigen::Vector3d& rightLink4Pos,
                                     const Eigen::Quaterniond& rightLink4Quat,
                                     const Eigen::Vector3d& leftLink6Pos,
                                     const Eigen::Quaterniond& leftLink6Quat,
                                     const Eigen::Vector3d& rightLink6Pos,
                                     const Eigen::Quaterniond& rightLink6Quat,
                                     const Eigen::Vector3d& leftEEPos,
                                     const Eigen::Quaterniond& leftEEQuat,
                                     const Eigen::Vector3d& rightEEPos,
                                     const Eigen::Quaterniond& rightEEQuat);

  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagramContext_;
  drake::multibody::MultibodyPlant<double>* plant_ = nullptr;

  std::unique_ptr<Quest3ArmInfoTransformer> transformer_;
  std::unique_ptr<OneStageIKEndEffector> oneStageIk_;
  std::unique_ptr<IncrementalControlModule> incrementalController_;
  std::unique_ptr<DrakeVelocityIKSolver> leftVelocityIkSolverPtr_;
  std::unique_ptr<DrakeVelocityIKSolver> rightVelocityIkSolverPtr_;
  std::unique_ptr<HandSmoother> leftHandSmoother_;
  std::unique_ptr<HandSmoother> rightHandSmoother_;

  ArmJointCallback armJointCallback_;
  HeadBodyPoseCallback headBodyPoseCallback_;
  double publishRate_ = 100.0;
  ArmIdx ctrlArmIdx_ = ArmIdx::BOTH;

  std::mutex bonePosesMutex_;
  std::mutex joystickMutex_;
  std::mutex armCtrlModeMutex_;
  std::mutex sensorJointsMutex_;
  std::mutex modeMutex_;
  PoseInfoList latestBonePoses_;
  JoyStickData latestJoystick_;
  ArmControlModeState latestArmCtrlModeState_;
  Eigen::VectorXd sensorArmJoints_;
  Eigen::VectorXd filteredSensorArmJoints_;
  Eigen::VectorXd jointMidValues_;
  bool hasBonePoses_ = false;
  bool hasSensorJoints_ = false;
  bool leftArmCtrlModeActive_ = false;
  bool rightArmCtrlModeActive_ = false;
  bool pendingLeftArmCtrlModeChanged_ = false;
  bool pendingRightArmCtrlModeChanged_ = false;

  std::vector<PoseData> latestPoseConstraintList_;
  IncrementalPoseResult latestIncrementalResult_;

  Eigen::Vector3d leftEE2Link6Offset_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightEE2Link6Offset_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d leftThumb2Link6Offset_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightThumb2Link6Offset_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d robotLeftFixedShoulderPos_ = Eigen::Vector3d(-0.017499853, 0.2927, 0.4245);
  Eigen::Vector3d robotRightFixedShoulderPos_ = Eigen::Vector3d(-0.017499853, -0.2927, 0.4245);
  double l1_ = 0.2837;
  double l2_ = 0.2335;
  Eigen::Vector3d leftElbowFixedPoint_ = Eigen::Vector3d(-0.3, 0.5, 0.32);
  Eigen::Vector3d rightElbowFixedPoint_ = Eigen::Vector3d(-0.3, -0.5, 0.32);

  Eigen::Vector3d defaultLeftHandPosOnExit_ = Eigen::Vector3d(0.0, 0.293, -0.16);
  Eigen::Vector3d defaultRightHandPosOnExit_ = Eigen::Vector3d(0.0, -0.293, -0.16);
  bool useIncrementalHandOrientation_ = true;

  Eigen::Vector3d leftLink6Position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightLink6Position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d leftEndEffectorPosition_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightEndEffectorPosition_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d leftVirtualThumbPosition_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightVirtualThumbPosition_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond leftLink4Quat_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond rightLink4Quat_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond leftLink6Quat_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond rightLink6Quat_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond leftEndEffectorQuat_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond rightEndEffectorQuat_ = Eigen::Quaterniond::Identity();

  Eigen::Vector3d latestHumanLeftElbowPos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d latestHumanRightElbowPos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d latestRobotLeftElbowPos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d latestRobotRightElbowPos_ = Eigen::Vector3d::Zero();

  int currentArmMode_ = 1;
  int previousArmMode_ = 1;
  bool pendingEnterMode2_ = false;
  bool pendingExitMode2_ = false;
  bool prevLeftGripPressed_ = false;
  bool prevRightGripPressed_ = false;
  bool currentLeftGripPressed_ = false;
  bool currentRightGripPressed_ = false;
  double handChangingModeThreshold_ = 0.085;
};

}  // namespace ik
}  // namespace leju
