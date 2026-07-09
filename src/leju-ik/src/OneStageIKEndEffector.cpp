#include "leju-ik/OneStageIKEndEffector.h"

#include <chrono>
#include <stdexcept>

namespace leju {
namespace ik {

OneStageIKEndEffector::OneStageIKEndEffector(drake::multibody::MultibodyPlant<double>* plant,
                                             const std::vector<std::string>& ikConstraintFrameNames,
                                             const IKSolverConfig& config,
                                             size_t historyBufferSize)
    : BaseIKSolver(plant, ikConstraintFrameNames, config), historyBuffer_(historyBufferSize) {
  plant_context_ = plant_->CreateDefaultContext();
  if (nq_ == 14) {
    Eigen::VectorXd initialSolution = Eigen::VectorXd::Zero(nq_);
    initialSolution(1) = 0.3;
    initialSolution(2) = -0.3;
    initialSolution(3) = 0.3;
    initialSolution(8) = -0.3;
    initialSolution(9) = 0.3;
    initialSolution(10) = 0.3;
    for (size_t i = 0; i < historyBufferSize; ++i) {
      historyBuffer_.add(IKSolveResult(initialSolution, std::chrono::milliseconds(0)));
    }
  }
}

OneStageIKEndEffector::OneStageIKEndEffector(drake::multibody::MultibodyPlant<double>* plant,
                                             const std::vector<std::string>& ikConstraintFrameNames,
                                             const PointTrackIKSolverConfig& config)
    : BaseIKSolver(plant, ikConstraintFrameNames, config),
      historyBuffer_(config.historyBufferSize),
      pointTrackConfig_(std::make_unique<PointTrackIKSolverConfig>(config)) {
  plant_context_ = plant_->CreateDefaultContext();
  if (nq_ == 14) {
    Eigen::VectorXd initialSolution = Eigen::VectorXd::Zero(nq_);
    initialSolution(1) = 0.3;
    initialSolution(2) = -0.3;
    initialSolution(3) = 0.3;
    initialSolution(8) = -0.3;
    initialSolution(9) = 0.3;
    initialSolution(10) = 0.3;
    for (size_t i = 0; i < config.historyBufferSize; ++i) {
      historyBuffer_.add(IKSolveResult(initialSolution, std::chrono::milliseconds(0)));
    }
  }
}

IKSolveResult OneStageIKEndEffector::solveIK(const std::vector<PoseData>& PoseConstraintList,
                                             ArmIdx controlArmIndex,
                                             const Eigen::VectorXd& jointMidValues) {
  if (!plant_context_) return IKSolveResult(nq_, "plant_context_ is null");
  if (nq_ != 14) return IKSolveResult(nq_, "nq should be 14");
  if (!preSolveCheck(PoseConstraintList)) return IKSolveResult(nq_, "preSolveCheck failed");

  bool useJointLimit = true;
  drake::multibody::InverseKinematics endEffectorIK(*plant_, useJointLimit);
  initInverseKinematicsSolver(endEffectorIK, SolverType::SNOPT);

  Eigen::VectorXd referenceSolution = getWarmStartSolution();
  if (!historyBuffer_.empty()) {
    Eigen::VectorXd meanSolution = getMeanSolution();
    if (meanSolution.size() == nq_ && meanSolution.norm() > 1e-6) referenceSolution = meanSolution;
  }
  if (jointMidValues.size() == nq_ && jointMidValues.norm() > 1e-6) {
    referenceSolution = jointMidValues;
  }

  currentReferenceSolution_ = referenceSolution;
  setConstraints(endEffectorIK,
                 PoseConstraintList,
                 controlArmIndex,
                 Eigen::VectorXd::Zero(nq_),
                 referenceSolution);

  auto startTime = std::chrono::high_resolution_clock::now();
  auto ikResult = solveDrakeIK(endEffectorIK, referenceSolution, "SolveEndEffectorIK");
  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

  if (!ikResult.first) return IKSolveResult(nq_, "EndEffectorIK solve failed");

  updateLatestSolution(ikResult.second);
  IKSolveResult result(ikResult.second, duration);
  historyBuffer_.add(result);
  return result;
}

void OneStageIKEndEffector::setConstraints(drake::multibody::InverseKinematics& ik,
                                          const std::vector<PoseData>& PoseConstraintList,
                                          ArmIdx controlArmIndex,
                                          const Eigen::VectorXd& /*initialGuess*/,
                                          const Eigen::VectorXd& referenceSolution) const {
  double eeWeight = pointTrackConfig_ ? pointTrackConfig_->eeTrackingWeight : 4e3;
  double elbowWeight = pointTrackConfig_ ? pointTrackConfig_->elbowTrackingWeight : 4e2;
  double link6Weight = pointTrackConfig_ ? pointTrackConfig_->link6TrackingWeight : 4e3;
  double virtualThumbWeight = pointTrackConfig_ ? pointTrackConfig_->virtualThumbTrackingWeight : 4e3;

  if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].position,
                         plant_->GetFrameByName("zarm_l7_end_effector"), Eigen::Vector3d::Zero(),
                         eeWeight * Eigen::Matrix3d::Identity());
    }
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position,
                         plant_->GetFrameByName("zarm_l4_link"), Eigen::Vector3d::Zero(),
                         elbowWeight * Eigen::Matrix3d::Identity());
    }
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_LINK6) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_LINK6].position,
                         plant_->GetFrameByName("zarm_l6_link"), Eigen::Vector3d::Zero(),
                         link6Weight * Eigen::Matrix3d::Identity());
    }
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB) {
      try {
        ik.AddPositionCost(plant_->world_frame(),
                           PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].position,
                           plant_->GetFrameByName("zarm_l7_virtual_thumb_link"), Eigen::Vector3d::Zero(),
                           virtualThumbWeight * Eigen::Matrix3d::Identity());
      } catch (const std::logic_error&) {
        // Virtual thumb frame absent in this model; skip the cost (graceful degradation).
      }
    }
  }

  if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].position,
                         plant_->GetFrameByName("zarm_r7_end_effector"), Eigen::Vector3d::Zero(),
                         eeWeight * Eigen::Matrix3d::Identity());
    }
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position,
                         plant_->GetFrameByName("zarm_r4_link"), Eigen::Vector3d::Zero(),
                         elbowWeight * Eigen::Matrix3d::Identity());
    }
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_LINK6) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_LINK6].position,
                         plant_->GetFrameByName("zarm_r6_link"), Eigen::Vector3d::Zero(),
                         link6Weight * Eigen::Matrix3d::Identity());
    }
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB) {
      try {
        ik.AddPositionCost(plant_->world_frame(),
                           PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].position,
                           plant_->GetFrameByName("zarm_r7_virtual_thumb_link"), Eigen::Vector3d::Zero(),
                           virtualThumbWeight * Eigen::Matrix3d::Identity());
      } catch (const std::logic_error&) {
        // Virtual thumb frame absent in this model; skip the cost (graceful degradation).
      }
    }
  }

  std::vector<double> jointSmoothWeight(nq_, pointTrackConfig_ ? pointTrackConfig_->jointSmoothWeightDefault : 5e1);
  if (pointTrackConfig_) {
    jointSmoothWeight[0] = pointTrackConfig_->jointSmoothWeight0;
    jointSmoothWeight[1] = pointTrackConfig_->jointSmoothWeight1;
    jointSmoothWeight[2] = pointTrackConfig_->jointSmoothWeight2;
    jointSmoothWeight[3] = pointTrackConfig_->jointSmoothWeight3;
    jointSmoothWeight[4] = pointTrackConfig_->jointSmoothWeight4;
    jointSmoothWeight[5] = pointTrackConfig_->jointSmoothWeight5;
    jointSmoothWeight[6] = pointTrackConfig_->jointSmoothWeight6;
    jointSmoothWeight[7] = pointTrackConfig_->jointSmoothWeight0;
    jointSmoothWeight[8] = pointTrackConfig_->jointSmoothWeight1;
    jointSmoothWeight[9] = pointTrackConfig_->jointSmoothWeight2;
    jointSmoothWeight[10] = pointTrackConfig_->jointSmoothWeight3;
    jointSmoothWeight[11] = pointTrackConfig_->jointSmoothWeight4;
    jointSmoothWeight[12] = pointTrackConfig_->jointSmoothWeight5;
    jointSmoothWeight[13] = pointTrackConfig_->jointSmoothWeight6;
  } else {
    jointSmoothWeight[3] = jointSmoothWeight[10] = 1e1;
    jointSmoothWeight[4] = jointSmoothWeight[5] = jointSmoothWeight[6] = 1e-3;
    jointSmoothWeight[11] = jointSmoothWeight[12] = jointSmoothWeight[13] = 1e-3;
  }

  Eigen::VectorXd weightVec = Eigen::VectorXd::Map(jointSmoothWeight.data(), jointSmoothWeight.size());
  Eigen::MatrixXd W = weightVec.asDiagonal();
  ik.get_mutable_prog()->AddQuadraticErrorCost(W, referenceSolution, ik.q());
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> OneStageIKEndEffector::FK(const Eigen::VectorXd& q,
                                                                          const std::string& frameName,
                                                                          int expectedSize) {
  if (!plant_context_) return {Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
  if (expectedSize > 0 && q.size() != static_cast<size_t>(expectedSize)) {
    return {Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
  }
  plant_->SetPositions(plant_context_.get(), q);
  try {
    const auto& target_frame = plant_->GetFrameByName(frameName);
    const auto& ref_frame = (ConstraintFrames_.size() > 0) ? *ConstraintFrames_[0] : plant_->world_frame();
    auto pose = target_frame.CalcPose(*plant_context_, ref_frame);
    return {pose.translation(), pose.rotation().ToQuaternion()};
  } catch (...) {
    return {Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
  }
}

}  // namespace ik
}  // namespace leju
