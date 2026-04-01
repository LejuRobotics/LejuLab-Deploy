#include "leju-ik/TwoStageTorsoIK.h"

#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/solve.h>

#include <chrono>
#include <iostream>

namespace leju {
namespace ik {

TwoStageTorsoIK::TwoStageTorsoIK(drake::multibody::MultibodyPlant<double>* plant,
                                 const std::vector<std::string>& ikConstraintFrameNames,
                                 double constraintTolerance,
                                 double solverTolerance,
                                 int maxSolverIterations,
                                 ArmIdx controlArmIndex,
                                 bool isWeldBaseLink)
    : plant_(plant),
      constraintTolerance_(constraintTolerance),
      solverTolerance_(solverTolerance),
      maxSolverIterations_(maxSolverIterations),
      controlArmIndex_(controlArmIndex),
      isWeldBaseLink_(isWeldBaseLink),
      stage1Result_(false, Eigen::VectorXd()),
      stage2Result_(false, Eigen::VectorXd()),
      hasLatestSolution_(false),
      hasJointLimits_(false) {
  if (plant_ == nullptr) {
    throw std::invalid_argument("Plant pointer is null");
  }
  if (ikConstraintFrameNames.empty()) {
    throw std::invalid_argument("Frame names should not be empty");
  }
  if (!plant_->is_finalized()) {
    plant_->Finalize();
  }

  nq_ = plant_->num_positions();
  frames_.clear();
  for (const auto& frameName : ikConstraintFrameNames) {
    try {
      frames_.push_back(&plant_->GetFrameByName(frameName));
    } catch (const std::exception& e) {
      std::cerr << "Error adding ikConstraintFrame: " << frameName << ", error: " << e.what() << std::endl;
      throw;
    }
  }

  initializeJointIndices();
  initializeWristFrames();
  latestSolution_ = Eigen::VectorXd::Zero(nq_);
  hasLatestSolution_ = false;
  initializeJointLimits();
}

std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::solveTwoStageIK(const std::vector<PoseData>& poseDataList,
                                                                  ArmIdx controlArmIndex,
                                                                  const Eigen::VectorXd& q0) {
  if (!plantContext_) {
    std::cerr << "Error: plantContext_ is null" << std::endl;
    return {false, Eigen::VectorXd()};
  }

  Eigen::VectorXd referenceSolution;
  if (q0.size() == nq_ && q0.norm() > 1e-6) {
    referenceSolution = q0;
  } else if (hasLatestSolution_ && latestSolution_.size() == nq_ && latestSolution_.norm() > 1e-6) {
    referenceSolution = latestSolution_;
  } else {
    referenceSolution = Eigen::VectorXd::Zero(nq_);
  }
  stage1Result_ = solveStage1(poseDataList, controlArmIndex, referenceSolution);
  if (!stage1Result_.first) {
    return {false, Eigen::VectorXd()};
  }
  stage2Result_ = solveStage2(poseDataList, controlArmIndex);
  if (!stage2Result_.first) {
    return {false, Eigen::VectorXd()};
  }

  Eigen::VectorXd limitedSolution = stage2Result_.second;
  if (hasJointLimits_) {
    limitedSolution = limitAngle(limitedSolution);
  }
  Eigen::VectorXd referenceForVelocity = hasLatestSolution_ ? latestSolution_ : Eigen::VectorXd::Zero(nq_);
  limitedSolution = limitAngleByVelocity(referenceForVelocity, limitedSolution, 720.0, 0.01);

  latestSolution_ = limitedSolution;
  hasLatestSolution_ = true;
  return {true, limitedSolution};
}

void TwoStageTorsoIK::initializeJointIndices() {
  leftWristIdx_.clear();
  rightWristIdx_.clear();
  if (nq_ >= 14) {
    leftWristIdx_ = {4, 5, 6};
    rightWristIdx_ = {11, 12, 13};
  }
}

void TwoStageTorsoIK::initializeWristFrames() {
  wristFrames_.clear();
  if (nq_ >= 14) {
    try {
      wristFrames_.push_back(&plant_->GetFrameByName("zarm_l6_link"));
      wristFrames_.push_back(&plant_->GetFrameByName("zarm_r6_link"));
    } catch (const std::exception& e) {
      std::cerr << "Error adding wrist frames: " << e.what() << std::endl;
      throw;
    }
  }
}

std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::solveStage1(const std::vector<PoseData>& poseDataList,
                                                              ArmIdx controlArmIndex,
                                                              const Eigen::VectorXd& q0) {
  if (poseDataList.size() != 5) {
    std::cerr << "poseDataList size invalid: " << poseDataList.size() << ", expected 5" << std::endl;
    return {false, Eigen::VectorXd()};
  }

  for (size_t i = 0; i < poseDataList.size(); ++i) {
    const auto& pos = poseDataList[i].position;
    if (pos.hasNaN() || !std::isfinite(pos(0)) || !std::isfinite(pos(1)) || !std::isfinite(pos(2)) ||
        (pos.array().abs() > 1e6).any()) {
      std::cerr << "Invalid position data at index " << i << std::endl;
      return {false, Eigen::VectorXd()};
    }
  }

  Eigen::VectorXd referenceSolution = q0;
  bool useJointLimit = true;
  drake::multibody::InverseKinematics stage1Ik(*plant_, useJointLimit);

  drake::solvers::SnoptSolver snopt;
  auto snoptId = snopt.solver_id();
  stage1Ik.get_mutable_prog()->SetSolverOption(snoptId, "Major Optimality Tolerance", solverTolerance_);
  stage1Ik.get_mutable_prog()->SetSolverOption(snoptId, "Major Iterations Limit", maxSolverIterations_);

  if (wristFrames_.size() > 0) {
    stage1Ik.AddPositionCost(plant_->world_frame(),
                             poseDataList[POSE_DATA_LIST_INDEX_LEFT_HAND].position,
                             *wristFrames_[0], Eigen::Vector3d::Zero(),
                             10.0 * Eigen::Matrix3d::Identity());
  } else if (frames_.size() > 1) {
    stage1Ik.AddPositionCost(plant_->world_frame(),
                             poseDataList[POSE_DATA_LIST_INDEX_LEFT_HAND].position,
                             *frames_[1], Eigen::Vector3d::Zero(),
                             10.0 * Eigen::Matrix3d::Identity());
  }

  if (wristFrames_.size() > 1) {
    stage1Ik.AddPositionCost(plant_->world_frame(),
                             poseDataList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position,
                             *wristFrames_[1], Eigen::Vector3d::Zero(),
                             10.0 * Eigen::Matrix3d::Identity());
  } else if (frames_.size() > 2) {
    stage1Ik.AddPositionCost(plant_->world_frame(),
                             poseDataList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position,
                             *frames_[2], Eigen::Vector3d::Zero(),
                             10.0 * Eigen::Matrix3d::Identity());
  }

  if (frames_.size() > 3) {
    stage1Ik.AddPositionCost(plant_->world_frame(),
                             poseDataList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position,
                             *frames_[3], Eigen::Vector3d::Zero(),
                             10.0 * Eigen::Matrix3d::Identity());
  }
  if (frames_.size() > 4) {
    stage1Ik.AddPositionCost(plant_->world_frame(),
                             poseDataList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position,
                             *frames_[4], Eigen::Vector3d::Zero(),
                             10.0 * Eigen::Matrix3d::Identity());
  }

  std::vector<double> stage1Weights(nq_, 0.2);
  if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : leftWristIdx_) {
      if (idx < nq_) stage1Weights[idx] = 5.0;
    }
  }
  if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : rightWristIdx_) {
      if (idx < nq_) stage1Weights[idx] = 5.0;
    }
  }

  Eigen::VectorXd weightVec = Eigen::VectorXd::Map(stage1Weights.data(), stage1Weights.size());
  Eigen::MatrixXd W_prev_solution = weightVec.asDiagonal();
  stage1Ik.get_mutable_prog()->AddQuadraticErrorCost(W_prev_solution, referenceSolution, stage1Ik.q());

  try {
    drake::solvers::MathematicalProgramResult result1 = drake::solvers::Solve(stage1Ik.prog(), referenceSolution);
    if (result1.is_success()) {
      return {true, result1.GetSolution(stage1Ik.q())};
    }
    return {false, Eigen::VectorXd()};
  } catch (const std::exception& e) {
    std::cerr << "SolveStage1 exception: " << e.what() << std::endl;
    return {false, Eigen::VectorXd()};
  }
}

std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::solveStage2(const std::vector<PoseData>& poseDataList,
                                                             ArmIdx controlArmIndex) {
  drake::multibody::InverseKinematics stage2Ik(*plant_, true);

  drake::solvers::SnoptSolver snopt;
  auto snoptId = snopt.solver_id();
  stage2Ik.get_mutable_prog()->SetSolverOption(snoptId, "Major Optimality Tolerance", solverTolerance_);
  stage2Ik.get_mutable_prog()->SetSolverOption(snoptId, "Major Iterations Limit", maxSolverIterations_);

  if ((controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) &&
      poseDataList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    bool hasValidLeftHandRotation =
        !poseDataList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix.isApprox(Eigen::Matrix3d::Identity(), 1e-6);
    if (hasValidLeftHandRotation && frames_.size() > 1) {
      drake::math::RotationMatrix<double> R_desired(poseDataList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix);
      stage2Ik.AddOrientationConstraint(plant_->world_frame(), R_desired, *frames_[1],
                                        drake::math::RotationMatrix<double>::Identity(), constraintTolerance_);
    }
  }

  if ((controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) &&
      poseDataList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    bool hasValidRightHandRotation =
        !poseDataList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix.isApprox(Eigen::Matrix3d::Identity(), 1e-6);
    if (hasValidRightHandRotation && frames_.size() > 2) {
      drake::math::RotationMatrix<double> R_desired(poseDataList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix);
      stage2Ik.AddOrientationConstraint(plant_->world_frame(), R_desired, *frames_[2],
                                        drake::math::RotationMatrix<double>::Identity(), constraintTolerance_);
    }
  }

  std::vector<double> stage2Weights(nq_, 100.0);
  if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : leftWristIdx_) {
      if (idx < nq_) stage2Weights[idx] = 0.05;
    }
  }
  if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : rightWristIdx_) {
      if (idx < nq_) stage2Weights[idx] = 0.05;
    }
  }

  Eigen::VectorXd weightVec = Eigen::VectorXd::Map(stage2Weights.data(), stage2Weights.size());
  Eigen::MatrixXd W_prev_solution = weightVec.asDiagonal();
  stage2Ik.get_mutable_prog()->AddQuadraticErrorCost(W_prev_solution, stage1Result_.second, stage2Ik.q());

  try {
    drake::solvers::MathematicalProgramResult result2 = drake::solvers::Solve(stage2Ik.prog(), stage1Result_.second);
    if (result2.is_success()) {
      return {true, result2.GetSolution(stage2Ik.q())};
    }
    return {false, Eigen::VectorXd()};
  } catch (const std::exception& e) {
    std::cerr << "Error in SolveStage2: " << e.what() << std::endl;
    return {false, Eigen::VectorXd()};
  }
}

std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::getStage1Result() const { return stage1Result_; }
std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::getStage2Result() const { return stage2Result_; }

void TwoStageTorsoIK::setPlantContext(std::unique_ptr<drake::systems::Context<double>> context) {
  if (!context) {
    std::cerr << "Error: Cannot set null context" << std::endl;
    return;
  }
  plantContext_ = std::move(context);
}

void TwoStageTorsoIK::initializeJointLimits() {
  if (!plant_) {
    hasJointLimits_ = false;
    return;
  }
  try {
    jointLowerBounds_ = Eigen::VectorXd::Zero(nq_);
    jointUpperBounds_ = Eigen::VectorXd::Zero(nq_);
    for (int i = 0; i < nq_; ++i) {
      jointLowerBounds_(i) = -M_PI;
      jointUpperBounds_(i) = M_PI;
    }
    hasJointLimits_ = false;
  } catch (const std::exception& e) {
    std::cerr << "Error initializing joint limits: " << e.what() << std::endl;
    hasJointLimits_ = false;
  }
}

Eigen::VectorXd TwoStageTorsoIK::limitAngle(const Eigen::VectorXd& q) const {
  if (!hasJointLimits_) return q;
  Eigen::VectorXd qLimited = q;
  for (int i = 0; i < q.size() && i < jointLowerBounds_.size() && i < jointUpperBounds_.size(); ++i) {
    qLimited(i) = std::max(jointLowerBounds_(i), std::min(q(i), jointUpperBounds_(i)));
  }
  return qLimited;
}

Eigen::VectorXd TwoStageTorsoIK::limitAngleByVelocity(const Eigen::VectorXd& qLast,
                                                     const Eigen::VectorXd& qNow,
                                                     double velLimit,
                                                     double controllerDt) const {
  if (qLast.size() != qNow.size()) return qNow;
  Eigen::VectorXd qLimited = qNow;
  int size = qNow.size();
  double aglLimit = controllerDt * velLimit * M_PI / 180.0;
  double angleLimit120Deg = controllerDt * 120.0 * M_PI / 180.0;
  int singleArmDof = 7;

  for (int i = 0; i < size; ++i) {
    qLimited(i) = std::max(qLast(i) - aglLimit, std::min(qNow(i), qLast(i) + aglLimit));
    if (i == 0 || i == singleArmDof) {
      qLimited(i) = std::max(qLast(i) - angleLimit120Deg, std::min(qNow(i), qLast(i) + angleLimit120Deg));
    }
  }
  return qLimited;
}

void TwoStageTorsoIK::updateBaseHeightOffset(double) {}
void TwoStageTorsoIK::updateBaseChestOffsetX(double) {}
void TwoStageTorsoIK::updateShoulderWidth(double) {}
void TwoStageTorsoIK::updateUpperArmLength(double) {}
void TwoStageTorsoIK::updateLowerArmLength(double) {}

}  // namespace ik
}  // namespace leju
