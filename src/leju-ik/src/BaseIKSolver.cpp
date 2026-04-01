#include "leju-ik/BaseIKSolver.h"

#include <iostream>
#include <stdexcept>

namespace leju {
namespace ik {

BaseIKSolver::BaseIKSolver(drake::multibody::MultibodyPlant<double>* plant,
                           const std::vector<std::string>& ikConstraintFrameNames,
                           const IKSolverConfig& config)
    : config_(config), plant_(plant), nq_(0), hasJointLimits_(false), hasLatestSolution_(false) {
  if (!plant_) {
    throw std::invalid_argument("Plant pointer is null");
  }
  if (ikConstraintFrameNames.empty()) {
    throw std::invalid_argument("Frame names should not be empty");
  }
  if (!plant_->is_finalized()) {
    plant_->Finalize();
  }
  nq_ = plant_->num_positions();
  latestSolution_ = Eigen::VectorXd::Zero(nq_);
  initializeFrames(ikConstraintFrameNames);
}

void BaseIKSolver::initializeFrames(const std::vector<std::string>& ikConstraintFrameNames) {
  ConstraintFrames_.clear();
  ConstraintFrames_.reserve(ikConstraintFrameNames.size());
  for (const auto& frameName : ikConstraintFrameNames) {
    ConstraintFrames_.push_back(&plant_->GetFrameByName(frameName));
  }
}

bool BaseIKSolver::preSolveCheck(const std::vector<PoseData>& PoseConstraintList) const {
  if (!plant_) return false;
  if (PoseConstraintList.empty()) return false;
  for (size_t i = 0; i < PoseConstraintList.size(); ++i) {
    const auto& pos = PoseConstraintList[i].position;
    if (pos.hasNaN() || !std::isfinite(pos(0)) || !std::isfinite(pos(1)) || !std::isfinite(pos(2)) ||
        (pos.array().abs() > 1e6).any()) {
      return false;
    }
  }
  return true;
}

std::pair<bool, Eigen::VectorXd> BaseIKSolver::solveDrakeIK(drake::multibody::InverseKinematics& ik,
                                                            const Eigen::VectorXd& initialGuess,
                                                            const std::string& /*stageName*/) const {
  drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(ik.prog(), initialGuess);
  if (result.is_success()) {
    return {true, result.GetSolution(ik.q())};
  }
  return {false, Eigen::VectorXd::Zero(nq_)};
}

void BaseIKSolver::initInverseKinematicsSolver(drake::multibody::InverseKinematics& ik,
                                               SolverType solverType) const {
  switch (solverType) {
    case SolverType::SNOPT: {
      drake::solvers::SnoptSolver snopt;
      ik.get_mutable_prog()->SetSolverOption(snopt.solver_id(), "Major Optimality Tolerance",
                                             config_.solverTolerance);
      ik.get_mutable_prog()->SetSolverOption(snopt.solver_id(), "Major Iterations Limit", config_.maxIterations);
      break;
    }
    case SolverType::IPOPT:
    case SolverType::NLOPT:
    case SolverType::OSQP:
    case SolverType::DEFAULT:
      break;
  }
}

void BaseIKSolver::updateLatestSolution(const Eigen::VectorXd& solution) {
  latestSolution_ = solution;
  hasLatestSolution_ = true;
}

Eigen::VectorXd BaseIKSolver::getWarmStartSolution() const {
  if (hasLatestSolution_ && latestSolution_.size() == nq_ && latestSolution_.norm() > 1e-6) {
    return latestSolution_;
  }
  return Eigen::VectorXd::Zero(nq_);
}

}  // namespace ik
}  // namespace leju
