#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <drake/geometry/scene_graph.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/solve.h>
#include <drake/systems/framework/context.h>

#include <Eigen/Dense>

#include "leju-ik/BaseIKSolver.h"
#include "leju-ik/ik_types.h"

namespace leju {
namespace ik {

class IKResultHistoryBuffer {
 public:
  explicit IKResultHistoryBuffer(size_t maxSize = 15) : maxSize_(maxSize) {}

  void add(const IKSolveResult& result) {
    if (result.isSuccess) {
      buffer_.push_back(result);
      if (buffer_.size() > maxSize_) buffer_.pop_front();
    }
  }
  void setMaxSize(size_t size) {
    maxSize_ = size;
    while (buffer_.size() > maxSize_) buffer_.pop_front();
  }
  size_t size() const { return buffer_.size(); }
  bool empty() const { return buffer_.empty(); }
  void clear() { buffer_.clear(); }
  Eigen::VectorXd getMeanSolution(int nq) const {
    if (buffer_.empty()) return Eigen::VectorXd::Zero(nq);
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(nq);
    for (const auto& r : buffer_) {
      if (r.isSuccess && r.solution.size() == nq) mean += r.solution;
    }
    if (buffer_.size() > 0) mean /= static_cast<double>(buffer_.size());
    return mean;
  }

 private:
  std::deque<IKSolveResult> buffer_;
  size_t maxSize_;
};

class OneStageIKEndEffector : public BaseIKSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit OneStageIKEndEffector(drake::multibody::MultibodyPlant<double>* plant,
                                 const std::vector<std::string>& ikConstraintFrameNames,
                                 const IKSolverConfig& config,
                                 size_t historyBufferSize = 10);

  explicit OneStageIKEndEffector(drake::multibody::MultibodyPlant<double>* plant,
                                 const std::vector<std::string>& ikConstraintFrameNames,
                                 const PointTrackIKSolverConfig& config);

  IKSolveResult solveIK(const std::vector<PoseData>& PoseConstraintList,
                        ArmIdx controlArmIndex = ArmIdx::LEFT,
                        const Eigen::VectorXd& jointMidValues = Eigen::VectorXd()) override;

  std::pair<Eigen::Vector3d, Eigen::Quaterniond> FK(const Eigen::VectorXd& q,
                                                      const std::string& frameName,
                                                      int expectedSize = -1);

  Eigen::VectorXd getMeanSolution() const { return historyBuffer_.getMeanSolution(nq_); }

 private:
  void setConstraints(drake::multibody::InverseKinematics& ik,
                      const std::vector<PoseData>& PoseConstraintList,
                      ArmIdx controlArmIndex,
                      const Eigen::VectorXd& initialGuess,
                      const Eigen::VectorXd& referenceSolution) const;

  std::unique_ptr<drake::systems::Context<double>> plant_context_;
  mutable Eigen::VectorXd currentReferenceSolution_;
  IKResultHistoryBuffer historyBuffer_;
  std::unique_ptr<PointTrackIKSolverConfig> pointTrackConfig_;
};

}  // namespace ik
}  // namespace leju
