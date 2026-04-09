#pragma once

#include <drake/common/symbolic/expression.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>

#include <Eigen/Dense>

#include <stdexcept>
#include <utility>

namespace leju {
namespace ik {

struct DrakeVelocityIKWeightConfig {
  const char* name;
  double q11;
  double q12;
  double q2;
  double qv1;
  double qv2;
};

namespace DrakeVelocityIKWeights {
inline constexpr DrakeVelocityIKWeightConfig PureTracking{"PureTracking", 0.2, 0.8, 1.0, 0.0, 0.0};
inline constexpr DrakeVelocityIKWeightConfig Balanced{"Balanced", 0.2, 0.8, 1.0, 0.1, 0.1};
inline constexpr DrakeVelocityIKWeightConfig HandPriority{"HandPriority", 0.5, 0.5, 500.0, 0.05, 0.001};
inline constexpr DrakeVelocityIKWeightConfig ElbowPriority{"ElbowPriority", 2.0, 8.0, 1.0, 0.1, 0.05};
inline constexpr DrakeVelocityIKWeightConfig LightSmooth{"LightSmooth", 0.4, 3.6, 2.0, 0.02, 0.02};
inline constexpr DrakeVelocityIKWeightConfig Aggressive{"Aggressive", 2.0, 8.0, 10.0, 0.01, 0.01};
}  // namespace DrakeVelocityIKWeights

class DrakeVelocityIKSolver final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit DrakeVelocityIKSolver(const Eigen::Vector3d& p0,
                                 double link1Length,
                                 double link2Length,
                                 const Eigen::Vector3d& p2LowerBound,
                                 const Eigen::Vector3d& p2UpperBound)
      : p0_(p0),
        link1Length_(link1Length),
        link2Length_(link2Length),
        p2LowerBound_(p2LowerBound),
        p2UpperBound_(p2UpperBound) {
    if (!(link1Length_ > 0.0) || !(link2Length_ > 0.0)) {
      throw std::invalid_argument("DrakeVelocityIKSolver: link lengths must be > 0.");
    }

    Q11_.setIdentity();
    Q12_.setIdentity();
    Q2_.setIdentity();
    Qv1_.setIdentity();
    Qv2_.setIdentity();

    u1Prev_ = Eigen::Vector3d::UnitX();
    u2Prev_ = Eigen::Vector3d::UnitX();
  }

  ~DrakeVelocityIKSolver() = default;

  void setP2Bounds(const Eigen::Vector3d& lb, const Eigen::Vector3d& ub) {
    p2LowerBound_ = lb;
    p2UpperBound_ = ub;
  }

  void setWeights(const Eigen::Matrix3d& Q11,
                  const Eigen::Matrix3d& Q12,
                  const Eigen::Matrix3d& Q2,
                  const Eigen::Matrix3d& Qv1,
                  const Eigen::Matrix3d& Qv2) {
    Q11_ = Q11;
    Q12_ = Q12;
    Q2_ = Q2;
    Qv1_ = Qv1;
    Qv2_ = Qv2;
  }

  void setWeights(const DrakeVelocityIKWeightConfig& config) {
    setWeights(config.q11 * Eigen::Matrix3d::Identity(),
               config.q12 * Eigen::Matrix3d::Identity(),
               config.q2 * Eigen::Matrix3d::Identity(),
               config.qv1 * Eigen::Matrix3d::Identity(),
               config.qv2 * Eigen::Matrix3d::Identity());
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> solve(const Eigen::Vector3d& p1Ref,
                                                    const Eigen::Vector3d& p1Fixed,
                                                    const Eigen::Vector3d& p2Ref) {
    using drake::symbolic::Expression;

    drake::solvers::MathematicalProgram prog;

    const auto u1 = prog.NewContinuousVariables(3, "u1");
    const auto u2 = prog.NewContinuousVariables(3, "u2");
    const auto p1 = prog.NewContinuousVariables(3, "p1");
    const auto p2 = prog.NewContinuousVariables(3, "p2");

    prog.AddBoundingBoxConstraint(p2LowerBound_, p2UpperBound_, p2);

    constexpr double kMinP2XyNorm = 0.2;
    constexpr double kMinP2XyNormSquared = kMinP2XyNorm * kMinP2XyNorm;
    const Expression p2XyNormSquared = p2(0) * p2(0) + 0.81 * p2(1) * p2(1);
    prog.AddConstraint(p2XyNormSquared >= kMinP2XyNormSquared);

    constexpr double kMinP1YSquared = 0.22 * 0.22;
    const Expression p1YSquared = p1(1) * p1(1);
    prog.AddConstraint(p1YSquared >= kMinP1YSquared);

    const Expression u1DotU1 = u1(0) * u1(0) + u1(1) * u1(1) + u1(2) * u1(2);
    const Expression u2DotU2 = u2(0) * u2(0) + u2(1) * u2(1) + u2(2) * u2(2);
    prog.AddConstraint(u1DotU1 == 1.0);
    prog.AddConstraint(u2DotU2 == 1.0);

    const Eigen::Matrix<Expression, 3, 1> fkP1 =
        p0_.cast<Expression>() + link1Length_ * u1.cast<Expression>();
    const Eigen::Matrix<Expression, 3, 1> fkP2 =
        p0_.cast<Expression>() + link1Length_ * u1.cast<Expression>() +
        link2Length_ * u2.cast<Expression>();
    prog.AddLinearEqualityConstraint(p1.cast<Expression>() - fkP1, Eigen::Vector3d::Zero());
    prog.AddLinearEqualityConstraint(p2.cast<Expression>() - fkP2, Eigen::Vector3d::Zero());

    const Eigen::Vector3d p1Prev = p0_ + link1Length_ * u1Prev_;
    const Eigen::Vector3d p2Prev = p0_ + link1Length_ * u1Prev_ + link2Length_ * u2Prev_;

    prog.AddQuadraticErrorCost(Q11_, p1Fixed, p1);
    prog.AddQuadraticErrorCost(Q12_, p1Ref, p1);
    prog.AddQuadraticErrorCost(Q2_, p2Ref, p2);
    prog.AddQuadraticErrorCost(Qv1_, p1Prev, p1);
    prog.AddQuadraticErrorCost(Qv2_, p2Prev, p2);

    prog.SetInitialGuess(u1, u1Prev_);
    prog.SetInitialGuess(u2, u2Prev_);
    prog.SetInitialGuess(p1, p1Prev);
    prog.SetInitialGuess(p2, p2Prev);

    const auto result = drake::solvers::Solve(prog);
    if (!result.is_success()) {
      return {p1Prev, p2Prev};
    }

    Eigen::Vector3d u1Sol = result.GetSolution(u1);
    Eigen::Vector3d u2Sol = result.GetSolution(u2);
    const double u1Norm = u1Sol.norm();
    const double u2Norm = u2Sol.norm();
    if (u1Norm > 0.0) {
      u1Sol /= u1Norm;
    }
    if (u2Norm > 0.0) {
      u2Sol /= u2Norm;
    }

    u1Prev_ = u1Sol;
    u2Prev_ = u2Sol;

    return {result.GetSolution(p1), result.GetSolution(p2)};
  }

 private:
  Eigen::Vector3d p0_;
  double link1Length_ = 0.0;
  double link2Length_ = 0.0;
  Eigen::Vector3d p2LowerBound_ = -Eigen::Vector3d::Ones();
  Eigen::Vector3d p2UpperBound_ = Eigen::Vector3d::Ones();

  Eigen::Matrix3d Q11_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Q12_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Q2_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Qv1_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Qv2_ = Eigen::Matrix3d::Identity();

  Eigen::Vector3d u1Prev_ = Eigen::Vector3d::UnitX();
  Eigen::Vector3d u2Prev_ = Eigen::Vector3d::UnitX();
};

}  // namespace ik
}  // namespace leju
