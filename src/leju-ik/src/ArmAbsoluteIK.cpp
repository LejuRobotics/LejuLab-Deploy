/**
 * @file ArmAbsoluteIK.cpp
 * @brief 双臂绝对式IK求解器实现
 *
 * C++ 移植自 torso_ik.py::ArmIk + TorsoIK，针对8自由度手臂（每臂4关节）。
 * Drake IK 约束设置逻辑与 Python 版本的 is_roban_dof=True, as_mc_ik=True 分支一致。
 */

#include "leju-ik/ArmAbsoluteIK.h"

#include <drake/geometry/scene_graph.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/tree/fixed_offset_frame.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/solve.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>

#include <iostream>
#include <stdexcept>

namespace leju {
namespace ik {

// ============================================================
// Impl（隐藏实现细节）
// ============================================================
struct ArmAbsoluteIK::Impl {
  ArmAbsoluteIKConfig config;
  double torso_yaw_rad = 0.0;
  double torso_height = 0.0;

  // Drake 多体系统（由 DiagramBuilder 构建后由 diagram 所有）
  drake::multibody::MultibodyPlant<double>* plant = nullptr;
  std::unique_ptr<drake::systems::Diagram<double>> diagram;
  std::unique_ptr<drake::systems::Context<double>> diagram_context;

  // 常用坐标系（由 plant 所有，diagram 存活期间有效）
  const drake::multibody::Frame<double>* torso_frame = nullptr;
  const drake::multibody::Frame<double>* left_eef_frame = nullptr;
  const drake::multibody::Frame<double>* right_eef_frame = nullptr;
  const drake::multibody::Frame<double>* left_elbow_frame = nullptr;
  const drake::multibody::Frame<double>* right_elbow_frame = nullptr;

  int nq = 0;
  Eigen::VectorXd default_q;

  // 上一帧IK解（用于平滑代价项）
  Eigen::VectorXd last_solution;
  bool has_last_solution = false;

  /// 从 diagram_context 获取 plant 的可变上下文（用于FK）
  drake::systems::Context<double>& plantContext() {
    return plant->GetMyMutableContextFromRoot(diagram_context.get());
  }
};

// ============================================================
// 构造 / 析构
// ============================================================
ArmAbsoluteIK::ArmAbsoluteIK(const std::string& urdf_path,
                               const ArmAbsoluteIKConfig& config)
    : impl_(std::make_unique<Impl>()) {
  impl_->config = config;

  auto builder = std::make_unique<drake::systems::DiagramBuilder<double>>();
  auto [plant_ref, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(builder.get(), 0.0);
  impl_->plant = &plant_ref;

  drake::multibody::Parser parser(impl_->plant);
  parser.AddModels(urdf_path);

  // 与 torso_ik.py::ArmIk 一致：名义末端上始终挂子 frame，手部 IK/FK 均对该子 frame 原点（已含 eef_z_bias；kuavo.json eef_z_offset）约束，而非裸 URDF link
  {
    using drake::math::RigidTransform;
    using drake::multibody::FixedOffsetFrame;
    const drake::multibody::Frame<double>& pL =
        impl_->plant->GetFrameByName(config.left_eef_frame);
    const drake::multibody::Frame<double>& pR =
        impl_->plant->GetFrameByName(config.right_eef_frame);
    const RigidTransform<double> X_bias(
        Eigen::Vector3d(0.0, 0.0, config.eef_z_bias));
      std::cout << "config.eef_z_bias: " << config.eef_z_bias<< std::endl;
    impl_->plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        "frame_eef_left_mc", pL, X_bias));
    impl_->plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        "frame_eef_right_mc", pR, X_bias));
  }

  // 将躯干焊接到世界坐标系原点（与 Python: plant.WeldFrames(world_frame, torso_frame) 对应）
  impl_->plant->WeldFrames(impl_->plant->world_frame(),
                            impl_->plant->GetFrameByName(config.torso_frame));
  impl_->plant->Finalize();

  impl_->diagram = builder->Build();
  impl_->diagram_context = impl_->diagram->CreateDefaultContext();

  // 缓存常用坐标系指针
  impl_->torso_frame =
      &impl_->plant->GetFrameByName(config.torso_frame);
  impl_->left_eef_frame =
      &impl_->plant->GetFrameByName("frame_eef_left_mc");
  impl_->right_eef_frame =
      &impl_->plant->GetFrameByName("frame_eef_right_mc");
  impl_->left_elbow_frame =
      &impl_->plant->GetFrameByName(config.left_elbow_frame);
  impl_->right_elbow_frame =
      &impl_->plant->GetFrameByName(config.right_elbow_frame);

  impl_->nq = impl_->plant->num_positions();

  // 读取默认关节角（全零）
  const auto& plant_ctx = impl_->plant->GetMyContextFromRoot(*impl_->diagram_context);
  impl_->default_q = impl_->plant->GetPositions(plant_ctx);
  impl_->last_solution = impl_->default_q;

  std::cout << "[ArmAbsoluteIK] Initialized. urdf=" << urdf_path
            << "  nq=" << impl_->nq
            << "  eef_z_bias=" << config.eef_z_bias << std::endl;
}

ArmAbsoluteIK::~ArmAbsoluteIK() = default;

// ============================================================
// 公共接口
// ============================================================
void ArmAbsoluteIK::setTorsoState(double torso_yaw_rad, double torso_height) {
  impl_->torso_yaw_rad = torso_yaw_rad;
  impl_->torso_height = torso_height;
}

Eigen::VectorXd ArmAbsoluteIK::computeIK(
    const std::optional<Eigen::Vector3d>& left_hand_pos,
    const std::optional<Eigen::Vector3d>& right_hand_pos,
    const std::optional<Eigen::Vector3d>& left_elbow_pos,
    const std::optional<Eigen::Vector3d>& right_elbow_pos,
    const Eigen::VectorXd& q_init) {

  using drake::multibody::InverseKinematics;
  using drake::math::RotationMatrix;
  using drake::math::RollPitchYaw;

  // ------- 构造IK问题（与 TorsoIK.solve() 中 is_roban_dof=True, as_mc_ik=True 对应）-------
  InverseKinematics ik(*impl_->plant, /*with_joint_limits=*/true);

  const auto snopt_id = drake::solvers::SnoptSolver::id();
  ik.get_mutable_prog()->SetSolverOption(snopt_id, "Major Optimality Tolerance",
                                          impl_->config.solver_tol);
  ik.get_mutable_prog()->SetSolverOption(snopt_id, "Major Iterations Limit",
                                          impl_->config.iterations_limit);

  const auto& world = impl_->plant->world_frame();
  const double tol = impl_->config.constraint_tol;

  // 躯干姿态硬约束（torsoR = [0, torso_yaw, 0]，双足固定躯干时该约束trivially满足）
  ik.AddOrientationConstraint(
      world,
      RotationMatrix<double>(RollPitchYaw<double>(0.0, impl_->torso_yaw_rad, 0.0)),
      *impl_->torso_frame,
      RotationMatrix<double>::Identity(),
      tol);

  // 躯干位置硬约束（r = [0, 0, torso_height]）
  Eigen::Vector3d torso_pos(0.0, 0.0, impl_->torso_height);
  ik.AddPositionConstraint(
      *impl_->torso_frame, Eigen::Vector3d::Zero(),
      world,
      torso_pos - Eigen::Vector3d::Constant(tol),
      torso_pos + Eigen::Vector3d::Constant(tol));

  // 手部位置软代价：frame 为挂接 eef_z_bias 后的 frame_eef_*_mc（p_BQ=0），与 Python ArmIk 自定义末端一致
  const Eigen::Matrix3d W_hand =
      impl_->config.hand_pos_weight * Eigen::Matrix3d::Identity();
  if (left_hand_pos.has_value()) {
    ik.AddPositionCost(world, left_hand_pos.value(),
                       *impl_->left_eef_frame, Eigen::Vector3d::Zero(),
                       W_hand);
  }
  if (right_hand_pos.has_value()) {
    ik.AddPositionCost(world, right_hand_pos.value(),
                       *impl_->right_eef_frame, Eigen::Vector3d::Zero(),
                       W_hand);
  }

  // 肘部位置软代价（is_roban_dof=True 时也用 10*I）
  const Eigen::Matrix3d W_elbow =
      impl_->config.elbow_pos_weight * Eigen::Matrix3d::Identity();
  if (left_elbow_pos.has_value()) {
    ik.AddPositionCost(world, left_elbow_pos.value(),
                       *impl_->left_elbow_frame, Eigen::Vector3d::Zero(),
                       W_elbow);
  }
  if (right_elbow_pos.has_value()) {
    ik.AddPositionCost(world, right_elbow_pos.value(),
                       *impl_->right_elbow_frame, Eigen::Vector3d::Zero(),
                       W_elbow);
  }

  // 关节平滑代价：AddQuadraticErrorCost(0.1*I, last_solution, q)
  if (impl_->has_last_solution) {
    Eigen::MatrixXd W_smooth =
        Eigen::MatrixXd::Identity(impl_->nq, impl_->nq) *
        impl_->config.smooth_weight;
    ik.get_mutable_prog()->AddQuadraticErrorCost(W_smooth, impl_->last_solution, ik.q());
  }

  // 初始猜测：优先用传入的 q_init，否则用上一帧解，最后用默认值
  Eigen::VectorXd q0;
  if (q_init.size() == impl_->nq) {
    q0 = q_init;
  } else if (impl_->has_last_solution) {
    q0 = impl_->last_solution;
  } else {
    q0 = impl_->default_q;
  }

  // ------- 求解 -------
  const auto result = drake::solvers::Solve(ik.prog(), q0);

  if (!result.is_success()) {
    return {};  // 空向量表示求解失败
  }

  Eigen::VectorXd q_sol = result.GetSolution(ik.q());
  impl_->last_solution = q_sol;
  impl_->has_last_solution = true;
  return q_sol;
}

Eigen::VectorXd ArmAbsoluteIK::defaultPositions() const {
  return impl_->default_q;
}

int ArmAbsoluteIK::numPositions() const {
  return impl_->nq;
}

void ArmAbsoluteIK::resetLastSolution() {
  impl_->last_solution = impl_->default_q;
  impl_->has_last_solution = false;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> ArmAbsoluteIK::getLeftHandPose(
    const Eigen::VectorXd& q) {
  auto& plant_ctx = impl_->plantContext();
  impl_->plant->SetPositions(&plant_ctx, q);
  const auto X = impl_->left_eef_frame->CalcPose(plant_ctx, *impl_->torso_frame);
  const auto rpy = drake::math::RollPitchYaw<double>(X.rotation()).vector();
  return {X.translation(), rpy};
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> ArmAbsoluteIK::getRightHandPose(
    const Eigen::VectorXd& q) {
  auto& plant_ctx = impl_->plantContext();
  impl_->plant->SetPositions(&plant_ctx, q);
  const auto X = impl_->right_eef_frame->CalcPose(plant_ctx, *impl_->torso_frame);
  const auto rpy = drake::math::RollPitchYaw<double>(X.rotation()).vector();
  return {X.translation(), rpy};
}

}  // namespace ik
}  // namespace leju
