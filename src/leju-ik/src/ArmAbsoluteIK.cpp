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
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/solve.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>

#include <algorithm>
#include <cmath>
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

  // 肩 pitch 关节原点在躯干系中的位置（构造时读 URDF；躯干焊接到世界原点时即世界位置）
  Eigen::Vector3d left_shoulder_offset = Eigen::Vector3d::Zero();
  Eigen::Vector3d right_shoulder_offset = Eigen::Vector3d::Zero();
  // 肩 roll 轴的固定预倾角 tilt（构造时从 zarm_l2/r2 joint origin rpy.x 读取，
  // 用于翻转分支闭式解 q2 = asin(d_y) - tilt；各机型不同——如 s17=±0.17453、s46=0——不能硬编码）
  double left_shoulder_tilt = 0.0;
  double right_shoulder_tilt = 0.0;
  // 上抬过肩"翻转分支"激活态（滞回，避免边界抖动）
  bool l_flip_active = false;
  bool r_flip_active = false;
  // "返回"激活态：翻转释放后引导肩回到常规分支(q1→0)，防止下放时卡在翻转分支翻不回来
  bool l_return_active = false;
  bool r_return_active = false;
  // q1 引导目标的时间平滑（EMA）：让翻转/返回在数帧内渐变，避免目标在肩部水平(90°)附近
  // 波动时 q1 在 0↔-π 间剧烈跳变（q2 目标连续，无需平滑）
  double l_q1_steer = 0.0;
  double r_q1_steer = 0.0;

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
  parser.package_map().PopulateFromRosPackagePath();
  // 从 URDF 路径自动推导 models 目录并注册 biped_sXX 包，无需手动设 ROS_PACKAGE_PATH
  // URDF: <repo>/src/leju_assets/models/biped_sXX/urdf/drake/biped_v3_arm.urdf
  {
    std::string models_dir = urdf_path;
    for (int i = 0; i < 4; ++i) {
      auto pos = models_dir.rfind('/');
      if (pos != std::string::npos) models_dir = models_dir.substr(0, pos);
    }
    parser.package_map().PopulateFromFolder(models_dir);
  }
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

  // 与 Python ArmIk._apply_shoulder_pitch_upper_limit 一致：Finalize 前收紧左右肩
  for (const char* joint_name : {"zarm_l1_joint", "zarm_r1_joint"}) {
    auto& joint = impl_->plant->GetMutableJointByName(joint_name);
    Eigen::VectorXd lower = joint.position_lower_limits();
    Eigen::VectorXd upper = joint.position_upper_limits();
    upper[0] = std::min(upper[0]+0.05, 0.3);
    joint.set_position_limits(lower, upper);
  }
  // 收紧左右肩 roll（|q2|≤1.4）：排除手臂上摆越过肩部水平后的深弯盆地（q2>1.4 的弯曲解），
  // 与翻转分支不冲突（翻转分支闭式解 q2 恒小）。避免 SNOPT 落入 q2 过大的弯曲局部极小。
  {
    auto& joint = impl_->plant->GetMutableJointByName("zarm_l2_joint");
    Eigen::VectorXd lower = joint.position_lower_limits();
    Eigen::VectorXd upper = joint.position_upper_limits();
    upper[0] = std::min(upper[0], 1.4);
    joint.set_position_limits(lower, upper);
    auto& joint_r = impl_->plant->GetMutableJointByName("zarm_r2_joint");
    Eigen::VectorXd lower_r = joint_r.position_lower_limits();
    Eigen::VectorXd upper_r = joint_r.position_upper_limits();
    lower_r[0] = std::max(lower_r[0], -1.4);
    joint_r.set_position_limits(lower_r, upper_r);
  }

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

  // 读肩 pitch 关节原点。
  impl_->left_shoulder_offset =
      impl_->plant->GetJointByName("zarm_l1_joint").frame_on_child()
          .CalcPoseInWorld(plant_ctx)
          .translation();
  impl_->right_shoulder_offset =
      impl_->plant->GetJointByName("zarm_r1_joint").frame_on_child()
          .CalcPoseInWorld(plant_ctx)
          .translation();

  // 读肩 roll 轴预倾角
  impl_->left_shoulder_tilt =
      impl_->plant->GetJointByName("zarm_l2_joint").frame_on_parent()
          .CalcPoseInWorld(plant_ctx).rotation().ToRollPitchYaw().roll_angle();
  impl_->right_shoulder_tilt =
      impl_->plant->GetJointByName("zarm_r2_joint").frame_on_parent()
          .CalcPoseInWorld(plant_ctx).rotation().ToRollPitchYaw().roll_angle();

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

  // 肩关节 pitch 上限（zarm_l1/r1 → 0.3rad）已在构造函数 Finalize 前通过收紧
  // ---- 肩 pitch 180° 翻转引导：目标上抬过肩时引导 q1→-π，使 q2 恒 ∈ (-π/2, π/2) ----
  // 上臂方向（肩→肘单位向量，躯干系）：d = R_y(q1)·R_x(tilt+q2)·(0,0,-1)
  //   = (-sin q1·cosθ, sinθ, -cos q1·cosθ)，θ = tilt + q2（tilt 为肩 roll 轴预倾角，构造时从 plant 读取）。
  // 过顶有两条分支：q1≈0, q2≈π（roll 绕满，q2>π/2，被 ±1.5 限位排除）与 q1≈-π, q2≈0
  // （pitch 翻转 180°，本方案目标）。翻转分支闭式解：q2 = asin(d_y) - tilt，
  // q1 = atan2(-d_x, -d_z)（折入收紧后的 pitch 区间）。
  // SNOPT 是局部求解器，仅收紧 q2 限位无法跨过肩部奇异点，必须用翻转分支的初始猜测 + 软引导代价。
  using drake::multibody::RevoluteJoint;
  const int l1_idx =
      impl_->plant->GetJointByName<RevoluteJoint>("zarm_l1_joint").position_start();
  const int l2_idx =
      impl_->plant->GetJointByName<RevoluteJoint>("zarm_l2_joint").position_start();
  const int r1_idx =
      impl_->plant->GetJointByName<RevoluteJoint>("zarm_r1_joint").position_start();
  const int r2_idx =
      impl_->plant->GetJointByName<RevoluteJoint>("zarm_r2_joint").position_start();

  // 收紧后的各臂关节限位（从 plant 读取，避免硬编码机型差异）
  const auto& jl1 = impl_->plant->GetJointByName("zarm_l1_joint");
  const auto& jl2 = impl_->plant->GetJointByName("zarm_l2_joint");
  const auto& jr1 = impl_->plant->GetJointByName("zarm_r1_joint");
  const auto& jr2 = impl_->plant->GetJointByName("zarm_r2_joint");
  const double l1_lo = jl1.position_lower_limits()[0], l1_hi = jl1.position_upper_limits()[0];
  const double l2_lo = jl2.position_lower_limits()[0], l2_hi = jl2.position_upper_limits()[0];
  const double r1_lo = jr1.position_lower_limits()[0], r1_hi = jr1.position_upper_limits()[0];
  const double r2_lo = jr2.position_lower_limits()[0], r2_hi = jr2.position_upper_limits()[0];

  // 肩 roll 轴预倾角（构造时从 plant 读取，适配各机型，如 s17=±0.17453、s46=0）
  const double tilt_l = impl_->left_shoulder_tilt;
  const double tilt_r = impl_->right_shoulder_tilt;
  constexpr double kFlipActivateZ = 0.08;     // 上抬过肩激活阈值
  constexpr double kFlipReleaseZ = 0.05;      // 下放回落释放阈值
  constexpr double kFlipActivateZHand = 0.30; // 无肘目标、仅用手目标兜底时的更高阈值
  constexpr double kFlipReleaseZHand = 0.15;
  constexpr double kFlipMinDist = 0.05;       // 目标过近时方向病态，不触发
  constexpr double kFlipSteerMaxStep = 0.35;  // q1 引导目标每帧最大变化(rad)：
                                              // 翻转/返回在 ~π/α≈9 帧(90ms@100Hz)内渐变，
                                              // 抑制目标在肩部水平附近波动时的剧烈甩动
  constexpr double kReturnSteerWeight = 5.0;  // 返回分支 q1 引导权重：肩下位置代价在翻转分支
                                              // (q1≈-π)有局部极小，权重须足够大才能强制 SNOPT
                                              // 逃出（否则"手臂朝上"翻不回来）
  constexpr double kNearShoulderZ = 0.15;      // 近肩水平引导带：|d_z|<该值且未处于翻转/返回引导时，
                                               // 用软代价把 q1 拉向肘对齐解，保证双臂都落进伸直盆地
  constexpr double kNearShoulderQ1Weight = 2.0;  // 近肩水平 q1 引导权重（≈1~2 即足以克服 0.1 平滑代价）

  // 分支闭式肩解：d_z>0 用翻转分支(q1 折入 [-π,-π/2])，d_z<0 用常规分支(q1 ∈ (-π/2,π/2))，返回引导用常规分支把肩拉回 q1≈0，防止下放时卡在翻转分支翻不回来。
  const auto solveFlipBranch = [](const Eigen::Vector3d& d, double tilt,
                                  bool flip_branch,
                                  double q1_lo, double q1_hi,
                                  double q2_lo, double q2_hi,
                                  double& q1, double& q2) {
    const double dy = std::clamp(d.y(), -1.0 + 1e-6, 1.0 - 1e-6);
    q2 = std::asin(dy) - tilt;
    q1 = std::atan2(-d.x(), -d.z());
    if (flip_branch && q1 > q1_hi) q1 -= 2.0 * M_PI;  // 仅翻转分支折入收紧后区间
    q1 = std::clamp(q1, q1_lo, q1_hi);
    q2 = std::clamp(q2, q2_lo, q2_hi);
  };

  //  flip翻转状态机：
  //   flip(引导翻转分支) ↔ return(引导常规分支) ↔ 无。
  //   上抬 d_z ≥ z_act → flip；下放 d_z < z_rel → flip→return；
  //   手臂已回常规分支(肩 pitch > -π/2) → return→无；又上抬 → return→flip。
  const Eigen::Vector3d* l_target =
      left_elbow_pos.has_value() ? &left_elbow_pos.value()
      : left_hand_pos.has_value() ? &left_hand_pos.value() : nullptr;
  const Eigen::Vector3d* r_target =
      right_elbow_pos.has_value() ? &right_elbow_pos.value()
      : right_hand_pos.has_value() ? &right_hand_pos.value() : nullptr;

  int l_mode = 0, r_mode = 0;  // 0=不引导, 1=翻转分支, 2=常规分支(返回翻转)
  double l_q1 = 0.0, l_q2 = 0.0, r_q1 = 0.0, r_q2 = 0.0;
  const auto updateFlip =
      [&](const Eigen::Vector3d* target, const Eigen::Vector3d& shoulder,
          double tilt, double z_act, double z_rel,
          double q1_lo, double q1_hi, double q2_lo, double q2_hi,
          double current_q1,
          bool& flip_active, bool& return_active,
          double& q1, double& q2) -> int {
    if (target == nullptr) { flip_active = false; return_active = false; return 0; }
    const Eigen::Vector3d v = *target - shoulder;
    const double dist = v.norm();
    if (dist < kFlipMinDist) { flip_active = false; return_active = false; return 0; }
    const Eigen::Vector3d d = v / dist;
    if (flip_active) {
      if (d.z() < z_rel) { flip_active = false; return_active = true; }
    } else if (return_active) {
      if (current_q1 > -M_PI_2) { return_active = false; }                    // 已回常规分支
      else if (d.z() > z_act) { return_active = false; flip_active = true; }  // 又上抬过肩
    } else {
      if (d.z() >= z_act) flip_active = true;
    }
    if (flip_active) {
      solveFlipBranch(d, tilt, /*flip_branch=*/true, q1_lo, q1_hi, q2_lo, q2_hi, q1, q2);
      return 1;
    }
    if (return_active) {
      solveFlipBranch(d, tilt, /*flip_branch=*/false, q1_lo, q1_hi, q2_lo, q2_hi, q1, q2);
      return 2;
    }
    return 0;
  };

  // 肩 pitch 关节原点（躯干焊接到世界原点，故躯干系偏移即世界位置；torso_yaw/height 被焊接钳为 0）
  const Eigen::Vector3d l_shoulder = impl_->left_shoulder_offset;
  const Eigen::Vector3d r_shoulder = impl_->right_shoulder_offset;
  l_mode = updateFlip(l_target, l_shoulder, tilt_l,
                      left_elbow_pos.has_value() ? kFlipActivateZ : kFlipActivateZHand,
                      left_elbow_pos.has_value() ? kFlipReleaseZ : kFlipReleaseZHand,
                      l1_lo, l1_hi, l2_lo, l2_hi,
                      impl_->last_solution(l1_idx),  // 上一帧肩 pitch，判定是否已回常规分支
                      impl_->l_flip_active, impl_->l_return_active, l_q1, l_q2);
  r_mode = updateFlip(r_target, r_shoulder, tilt_r,
                      right_elbow_pos.has_value() ? kFlipActivateZ : kFlipActivateZHand,
                      right_elbow_pos.has_value() ? kFlipReleaseZ : kFlipReleaseZHand,
                      r1_lo, r1_hi, r2_lo, r2_hi,
                      impl_->last_solution(r1_idx),
                      impl_->r_flip_active, impl_->r_return_active, r_q1, r_q2);

  // q1 引导目标限速率平滑
  if (l_mode != 0) {
    const double dl = l_q1 - impl_->l_q1_steer;
    impl_->l_q1_steer += std::clamp(dl, -kFlipSteerMaxStep, kFlipSteerMaxStep);
  } else {
    impl_->l_q1_steer = impl_->last_solution(l1_idx);
  }
  if (r_mode != 0) {
    const double dr = r_q1 - impl_->r_q1_steer;
    impl_->r_q1_steer += std::clamp(dr, -kFlipSteerMaxStep, kFlipSteerMaxStep);
  } else {
    impl_->r_q1_steer = impl_->last_solution(r1_idx);
  }
  const double l_q1_use = impl_->l_q1_steer;  // 限速率平滑后的引导目标
  const double r_q1_use = impl_->r_q1_steer;

  // 肩下硬约束：目标方向低于肩部(d_z < 翻转释放阈值)时，强制 q1 ≥ -π/2，把翻转分支(q1≈-π)从可行域排除。
  {
    const double l_z_rel = left_elbow_pos.has_value() ? kFlipReleaseZ : kFlipReleaseZHand;
    const double r_z_rel = right_elbow_pos.has_value() ? kFlipReleaseZ : kFlipReleaseZHand;
    const auto addReturnBound = [&](const Eigen::Vector3d* target,
                                    const Eigen::Vector3d& shoulder,
                                    double z_rel, double q1_hi, int q1_idx) {
      if (target == nullptr) return;
      const Eigen::Vector3d v = *target - shoulder;
      const double dist = v.norm();
      if (dist < kFlipMinDist) return;
      const Eigen::Vector3d d = v / dist;
      if (d.z() < z_rel) {
        ik.get_mutable_prog()->AddBoundingBoxConstraint(
            -M_PI_2, q1_hi, ik.q()(q1_idx));
      }
    };
    addReturnBound(l_target, l_shoulder, l_z_rel, l1_hi, l1_idx);
    addReturnBound(r_target, r_shoulder, r_z_rel, r1_hi, r1_idx);
  }

  // 分支软引导代价：把肩关节拉向闭式解（翻转分支或常规分支）。
  if (impl_->config.shoulder_flip_weight > 0.0) {
    if (l_mode != 0) {
      const auto dl1 = ik.q()(l1_idx) - l_q1_use;
      if (l_mode == 1) {
        const auto dl2 = ik.q()(l2_idx) - l_q2;
        ik.get_mutable_prog()->AddQuadraticCost(
            (dl1 * dl1 + dl2 * dl2) * impl_->config.shoulder_flip_weight);
      } else {
        ik.get_mutable_prog()->AddQuadraticCost(dl1 * dl1 * kReturnSteerWeight);
      }
    }
    if (r_mode != 0) {
      const auto dr1 = ik.q()(r1_idx) - r_q1_use;
      if (r_mode == 1) {
        const auto dr2 = ik.q()(r2_idx) - r_q2;
        ik.get_mutable_prog()->AddQuadraticCost(
            (dr1 * dr1 + dr2 * dr2) * impl_->config.shoulder_flip_weight);
      } else {
        ik.get_mutable_prog()->AddQuadraticCost(dr1 * dr1 * kReturnSteerWeight);
      }
    }
    // 近肩水平 q1 引导：目标方向接近肩平齐且未处于翻转/返回引导时，把肩 pitch 拉向肘对齐解
    // q1 = clamp(atan2(-d_x, -d_z), -π/2, q1_hi)。作用：辅助肘目标略前伸(d_x>0)时，精确伸直解
    // q1≈-1.63 落在翻转区，被肩下硬约束(q1≥-π/2)排除，剩下"伸直(q1=-π/2)/弯曲(q1≈0)"两个局部极小，SNOPT 可能落入弯曲盆地(q1≈0)而非伸直盆地(q1=-π/2)。该软代价把肩拉向肘对齐解，保证双臂都落进伸直盆地。
    const auto addNearShoulderCost = [&](const Eigen::Vector3d* target,
                                         const Eigen::Vector3d& shoulder,
                                         int mode, double q1_hi, int q1_idx) {
      if (target == nullptr || mode != 0) return;
      const Eigen::Vector3d v = *target - shoulder;
      const double dist = v.norm();
      if (dist < kFlipMinDist) return;
      const Eigen::Vector3d d = v / dist;
      if (std::abs(d.z()) > kNearShoulderZ) return;
      const double q1_near = std::clamp(std::atan2(-d.x(), -d.z()), -M_PI_2, q1_hi);
      const auto dq1 = ik.q()(q1_idx) - q1_near;
      ik.get_mutable_prog()->AddQuadraticCost(dq1 * dq1 * kNearShoulderQ1Weight);
    };
    addNearShoulderCost(l_target, l_shoulder, l_mode, l1_hi, l1_idx);
    addNearShoulderCost(r_target, r_shoulder, r_mode, r1_hi, r1_idx);
  }

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

  // 关节平滑代价：与 Python TorsoIK.solve() 一致——last_solution 总是存在（初值=默认零位），
  // 因此始终加入；roban(nq=8) 时左右臂 yaw 关节（索引2/6）权重置零以允许快速调整。
  const int l_arm_yaw =  impl_->plant->GetJointByName("zarm_l2_joint").position_start();
  const int r_arm_yaw =  impl_->plant->GetJointByName("zarm_r2_joint").position_start();
  Eigen::MatrixXd W_smooth =
      Eigen::MatrixXd::Identity(impl_->nq, impl_->nq) *
      impl_->config.smooth_weight;
  if (impl_->nq == 8) {
    W_smooth(l_arm_yaw, l_arm_yaw) = 0.0;
    W_smooth(r_arm_yaw, r_arm_yaw) = 0.0;
  }
  ik.get_mutable_prog()->AddQuadraticErrorCost(W_smooth, impl_->last_solution, ik.q());

  Eigen::VectorXd q0 = impl_->default_q;
  if (q_init.size() == impl_->nq) {
    q0 = q_init;
  }
  // 分支引导：把肩关节初始猜测放到平滑后的引导目标（翻转分支 q1→-π 或返回分支 q1→0），
  // 其余关节保持 q_init 以维持平滑。
  // 注意：仅翻转分支(mode=1)的种子覆盖 q2（帮助落入翻转盆地）；返回分支(mode=2)的种子
  // 保留 q_init 的实际 q2——返回目标 q2 与手臂实际 q2 可能相差大（如 1.4→0.87），若种子跳变
  // q2 会让 SNOPT 从"手臂朝上"的错误盆地出发，下落到翻转分支的局部极小（q1≈-π）翻不回来。
  if (l_mode != 0) {
    q0(l1_idx) = l_q1_use;
    if (l_mode == 1) q0(l2_idx) = l_q2;
  }
  if (r_mode != 0) {
    q0(r1_idx) = r_q1_use;
    if (r_mode == 1) q0(r2_idx) = r_q2;
  }

  // ------- 求解 -------
  const auto result = drake::solvers::Solve(ik.prog(), q0);

  if (!result.is_success()) {
    return {};  // 空向量表示求解失败
  }

  Eigen::VectorXd q_sol = result.GetSolution(ik.q());
  impl_->last_solution = q_sol;
  return q_sol;
}

Eigen::VectorXd ArmAbsoluteIK::defaultPositions() const {
  return impl_->default_q;
}

int ArmAbsoluteIK::numPositions() const {
  return impl_->nq;
}

void ArmAbsoluteIK::resetLastSolution() {
  // 与 Python ArmIk.reset_last_solution(q=None) 一致：回退到默认零位，平滑代价保持生效；
  // 同时清空翻转/返回分支激活态与引导目标平滑态（节点退出/进入外部控制时重置，避免残留翻转）。
  impl_->last_solution = impl_->default_q;
  impl_->l_flip_active = false;
  impl_->r_flip_active = false;
  impl_->l_return_active = false;
  impl_->r_return_active = false;
  impl_->l_q1_steer = 0.0;
  impl_->r_q1_steer = 0.0;
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
