/**
 * @file ArmAbsoluteIK.h
 * @brief 双臂绝对式IK求解器 - C++移植自 torso_ik.py (ArmIk/TorsoIK 类)
 *
 * 针对 biped_s17 等8自由度手臂模型（每臂4关节），
 * 将躯干焊接到世界坐标系，对手部和肘部位置使用软代价函数求解逆运动学。
 */

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace leju {
namespace ik {

/// ArmAbsoluteIK 配置参数
struct ArmAbsoluteIKConfig {
  double constraint_tol = 9e-3;   ///< 躯干硬约束容差
  double solver_tol = 5e-2;       ///< SNOPT 主优化容差
  int iterations_limit = 100;     ///< SNOPT 最大迭代次数
  /// 在 URDF 名义末端（left_eef_frame / right_eef_frame）局部 z 上叠加的平移（米），与 torso_ik.py::ArmIk 的 eef_z_bias 一致；IK/FK 均在挂接后的子 frame（frame_eef_*_mc）上计算
  double eef_z_bias = 0.0;

  double hand_pos_weight = 10.0;  ///< 手部位置软代价权重
  double elbow_pos_weight = 10.0; ///< 肘部位置软代价权重
  double smooth_weight = 0.1;     ///< 关节平滑代价权重（与上一帧解的二次误差）

  // URDF 中名义末端 link 的 frame 名（在其上再挂 frame_eef_*_mc 施加 eef_z_bias）
  std::string torso_frame = "torso";
  std::string left_eef_frame = "zarm_l4_link"; // 对于roban是zarm_l4_link
  std::string right_eef_frame = "zarm_r4_link";
  std::string left_elbow_frame = "zarm_l4_link";
  std::string right_elbow_frame = "zarm_r4_link";
};

/**
 * @brief 双臂绝对式IK求解器
 *
 * 加载手臂 URDF，将躯干焊接到世界坐标系，通过 Drake 的 InverseKinematics +
 * SNOPT 求解器对目标手部/肘部位置进行IK求解。
 *
 * 对应 Python 实现中的 ArmIk + TorsoIK 类（is_roban_dof=True 分支，即8自由度情形）。
 */
class ArmAbsoluteIK {
 public:
  /**
   * @brief 构造函数：加载URDF并初始化Drake多体系统
   * @param urdf_path  手臂模型 URDF 路径（如 biped_v3_arm.urdf）
   * @param config     IK 求解参数配置
   */
  explicit ArmAbsoluteIK(const std::string& urdf_path,
                         const ArmAbsoluteIKConfig& config = {});
  ~ArmAbsoluteIK();

  ArmAbsoluteIK(const ArmAbsoluteIK&) = delete;
  ArmAbsoluteIK& operator=(const ArmAbsoluteIK&) = delete;

  /**
   * @brief 设置躯干状态
   * @param torso_yaw_rad  躯干绕 Y 轴的偏航角（弧度），对应 torso_ik.py 中 torsoR[1]
   * @param torso_height   躯干在世界坐标系 z 方向的高度（米）
   */
  void setTorsoState(double torso_yaw_rad, double torso_height);

  /**
   * @brief 执行IK求解
   *
   * 所有位置均在世界坐标系（躯干为原点）中给出，对应 torso_ik.py::TorsoIK::solve()。
   *
   * @param left_hand_pos   左手目标位置（可选）
   * @param right_hand_pos  右手目标位置（可选）
   * @param left_elbow_pos  左肘目标位置（可选）
   * @param right_elbow_pos 右肘目标位置（可选）
   * @param q_init          初始关节角猜测（若为空则使用上一帧解或默认解）
   * @return 关节角向量（nq 个元素，弧度），求解失败时返回空向量
   */
  Eigen::VectorXd computeIK(
      const std::optional<Eigen::Vector3d>& left_hand_pos,
      const std::optional<Eigen::Vector3d>& right_hand_pos,
      const std::optional<Eigen::Vector3d>& left_elbow_pos = std::nullopt,
      const std::optional<Eigen::Vector3d>& right_elbow_pos = std::nullopt,
      const Eigen::VectorXd& q_init = {});

  /// 获取默认关节角（零位）
  Eigen::VectorXd defaultPositions() const;

  /// 获取关节数量
  int numPositions() const;

  /// 将上一帧解重置为默认零位
  void resetLastSolution();

  /**
   * @brief 计算左手末端执行器在躯干坐标系中的位姿（正运动学）
   * @return (位置 [x,y,z], RPY 欧拉角 [r,p,y])
   */
  std::pair<Eigen::Vector3d, Eigen::Vector3d> getLeftHandPose(
      const Eigen::VectorXd& q);

  /**
   * @brief 计算右手末端执行器在躯干坐标系中的位姿（正运动学）
   * @return (位置 [x,y,z], RPY 欧拉角 [r,p,y])
   */
  std::pair<Eigen::Vector3d, Eigen::Vector3d> getRightHandPose(
      const Eigen::VectorXd& q);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ik
}  // namespace leju
