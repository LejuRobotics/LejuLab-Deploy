/**
 * @file arm_controller.h
 * @brief 手臂控制器：行走时用 RL 摆手，站立时平滑插值到默认姿态
 *
 * 纯计算类，无 DDS 订阅。参考 kuavo-ros-control ArmController 模式 1 实现。
 */

#ifndef LEJU_RL_CONTROLLER_ARM_CONTROLLER_H_
#define LEJU_RL_CONTROLLER_ARM_CONTROLLER_H_

#include <chrono>
#include <Eigen/Dense>

namespace leju {
namespace rl_demo {

/**
 * @brief 手臂控制器配置
 */
struct ArmControllerConfig {
  bool enabled = true;
  double interpolation_velocity = 1.0;  ///< 插值最大速度 rad/s
  double min_duration = 0.2;            ///< 插值最短时长 s
  double max_duration = 2.0;            ///< 插值最长时长 s
};

/**
 * @brief 手臂控制器
 *
 * 输入: cmd_stance, 当前手臂位置/速度, 默认手臂姿态
 * 输出: 站立时覆盖手臂指令为插值到默认姿态；行走时不覆盖，使用 RL 输出
 */
class ArmController {
 public:
  ArmController() = default;
  explicit ArmController(const ArmControllerConfig& config);

  void setConfig(const ArmControllerConfig& config);
  const ArmControllerConfig& getConfig() const { return config_; }

  /**
   * @brief 初始化手臂索引与默认姿态
   * @param arm_start_index 手臂在 policy 关节中的起始索引
   * @param arm_joint_count 手臂关节数
   * @param default_arm_pos 默认手臂姿态 (rad)
   */
  void init(int arm_start_index, int arm_joint_count,
            const Eigen::VectorXd& default_arm_pos);

  /**
   * @brief 更新手臂控制
   * @param cmd_stance 1=站立, 0=行走
   * @param current_arm_pos 当前手臂位置 (rad)
   * @param current_arm_vel 当前手臂速度 (rad/s)
   * @param now 当前时间戳
   * @param desire_q [out] 期望手臂位置
   * @param desire_v [out] 期望手臂速度
   * @return 是否应覆盖 cmd 的手臂部分（站立时为 true）
   */
  bool update(double cmd_stance,
              const Eigen::VectorXd& current_arm_pos,
              const Eigen::VectorXd& current_arm_vel,
              std::chrono::steady_clock::time_point now,
              Eigen::VectorXd* desire_q,
              Eigen::VectorXd* desire_v);

 private:
  void resetInterpolationState(std::chrono::steady_clock::time_point time,
                               const Eigen::VectorXd& start_pos,
                               const Eigen::VectorXd& target_pos);
  void applySmoothInterpolation(std::chrono::steady_clock::time_point current_time,
                                const Eigen::VectorXd& target_pos,
                                const Eigen::VectorXd& target_vel,
                                Eigen::VectorXd* desire_q,
                                Eigen::VectorXd* desire_v);

  ArmControllerConfig config_;
  int arm_start_index_ = 0;
  int arm_joint_count_ = 0;
  Eigen::VectorXd default_arm_pos_;

  std::chrono::steady_clock::time_point interpolation_start_time_;
  Eigen::VectorXd interpolation_start_pos_;
  double interpolation_duration_ = 0.5;

  bool is_interpolating_to_default_ = false;
};

}  // namespace rl_demo
}  // namespace leju

#endif  // LEJU_RL_CONTROLLER_ARM_CONTROLLER_H_
