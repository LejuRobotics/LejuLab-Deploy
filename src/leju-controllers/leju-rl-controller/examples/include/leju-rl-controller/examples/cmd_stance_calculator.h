/**
 * @file cmd_stance_calculator.h
 * @brief cmdStance 计算器：根据 velocity_commands、智能停止条件计算 cmdStance
 *
 * 纯计算类，无 DDS 订阅。数据由 RLDemoController 传入。
 * 参考 kuavo-ros-control RlGaitReceiver 实现。
 */

#ifndef LEJU_RL_CONTROLLER_CMD_STANCE_CALCULATOR_H_
#define LEJU_RL_CONTROLLER_CMD_STANCE_CALCULATOR_H_

#include <Eigen/Dense>
#include <string>

namespace leju {
namespace rl_demo {

/**
 * @brief cmdStance 计算配置
 */
struct CmdStanceConfig {
  bool smart_stop_enabled = true;
  double torso_velocity_threshold = 0.05;   // m/s
  double feet_alignment_threshold = 0.08;   // m
  double velocity_magnitude_threshold = 0.01;  // 速度低于此值视为"小"
};

/**
 * @brief cmdStance 计算器
 *
 * 输入: velocity_commands (来自 get_obs_velocity_commands)、可选 torso_state、可选 feet_positions
 * 输出: cmdStance (1.0=站立, 0.0=行走)
 */
class CmdStanceCalculator {
 public:
  CmdStanceCalculator() = default;
  explicit CmdStanceCalculator(const CmdStanceConfig& config);

  void setConfig(const CmdStanceConfig& config);
  const CmdStanceConfig& getConfig() const { return config_; }

  /**
   * @brief 计算 cmdStance
   * @param velocity_commands [lin_vel_x, lin_vel_y, ang_vel_z]，来自 get_obs_velocity_commands
   * @param torso_state 可选，12维 [x,y,z,yaw,pitch,roll,vx,vy,vz,wx,wy,wz]，空则跳过智能停止
   * @param feet_positions 可选，24维（4点*3*2脚），空则跳过双脚对齐判断
   * @return cmdStance: 1.0=站立, 0.0=行走
   */
  double compute(const Eigen::Vector3d& velocity_commands,
                 const Eigen::VectorXd& torso_state,
                 const Eigen::VectorXd& feet_positions);

  /**
   * @brief 简化接口：仅使用 velocity_commands（无智能停止）
   */
  double computeSimple(const Eigen::Vector3d& velocity_commands);

 private:
  bool shouldSmartStop(const Eigen::VectorXd& torso_state,
                       const Eigen::VectorXd& feet_positions);

  CmdStanceConfig config_;
};

}  // namespace rl_demo
}  // namespace leju

#endif  // LEJU_RL_CONTROLLER_CMD_STANCE_CALCULATOR_H_
