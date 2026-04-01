/**
 * @file arm_controller.cpp
 * @brief 手臂控制器实现：三次多项式平滑插值
 */

#include "leju-rl-controller/examples/arm_controller.h"
#include <algorithm>
#include <cmath>

namespace leju {
namespace rl_demo {

ArmController::ArmController(const ArmControllerConfig& config)
    : config_(config) {}

void ArmController::setConfig(const ArmControllerConfig& config) {
  config_ = config;
}

void ArmController::init(int arm_start_index, int arm_joint_count,
                         const Eigen::VectorXd& default_arm_pos) {
  arm_start_index_ = arm_start_index;
  arm_joint_count_ = arm_joint_count;
  default_arm_pos_ = default_arm_pos;
  is_interpolating_to_default_ = false;
}

void ArmController::resetInterpolationState(
    std::chrono::steady_clock::time_point time,
    const Eigen::VectorXd& start_pos,
    const Eigen::VectorXd& target_pos) {
  interpolation_start_time_ = time;
  interpolation_start_pos_ = start_pos;

  // 根据最大关节位移和插值速度动态计算时长
  // 三次多项式 s = 3τ² - 2τ³，最大导数（速度系数）约 1.5
  // T = 1.5 * max_dist / V_limit
  if (start_pos.size() > 0 && start_pos.size() == target_pos.size()) {
    double max_dist =
        (target_pos - start_pos).lpNorm<Eigen::Infinity>();
    interpolation_duration_ =
        1.5 * max_dist / config_.interpolation_velocity;
    interpolation_duration_ = std::max(
        config_.min_duration,
        std::min(config_.max_duration, interpolation_duration_));
  } else {
    interpolation_duration_ = 0.5;
  }
}

void ArmController::applySmoothInterpolation(
    std::chrono::steady_clock::time_point current_time,
    const Eigen::VectorXd& target_pos,
    const Eigen::VectorXd& target_vel,
    Eigen::VectorXd* desire_q,
    Eigen::VectorXd* desire_v) {
  using namespace std::chrono;
  double t = duration<double>(current_time - interpolation_start_time_).count();
  double invT = 1.0 / interpolation_duration_;
  double tau = t * invT;

  if (tau >= 1.0) {
    *desire_q = target_pos;
    *desire_v = target_vel;
    is_interpolating_to_default_ = false;
    return;
  }

  // 三次多项式插值：s = 3τ² - 2τ³
  double tau2 = tau * tau;
  double s = tau2 * (3.0 - 2.0 * tau);

  // ds/dτ = 6τ - 6τ²，ds/dt = ds/dτ * dτ/dt
  double ds_dtau = 6.0 * tau * (1.0 - tau);
  double ds_dt = ds_dtau * invT;

  *desire_q = interpolation_start_pos_ +
              s * (target_pos - interpolation_start_pos_);
  *desire_v = ds_dt * (target_pos - interpolation_start_pos_);
}

bool ArmController::update(double cmd_stance,
                           const Eigen::VectorXd& current_arm_pos,
                           const Eigen::VectorXd& current_arm_vel,
                           std::chrono::steady_clock::time_point now,
                           Eigen::VectorXd* desire_q,
                           Eigen::VectorXd* desire_v) {
  if (!config_.enabled || arm_joint_count_ <= 0) {
    return false;
  }

  if (desire_q == nullptr || desire_v == nullptr) {
    return false;
  }

  desire_q->resize(arm_joint_count_);
  desire_v->resize(arm_joint_count_);

  if (cmd_stance >= 0.5) {
    // 站立：从当前位姿平滑插值到 default_arm_pos_
    if (!is_interpolating_to_default_) {
      interpolation_start_pos_ = current_arm_pos;
      resetInterpolationState(now, current_arm_pos, default_arm_pos_);
      is_interpolating_to_default_ = true;
    }
    Eigen::VectorXd target_vel = Eigen::VectorXd::Zero(arm_joint_count_);
    applySmoothInterpolation(now, default_arm_pos_, target_vel, desire_q,
                             desire_v);
    return true;
  }

  // 行走：不覆盖，使用 RL 输出
  is_interpolating_to_default_ = false;
  return false;
}

}  // namespace rl_demo
}  // namespace leju
