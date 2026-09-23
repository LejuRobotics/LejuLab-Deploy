#pragma once

#include <cmath>

namespace leju {

struct MixedMotionLimitsConfig {
  bool enabled = false;
  double angular_vel_threshold = 0.25;       // rad/s：角速度超过此值时限制线速度
  double max_linear_vel_with_angular = 0.2;  // m/s：转向时允许的最大线速度
  double linear_vel_threshold = 0.4;         // m/s：线速度超过此值时限制角速度
  double max_angular_vel_with_linear = 0.4;  // rad/s：直行时允许的最大角速度
};

// 线速度与角速度互耦限幅：转向时压线速度，直行时压角速度。
// 判断条件使用限幅前的原始模长，两个方向可同时触发。
class MixedMotionLimiter {
 public:
  void setConfig(const MixedMotionLimitsConfig& config) { config_ = config; }

  void apply(double& linear_x, double& linear_y, double& angular_z) const {
    if (!config_.enabled) return;

    const double angular_mag = std::abs(angular_z);
    const double linear_mag = std::hypot(linear_x, linear_y);

    if (angular_mag > config_.angular_vel_threshold &&
        linear_mag > config_.max_linear_vel_with_angular) {
      const double scale = config_.max_linear_vel_with_angular / linear_mag;
      linear_x *= scale;
      linear_y *= scale;
    }

    if (linear_mag > config_.linear_vel_threshold &&
        angular_mag > config_.max_angular_vel_with_linear) {
      angular_z = std::copysign(config_.max_angular_vel_with_linear, angular_z);
    }
  }

 private:
  MixedMotionLimitsConfig config_;
};

}  // namespace leju
