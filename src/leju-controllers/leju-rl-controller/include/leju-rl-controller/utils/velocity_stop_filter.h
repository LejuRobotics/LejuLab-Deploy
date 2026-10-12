#pragma once

#include <cmath>

namespace leju {

struct VelocityStopFilterConfig {
  bool enabled = false;
  double deceleration = 1.0;
  double stop_target_threshold = 0.01;
};

class VelocityStopFilter {
 public:
  void setConfig(const VelocityStopFilterConfig& config) { config_ = config; }

  double update(double target, double dt) {
    if (!initialized_) {
      output_ = target;
      initialized_ = true;
      return output_;
    }

    if (!config_.enabled || dt <= 0.0 || config_.deceleration <= 0.0 ||
        std::abs(target) > config_.stop_target_threshold) {
      output_ = target;
      return output_;
    }

    const double step = config_.deceleration * dt;
    if (std::abs(output_) <= step) {
      output_ = 0.0;
    } else {
      output_ += (output_ > 0.0) ? -step : step;
    }
    return output_;
  }

  void reset(double value = 0.0) {
    output_ = value;
    initialized_ = true;
  }

 private:
  VelocityStopFilterConfig config_;
  double output_ = 0.0;
  bool initialized_ = false;
};

}  // namespace leju
