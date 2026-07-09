/**
 * @file minimum_jerk_interpolator.cpp
 * @brief 最小急动度插值器实现
 */

#include "leju-rl-controller/trajectory/interpolator/minimum_jerk_interpolator.h"
#include <algorithm>

namespace leju {

bool MinimumJerkInterpolator::setup(const Eigen::VectorXd& start_pos,
                                    const Eigen::VectorXd& end_pos,
                                    double duration) {
    if (start_pos.size() != end_pos.size()) {
        return false;
    }
    if (duration <= 0.0) {
        return false;
    }

    start_pos_ = start_pos;
    end_pos_ = end_pos;
    duration_ = duration;
    initialized_ = true;
    return true;
}

bool MinimumJerkInterpolator::evaluate(double t,
                                       Eigen::VectorXd& pos,
                                       Eigen::VectorXd& vel) const {
    if (!initialized_) {
        return false;
    }

    // 限制 t 在 [0, duration_] 范围内
    const double t_clamped = std::clamp(t, 0.0, duration_);
    const double tau = t_clamped / duration_;

    // 计算插值系数
    const double s = computeS(tau);
    const Eigen::VectorXd delta = end_pos_ - start_pos_;

    // 计算位置: p(t) = p0 + s(tau) * (p1 - p0)
    pos = start_pos_ + s * delta;

    // 计算速度: v(t) = ds/dt * (p1 - p0) = (ds/dtau) * (1/T) * (p1 - p0)
    const double ds_dtau = computeDsDtau(tau);
    const double ds_dt = ds_dtau / duration_;
    vel = ds_dt * delta;

    return true;
}

bool MinimumJerkInterpolator::isFinished(double t) const {
    return t >= duration_;
}

void MinimumJerkInterpolator::reset() {
    start_pos_.resize(0);
    end_pos_.resize(0);
    duration_ = 0.0;
    initialized_ = false;
}

double MinimumJerkInterpolator::computeS(double tau) {
    // s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
    const double tau2 = tau * tau;
    const double tau3 = tau2 * tau;
    const double tau4 = tau3 * tau;
    const double tau5 = tau4 * tau;
    return 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
}

double MinimumJerkInterpolator::computeDsDtau(double tau) {
    // ds/dtau = 30*tau^2 - 60*tau^3 + 30*tau^4
    const double tau2 = tau * tau;
    const double tau3 = tau2 * tau;
    const double tau4 = tau3 * tau;
    return 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4;
}

const char* MinimumJerkInterpolator::getName() const {
    return "MinimumJerk";
}

}  // namespace leju
