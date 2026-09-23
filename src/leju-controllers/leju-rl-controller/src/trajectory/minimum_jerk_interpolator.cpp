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
    // 标准 min-jerk：起止速度为零
    return setup(start_pos, Eigen::VectorXd::Zero(start_pos.size()),
                 end_pos, Eigen::VectorXd::Zero(end_pos.size()),
                 duration);
}

bool MinimumJerkInterpolator::setup(const Eigen::VectorXd& start_pos,
                                    const Eigen::VectorXd& start_vel,
                                    const Eigen::VectorXd& end_pos,
                                    const Eigen::VectorXd& end_vel,
                                    double duration) {
    if (start_pos.size() != end_pos.size() ||
        start_vel.size() != start_pos.size() ||
        end_vel.size() != end_pos.size()) {
        return false;
    }
    if (duration <= 0.0) {
        return false;
    }

    start_pos_ = start_pos;
    start_vel_ = start_vel;
    end_pos_ = end_pos;
    end_vel_ = end_vel;
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
    const double T = duration_;

    // 带起止速度的五次多项式系数（标准 min-jerk 即 start_vel=end_vel=0 的特例）：
    //   边界条件: p(0)=p0, v(0)=v0, a(0)=0; p(T)=p1, v(T)=v1, a(T)=0
    //   令 A = p1 - p0, B = v0*T, C = v1*T
    //   p(τ) = p0 + B*τ + k3*τ³ + k4*τ⁴ + k5*τ⁵
    //   v(t) = v0 + (3*k3*τ² + 4*k4*τ³ + 5*k5*τ⁴) / T
    //   k3 = 10A - 6B - 4C,  k4 = -15A + 8B + 7C,  k5 = 6A - 3B - 3C
    const Eigen::VectorXd delta = end_pos_ - start_pos_;
    const Eigen::VectorXd b = start_vel_ * T;
    const Eigen::VectorXd c = end_vel_ * T;

    const Eigen::VectorXd k3 = 10.0 * delta - 6.0 * b - 4.0 * c;
    const Eigen::VectorXd k4 = -15.0 * delta + 8.0 * b + 7.0 * c;
    const Eigen::VectorXd k5 = 6.0 * delta - 3.0 * b - 3.0 * c;

    const double tau2 = tau * tau;
    const double tau3 = tau2 * tau;
    const double tau4 = tau3 * tau;
    const double tau5 = tau4 * tau;

    // 位置
    pos = start_pos_ + b * tau + k3 * tau3 + k4 * tau4 + k5 * tau5;

    // 速度
    vel = start_vel_ + (3.0 * k3 * tau2 + 4.0 * k4 * tau3 + 5.0 * k5 * tau4) / T;

    return true;
}

bool MinimumJerkInterpolator::isFinished(double t) const {
    return t >= duration_;
}

void MinimumJerkInterpolator::reset() {
    start_pos_.resize(0);
    start_vel_.resize(0);
    end_pos_.resize(0);
    end_vel_.resize(0);
    duration_ = 0.0;
    initialized_ = false;
}

const char* MinimumJerkInterpolator::getName() const {
    return "MinimumJerk";
}

}  // namespace leju
