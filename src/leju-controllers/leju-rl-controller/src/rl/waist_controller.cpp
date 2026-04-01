/**
 * @file waist_controller.cpp
 * @brief 腰部控制器实现：三次多项式插值 (kAuto) + 低通滤波 (kExternal)
 *
 * 参考 MultiModeArmController 模式，使用内部时间跟踪 (dt_ 累加)
 */

#include "leju-rl-controller/rl/waist_controller.h"
#include "leju-rl-controller/rl_log.h"

#include <algorithm>
#include <cmath>

namespace leju {

WaistController::WaistController(const WaistControllerConfig& config)
    : config_(config) {}

void WaistController::setConfig(const WaistControllerConfig& config) {
    config_ = config;

    // 更新滤波器截止频率
    if (initialized_) {
        Eigen::VectorXd cutoff_freq = Eigen::VectorXd::Constant(
            waist_joint_count_, config_.filter_cutoff_freq);
        filter_.setParams(dt_, cutoff_freq);
    }
}

void WaistController::init(int waist_joint_count,
                           const Eigen::VectorXd& default_waist_pos,
                           double dt) {
    waist_joint_count_ = waist_joint_count;
    default_waist_pos_ = default_waist_pos;
    dt_ = dt;

    // 初始化状态向量
    current_waist_pos_ = default_waist_pos;
    desire_waist_q_ = default_waist_pos;
    desire_waist_v_ = Eigen::VectorXd::Zero(waist_joint_count);
    raw_target_q_ = default_waist_pos;
    interpolation_start_pos_ = default_waist_pos;
    interpolation_target_pos_ = default_waist_pos;

    // 初始化滤波器 (用于 kExternal 模式)
    Eigen::VectorXd cutoff_freq = Eigen::VectorXd::Constant(
        waist_joint_count, config_.filter_cutoff_freq);
    filter_.setParams(dt, cutoff_freq);
    filter_.reset(default_waist_pos);

    // 重置状态标志
    mode_ = WaistControlMode::kAuto;
    is_interpolating_ = false;
    is_filtering_ = false;
    target_received_ = false;
    interpolation_time_ = 0.0;

    initialized_ = true;
}

void WaistController::startInterpolation(const Eigen::VectorXd& start_pos,
                                          const Eigen::VectorXd& target_pos) {
    interpolation_start_pos_ = start_pos;
    interpolation_target_pos_ = target_pos;
    interpolation_time_ = 0.0;

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

    is_interpolating_ = true;
}

bool WaistController::updateInterpolation(Eigen::VectorXd* desire_q,
                                           Eigen::VectorXd* desire_v) {
    if (!is_interpolating_) {
        return true;  // 已完成
    }

    interpolation_time_ += dt_;

    double invT = 1.0 / interpolation_duration_;
    double tau = interpolation_time_ * invT;

    if (tau >= 1.0) {
        *desire_q = interpolation_target_pos_;
        *desire_v = Eigen::VectorXd::Zero(waist_joint_count_);
        is_interpolating_ = false;
        return true;  // 插值完成
    }

    // 三次多项式插值：s = 3τ² - 2τ³
    double tau2 = tau * tau;
    double s = tau2 * (3.0 - 2.0 * tau);

    // ds/dτ = 6τ - 6τ²，ds/dt = ds/dτ * dτ/dt
    double ds_dtau = 6.0 * tau * (1.0 - tau);
    double ds_dt = ds_dtau * invT;

    *desire_q = interpolation_start_pos_ +
                s * (interpolation_target_pos_ - interpolation_start_pos_);
    *desire_v = ds_dt * (interpolation_target_pos_ - interpolation_start_pos_);

    return false;  // 插值进行中
}

void WaistController::setMode(WaistControlMode mode) {
    if (!initialized_ || mode == mode_) {
        return;
    }

    if (mode == WaistControlMode::kAuto) {
        // 切换到 kAuto: 启动三次多项式插值到默认位置
        double error = (default_waist_pos_ - current_waist_pos_).norm();
        if (error > config_.tracking_threshold) {
            startInterpolation(current_waist_pos_, default_waist_pos_);
        } else {
            is_interpolating_ = false;
        }
        is_filtering_ = false;
        target_received_ = false;
    } else if (mode == WaistControlMode::kExternal) {
        // 切换到 kExternal: 初始化滤波器
        is_filtering_ = true;
        is_interpolating_ = false;
        target_received_ = false;
        filter_.reset(current_waist_pos_);
        raw_target_q_ = current_waist_pos_;
    }

    mode_ = mode;
}

void WaistController::setExternalTarget(const Eigen::VectorXd& q) {
    if (!initialized_) {
        return;
    }
    if (q.size() != waist_joint_count_) {
        RL_LOG_WARNING("Waist external target dimension mismatch: expected %d, got %d",
                       waist_joint_count_, static_cast<int>(q.size()));
        return;
    }

    raw_target_q_ = q;
    target_received_ = true;
}

bool WaistController::update(double cmd_stance,
                             const Eigen::VectorXd& current_waist_pos,
                             const Eigen::VectorXd& current_waist_vel,
                             Eigen::VectorXd* desire_q,
                             Eigen::VectorXd* desire_v) {
    if (!config_.enabled || !initialized_ || waist_joint_count_ <= 0) {
        return false;
    }

    if (desire_q == nullptr || desire_v == nullptr) {
        return false;
    }

    // 保存当前位置（供 setMode 使用）
    current_waist_pos_ = current_waist_pos;

    desire_q->resize(waist_joint_count_);
    desire_v->resize(waist_joint_count_);

    // 根据模式调用对应的更新函数
    bool should_override = false;
    switch (mode_) {
        case WaistControlMode::kAuto:
            should_override = updateAuto(cmd_stance, current_waist_pos, current_waist_vel,
                                         desire_q, desire_v);
            break;

        case WaistControlMode::kExternal:
            should_override = updateExternal(current_waist_pos, current_waist_vel,
                                             desire_q, desire_v);
            break;
    }

    if (should_override) {
        desire_waist_q_ = *desire_q;
        desire_waist_v_ = *desire_v;
    }

    return should_override;
}

bool WaistController::updateAuto(double cmd_stance,
                                 const Eigen::VectorXd& current_waist_pos,
                                 const Eigen::VectorXd& current_waist_vel,
                                 Eigen::VectorXd* desire_q,
                                 Eigen::VectorXd* desire_v) {
    if (cmd_stance >= 0.5) {
        // 站立：使用三次多项式插值过渡到默认位置
        if (!is_interpolating_) {
            // 检查与默认姿态的误差
            double error = (default_waist_pos_ - current_waist_pos).norm();
            if (error > config_.tracking_threshold) {
                startInterpolation(current_waist_pos, default_waist_pos_);
            }
        }

        if (is_interpolating_) {
            // 使用三次多项式从当前位置过渡到默认位置
            updateInterpolation(desire_q, desire_v);
            return true;  // 覆盖命令
        } else {
            // 已到达默认姿态，保持
            *desire_q = default_waist_pos_;
            *desire_v = Eigen::VectorXd::Zero(waist_joint_count_);
            return true;  // 保持默认姿态
        }
    }

    // 行走：不覆盖，使用 RL 输出
    is_interpolating_ = false;
    return false;
}

bool WaistController::updateExternal(const Eigen::VectorXd& current_waist_pos,
                                     const Eigen::VectorXd& current_waist_vel,
                                     Eigen::VectorXd* desire_q,
                                     Eigen::VectorXd* desire_v) {
    if (!target_received_) {
        // 未收到外部目标，保持当前位置
        *desire_q = current_waist_pos;
        *desire_v = Eigen::VectorXd::Zero(waist_joint_count_);
        return true;
    }

    // 使用低通滤波器平滑外部输入
    Eigen::VectorXd filtered_q = filter_.update(raw_target_q_);
    *desire_q = filtered_q;

    // 速度通过差分计算
    if (desire_waist_q_.size() == waist_joint_count_) {
        *desire_v = (filtered_q - desire_waist_q_) / dt_;
    } else {
        *desire_v = Eigen::VectorXd::Zero(waist_joint_count_);
    }

    return true;
}

void WaistController::reset() {
    if (!initialized_) {
        return;
    }

    // 重置状态
    mode_ = WaistControlMode::kAuto;
    is_interpolating_ = false;
    is_filtering_ = false;
    target_received_ = false;
    interpolation_time_ = 0.0;

    // 重置滤波器
    filter_.reset(default_waist_pos_);

    // 重置位置
    current_waist_pos_ = default_waist_pos_;
    desire_waist_q_ = default_waist_pos_;
    desire_waist_v_ = Eigen::VectorXd::Zero(waist_joint_count_);
    raw_target_q_ = default_waist_pos_;
    interpolation_start_pos_ = default_waist_pos_;
    interpolation_target_pos_ = default_waist_pos_;
}

}  // namespace leju
