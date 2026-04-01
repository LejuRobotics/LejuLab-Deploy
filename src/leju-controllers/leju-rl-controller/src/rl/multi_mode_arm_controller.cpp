/**
 * @file multi_mode_arm_controller.cpp
 * @brief 多模式手臂控制器实现：三种控制模式与平滑过渡
 */

#include "leju-rl-controller/rl/multi_mode_arm_controller.h"
#include "leju-rl-controller/rl_log.h"

#include <algorithm>
#include <cmath>

namespace leju {

MultiModeArmController::MultiModeArmController(const MultiModeArmControllerConfig& config)
    : config_(config) {}

void MultiModeArmController::setConfig(const MultiModeArmControllerConfig& config) {
    config_ = config;

    // 更新滤波器截止频率
    if (initialized_) {
        Eigen::VectorXd cutoff_freq = Eigen::VectorXd::Constant(
            arm_joint_count_, config_.filter_cutoff_freq);
        target_filter_.setParams(dt_, cutoff_freq);
    }
}

void MultiModeArmController::init(int arm_joint_count,
                                  const Eigen::VectorXd& default_arm_pos,
                                  double dt) {
    arm_joint_count_ = arm_joint_count;
    default_arm_pos_ = default_arm_pos;
    dt_ = dt;

    // 初始化状态向量
    current_arm_pos_ = default_arm_pos;
    desire_arm_q_ = default_arm_pos;
    desire_arm_v_ = Eigen::VectorXd::Zero(arm_joint_count);
    keep_pose_q_ = default_arm_pos;
    raw_target_q_ = default_arm_pos;
    filtered_target_q_ = default_arm_pos;

    // 初始化滤波器
    Eigen::VectorXd cutoff_freq = Eigen::VectorXd::Constant(
        arm_joint_count, config_.filter_cutoff_freq);
    target_filter_.setParams(dt, cutoff_freq);
    target_filter_.reset(default_arm_pos);

    // 初始化外部模式轨迹生成器
    Eigen::VectorXd max_vel = Eigen::VectorXd::Constant(arm_joint_count, config_.approach_velocity);
    external_generator_.setMaxVelocity(max_vel);

    // 重置状态标志
    mode_ = ArmControlMode::kAuto;

    is_interpolating_to_default_ = false;
    is_approaching_ = true;
    target_received_ = false;
    mode_transitioning_ = false;
    auto_interpolator_time_ = 0.0;
    transition_time_ = 0.0;

    initialized_ = true;
}

void MultiModeArmController::setMode(ArmControlMode mode) {
    if (!initialized_) {
        return;
    }

    // 检查是否需要切换：当前模式或待切换模式与目标相同时跳过
    // 修复：当正在过渡到其他模式时，如果请求切换回当前模式，需要重新启动过渡
    if (mode == mode_ && (!mode_transitioning_ || mode == pending_mode_)) {
        return;
    }

    // 计算模式切换的目标位置
    Eigen::VectorXd target_q;
    switch (mode) {
        case ArmControlMode::kKeepPose:
            // 切换到 KEEP_POSE: 目标是当前位置
            keep_pose_q_ = current_arm_pos_;
            target_q = current_arm_pos_;
            break;

        case ArmControlMode::kAuto:
            // 切换到 AUTO: 目标是默认姿态
            target_q = default_arm_pos_;
            is_interpolating_to_default_ = false;
            break;

        case ArmControlMode::kExternal: {
            // 重置轨迹生成器状态，避免残留旧状态导致手臂先完成旧动作
            external_generator_.reset();

            // 重置目标接收标志，避免使用上一次程序残留的旧目标
            target_received_ = false;

            // 切换到 EXTERNAL: 从当前位置开始，等待新的外部命令
            target_q = current_arm_pos_;
            raw_target_q_ = current_arm_pos_;
            filtered_target_q_ = current_arm_pos_;
            target_filter_.reset(current_arm_pos_);

            // 重置为接入阶段
            is_approaching_ = true;
            // 设置接入阶段速度
            Eigen::VectorXd approach_vel = Eigen::VectorXd::Constant(
                arm_joint_count_, config_.approach_velocity);
            external_generator_.setMaxVelocity(approach_vel);
            break;
        }
    }

    // 启动平滑过渡
    Eigen::VectorXd from_q = desire_arm_q_.size() > 0 ? desire_arm_q_ : current_arm_pos_;
    startModeTransition(from_q, target_q);

    pending_mode_ = mode;
}

void MultiModeArmController::setExternalTarget(const Eigen::VectorXd& q,
                                               const Eigen::VectorXd& v) {
    if (!initialized_) {
        return;
    }
    if (q.size() != arm_joint_count_) {
        RL_LOG_WARNING("Arm external target dimension mismatch: expected %d, got %d",
                       arm_joint_count_, static_cast<int>(q.size()));
        return;
    }

    raw_target_q_ = q;
    target_received_ = true;

    // 更新滤波器
    filtered_target_q_ = target_filter_.update(q);
}

void MultiModeArmController::startModeTransition(const Eigen::VectorXd& from_q,
                                                 const Eigen::VectorXd& to_q) {
    // 根据最大关节位移动态计算过渡时长
    // 五次多项式峰值速度 V_max = 1.875 * (max_dist / T)
    // 为保证峰值速度不超过 mode_interpolation_velocity，取 T = 1.875 * max_dist / V_limit
    double duration = config_.mode_transition_duration;  // 默认值
    if (from_q.size() > 0 && from_q.size() == to_q.size()) {
        double max_dist = (to_q - from_q).lpNorm<Eigen::Infinity>();
        if (max_dist > 1e-6 && config_.mode_interpolation_velocity > 0) {
            // 使用 1.875 系数（五次多项式峰值速度系数）
            duration = 1.875 * max_dist / config_.mode_interpolation_velocity;
            // 限制时长范围 [min_duration, max_duration]
            duration = std::max(config_.min_duration, std::min(config_.max_duration, duration));
        }
    }

    transition_interpolator_.setup(from_q, to_q, duration);
    transition_time_ = 0.0;
    mode_transitioning_ = true;
}

bool MultiModeArmController::updateModeTransition(Eigen::VectorXd* desire_q,
                                                  Eigen::VectorXd* desire_v) {
    if (!mode_transitioning_) {
        return true;  // 已完成
    }

    transition_time_ += dt_;

    Eigen::VectorXd pos, vel;
    transition_interpolator_.evaluate(transition_time_, pos, vel);

    *desire_q = pos;
    *desire_v = vel;

    if (transition_interpolator_.isFinished(transition_time_)) {
        mode_transitioning_ = false;
        mode_ = pending_mode_;
        return true;  // 过渡完成
    }

    return false;  // 过渡中
}

bool MultiModeArmController::update(double cmd_stance,
                                    const Eigen::VectorXd& current_arm_pos,
                                    const Eigen::VectorXd& current_arm_vel,
                                    Eigen::VectorXd* desire_q,
                                    Eigen::VectorXd* desire_v) {
    if (!config_.enabled || !initialized_ || arm_joint_count_ <= 0) {
        return false;
    }

    if (desire_q == nullptr || desire_v == nullptr) {
        return false;
    }

    // 保存当前位置（供 setMode 使用）
    current_arm_pos_ = current_arm_pos;

    desire_q->resize(arm_joint_count_);
    desire_v->resize(arm_joint_count_);

    // 处理模式切换过渡
    if (mode_transitioning_) {
        bool transition_done = updateModeTransition(desire_q, desire_v);
        desire_arm_q_ = *desire_q;

        if (!transition_done) {
            return true;  // 过渡期间始终覆盖
        }
        // 过渡完成，继续执行新模式逻辑
    }

    // 根据模式调用对应的更新函数
    bool should_override = false;
    switch (mode_) {
        case ArmControlMode::kKeepPose:
            should_override = updateKeepPose(current_arm_pos, desire_q, desire_v);
            break;

        case ArmControlMode::kAuto:
            should_override = updateAuto(cmd_stance, current_arm_pos, current_arm_vel,
                                         desire_q, desire_v);
            break;

        case ArmControlMode::kExternal:
            should_override = updateExternal(current_arm_pos, current_arm_vel,
                                             desire_q, desire_v);
            break;
    }

    if (should_override) {
        desire_arm_q_ = *desire_q;
        desire_arm_v_ = *desire_v;
    }

    return should_override;
}

bool MultiModeArmController::updateKeepPose(const Eigen::VectorXd& current_arm_pos,
                                            Eigen::VectorXd* desire_q,
                                            Eigen::VectorXd* desire_v) {
    // 保持固定位置
    *desire_q = keep_pose_q_;
    *desire_v = Eigen::VectorXd::Zero(arm_joint_count_);
    return true;
}

bool MultiModeArmController::updateAuto(double cmd_stance,
                                        const Eigen::VectorXd& current_arm_pos,
                                        const Eigen::VectorXd& current_arm_vel,
                                        Eigen::VectorXd* desire_q,
                                        Eigen::VectorXd* desire_v) {
    if (cmd_stance >= 0.5) {
        // 站立：从当前位姿平滑插值到 default_arm_pos_
        if (!is_interpolating_to_default_) {
            // 动态计算插值时长：T = 1.5 * max_dist / V_limit
            double max_dist = (default_arm_pos_ - current_arm_pos).lpNorm<Eigen::Infinity>();
            double duration = 1.5 * max_dist / config_.interpolation_velocity;
            duration = std::max(config_.min_duration,
                               std::min(config_.max_duration, duration));

            auto_interpolator_.setup(current_arm_pos, default_arm_pos_, duration);
            auto_interpolator_time_ = 0.0;
            is_interpolating_to_default_ = true;
        }

        auto_interpolator_time_ += dt_;

        Eigen::VectorXd pos, vel;
        auto_interpolator_.evaluate(auto_interpolator_time_, pos, vel);
        *desire_q = pos;
        *desire_v = vel;

        if (auto_interpolator_.isFinished(auto_interpolator_time_)) {
            // 插值完成，重置标志以便下一帧重新捕获当前位置
            *desire_q = default_arm_pos_;
            *desire_v = Eigen::VectorXd::Zero(arm_joint_count_);
            is_interpolating_to_default_ = false;
        }

        return true;
    }

    // 行走：不覆盖，使用 RL 输出
    is_interpolating_to_default_ = false;
    return false;
}

bool MultiModeArmController::updateExternal(const Eigen::VectorXd& current_arm_pos,
                                            const Eigen::VectorXd& current_arm_vel,
                                            Eigen::VectorXd* desire_q,
                                            Eigen::VectorXd* desire_v) {
    if (!target_received_) {
        // 未收到外部目标，保持当前位置
        *desire_q = current_arm_pos;
        *desire_v = Eigen::VectorXd::Zero(arm_joint_count_);
        return true;
    }

    // 选择目标和速度
    Eigen::VectorXd target_q;
    if (is_approaching_) {
        // 接入阶段：使用原始目标，慢速
        target_q = raw_target_q_;
    } else {
        // 跟踪阶段：使用滤波目标，快速
        target_q = filtered_target_q_;
    }

    // 更新轨迹生成器
    external_generator_.setTarget(target_q);
    external_generator_.update(current_arm_pos, current_arm_vel, dt_);

    *desire_q = external_generator_.getDesiredPos();
    *desire_v = external_generator_.getDesiredVel();

    // 检查接入阶段是否完成
    if (is_approaching_) {
        if (external_generator_.isReached(config_.approach_threshold)) {
            // 接入完成，切换到跟踪阶段
            is_approaching_ = false;

            // 设置跟踪阶段速度
            Eigen::VectorXd tracking_vel = Eigen::VectorXd::Constant(
                arm_joint_count_, config_.tracking_velocity);
            external_generator_.setMaxVelocity(tracking_vel);

            // 重置滤波器到当前目标
            target_filter_.reset(raw_target_q_);
            filtered_target_q_ = raw_target_q_;
        }
    }

    return true;
}

}  // namespace leju
