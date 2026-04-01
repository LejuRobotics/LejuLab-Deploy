/**
 * @file velocity_limited_generator.cpp
 * @brief 限速逼近轨迹生成器实现
 */

#include "leju-rl-controller/trajectory/generator/velocity_limited_generator.h"

#include <algorithm>
#include <cmath>

namespace leju {

VelocityLimitedGenerator::VelocityLimitedGenerator(VelocityLimitMode mode)
    : limit_mode_(mode) {
}

void VelocityLimitedGenerator::setTarget(const Eigen::VectorXd& target_pos,
                                         const Eigen::VectorXd& target_vel) {
    target_pos_ = target_pos;
    if (target_vel.size() > 0) {
        target_vel_ = target_vel;
    } else {
        target_vel_ = Eigen::VectorXd::Zero(target_pos.size());
    }
}

void VelocityLimitedGenerator::update(const Eigen::VectorXd& current_pos,
                                      const Eigen::VectorXd& current_vel,
                                      double dt) {
    // 检查维度
    const int dim = static_cast<int>(current_pos.size());
    if (target_pos_.size() == 0) {
        // 未设置目标，保持当前位置
        desired_pos_ = current_pos;
        desired_vel_ = Eigen::VectorXd::Zero(dim);
        error_ = Eigen::VectorXd::Zero(dim);
        return;
    }

    if (target_pos_.size() != dim) {
        // 维度不匹配，保持当前位置
        desired_pos_ = current_pos;
        desired_vel_ = Eigen::VectorXd::Zero(dim);
        error_ = Eigen::VectorXd::Zero(dim);
        return;
    }

    // 初始化或调整 max_velocity_ 维度
    if (max_velocity_.size() != dim) {
        // 默认最大速度为 1.0 rad/s
        max_velocity_ = Eigen::VectorXd::Constant(dim, 1.0);
    }

    // 初始化 desired_pos_（首次调用或维度不匹配时）
    if (desired_pos_.size() != dim) {
        desired_pos_ = current_pos;
    }

    // 使用 desired_pos_ 而非 current_pos 计算误差，确保轨迹连续跟踪目标
    // 这样即使实际位置落后，期望位置仍能持续追踪目标
    error_ = target_pos_ - desired_pos_;

    // 限制增量
    Eigen::VectorXd limited_delta = error_;

    if (limit_mode_ == VelocityLimitMode::kPerJoint) {
        // 逐关节限速
        Eigen::VectorXd max_delta = max_velocity_ * dt;
        for (int i = 0; i < dim; ++i) {
            if (std::abs(limited_delta[i]) > max_delta[i]) {
                limited_delta[i] = std::copysign(max_delta[i], limited_delta[i]);
            }
        }
    } else {
        // kSynchronized: 整体向量限速
        // 使用最大速度的范数作为向量速度限制
        double max_delta_norm = max_velocity_.norm() * dt;
        double error_norm = error_.norm();

        if (error_norm > max_delta_norm) {
            limited_delta = error_.normalized() * max_delta_norm;
        }
    }

    // 计算期望位置和速度（基于 desired_pos_ 累积）
    desired_pos_ = desired_pos_ + limited_delta;
    desired_vel_ = limited_delta / dt;
}

bool VelocityLimitedGenerator::isReached(double threshold) const {
    if (error_.size() == 0) {
        return true;
    }
    return error_.norm() < threshold;
}

const char* VelocityLimitedGenerator::getName() const {
    return "VelocityLimited";
}

void VelocityLimitedGenerator::setMaxVelocity(const Eigen::VectorXd& max_vel) {
    max_velocity_ = max_vel;
}

void VelocityLimitedGenerator::setMaxAcceleration(const Eigen::VectorXd& max_acc) {
    // 预留接口，当前实现不使用加速度约束
    max_acceleration_ = max_acc;
}

void VelocityLimitedGenerator::reset() {
    target_pos_.resize(0);
    target_vel_.resize(0);
    desired_pos_.resize(0);
    desired_vel_.resize(0);
    error_.resize(0);
    max_velocity_.resize(0);
    max_acceleration_.resize(0);
}

int VelocityLimitedGenerator::getDimension() const {
    return static_cast<int>(target_pos_.size());
}

void VelocityLimitedGenerator::setLimitMode(VelocityLimitMode mode) {
    limit_mode_ = mode;
}

}  // namespace leju
