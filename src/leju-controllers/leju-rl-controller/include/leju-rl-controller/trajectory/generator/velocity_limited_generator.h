/**
 * @file velocity_limited_generator.h
 * @brief 限速逼近轨迹生成器
 *
 * 实现基于速度限制的在线轨迹生成，特性：
 * - 支持逐关节 (kPerJoint) 和同步 (kSynchronized) 两种限速模式
 * - 目标可随时更新，轨迹自动适应
 * - 简单高效，适用于实时控制
 */

#ifndef LEJU_TRAJECTORY_VELOCITY_LIMITED_GENERATOR_H_
#define LEJU_TRAJECTORY_VELOCITY_LIMITED_GENERATOR_H_

#include "leju-rl-controller/trajectory/generator/trajectory_generator_base.h"

namespace leju {

/**
 * @enum VelocityLimitMode
 * @brief 速度限制模式
 */
enum class VelocityLimitMode {
    kPerJoint,     ///< 逐关节限速（默认）：每个关节独立限速
    kSynchronized  ///< 整体向量限速：各关节同步到达目标
};

/**
 * @class VelocityLimitedGenerator
 * @brief 限速逼近轨迹生成器
 *
 * 核心算法：
 * 1. 计算误差: error = target_pos - current_pos
 * 2. 计算最大增量: max_delta = max_velocity * dt
 * 3. 限制误差:
 *    - kPerJoint: 逐元素限制
 *    - kSynchronized: 整体向量限制
 * 4. 计算期望: desired_pos = current_pos + limited_error
 */
class VelocityLimitedGenerator : public TrajectoryGeneratorBase {
public:
    /**
     * @brief 构造函数
     * @param mode 速度限制模式，默认为 kPerJoint
     */
    explicit VelocityLimitedGenerator(VelocityLimitMode mode = VelocityLimitMode::kPerJoint);

    ~VelocityLimitedGenerator() override = default;

    /**
     * @brief 设置目标位置和速度
     * @param target_pos 目标位置向量
     * @param target_vel 目标速度向量 (当前实现忽略此参数)
     */
    void setTarget(const Eigen::VectorXd& target_pos,
                   const Eigen::VectorXd& target_vel = Eigen::VectorXd()) override;

    /**
     * @brief 每周期调用，根据当前状态计算期望输出
     * @param current_pos 当前位置向量
     * @param current_vel 当前速度向量 (当前实现忽略此参数)
     * @param dt 时间步长 (秒)
     */
    void update(const Eigen::VectorXd& current_pos,
                const Eigen::VectorXd& current_vel,
                double dt) override;

    /**
     * @brief 获取计算的期望位置
     */
    const Eigen::VectorXd& getDesiredPos() const override { return desired_pos_; }

    /**
     * @brief 获取计算的期望速度
     */
    const Eigen::VectorXd& getDesiredVel() const override { return desired_vel_; }

    /**
     * @brief 检查是否已到达目标
     * @param threshold 到达阈值 (位置误差范数)
     */
    bool isReached(double threshold) const override;

    /**
     * @brief 获取轨迹生成器类型名称
     */
    const char* getName() const override;

    /**
     * @brief 设置最大速度约束
     * @param max_vel 各维度最大速度向量
     */
    void setMaxVelocity(const Eigen::VectorXd& max_vel) override;

    /**
     * @brief 设置最大加速度约束
     * @param max_acc 各维度最大加速度向量
     *
     * 当前实现不使用加速度约束，此方法仅存储参数供未来扩展。
     */
    void setMaxAcceleration(const Eigen::VectorXd& max_acc) override;

    /**
     * @brief 重置轨迹生成器状态
     */
    void reset() override;

    /**
     * @brief 获取维度
     */
    int getDimension() const override;

    /**
     * @brief 设置速度限制模式
     * @param mode 速度限制模式
     */
    void setLimitMode(VelocityLimitMode mode);

    /**
     * @brief 获取当前速度限制模式
     */
    VelocityLimitMode getLimitMode() const { return limit_mode_; }

    /**
     * @brief 获取当前误差向量
     */
    const Eigen::VectorXd& getError() const { return error_; }

private:
    VelocityLimitMode limit_mode_;     ///< 速度限制模式 (本类特有)
};

}  // namespace leju

#endif  // LEJU_TRAJECTORY_VELOCITY_LIMITED_GENERATOR_H_
