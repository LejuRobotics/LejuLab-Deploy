/**
 * @file trajectory_generator_base.h
 * @brief 在线轨迹生成器抽象基类
 *
 * 定义轨迹生成器的通用接口，用于 MODE_EXTERNAL 实时目标跟踪：
 * - VelocityLimitedGenerator (限速逼近)
 * - RuckigGenerator (时间最优，未来)
 */

#ifndef LEJU_TRAJECTORY_GENERATOR_BASE_H_
#define LEJU_TRAJECTORY_GENERATOR_BASE_H_

#include <Eigen/Dense>

namespace leju {

/**
 * @class TrajectoryGeneratorBase
 * @brief 在线轨迹生成器抽象基类
 *
 * 定义所有轨迹生成器必须实现的核心接口。
 * 与插值器不同，轨迹生成器支持：
 * - 目标可随时更新
 * - 基于当前状态计算期望输出
 * - 速度和加速度约束
 */
class TrajectoryGeneratorBase {
public:
    virtual ~TrajectoryGeneratorBase() = default;

    /**
     * @brief 设置目标位置和速度
     * @param target_pos 目标位置向量
     * @param target_vel 目标速度向量 (可选，默认为零向量)
     *
     * 目标可随时更新，轨迹生成器会自动适应新目标。
     */
    virtual void setTarget(const Eigen::VectorXd& target_pos,
                           const Eigen::VectorXd& target_vel = Eigen::VectorXd()) = 0;

    /**
     * @brief 每周期调用，根据当前状态计算期望输出
     * @param current_pos 当前位置向量
     * @param current_vel 当前速度向量
     * @param dt 时间步长 (秒)
     *
     * 计算结果存储在内部，通过 getDesiredPos/getDesiredVel 获取。
     */
    virtual void update(const Eigen::VectorXd& current_pos,
                        const Eigen::VectorXd& current_vel,
                        double dt) = 0;

    /**
     * @brief 获取计算的期望位置
     * @return 期望位置向量的常引用
     */
    virtual const Eigen::VectorXd& getDesiredPos() const = 0;

    /**
     * @brief 获取计算的期望速度
     * @return 期望速度向量的常引用
     */
    virtual const Eigen::VectorXd& getDesiredVel() const = 0;

    /**
     * @brief 检查是否已到达目标
     * @param threshold 到达阈值 (位置误差范数)
     * @return 当误差范数小于阈值时返回 true
     */
    virtual bool isReached(double threshold) const = 0;

    /**
     * @brief 获取轨迹生成器类型名称
     * @return 类型名称字符串
     */
    virtual const char* getName() const = 0;

    /**
     * @brief 设置最大速度约束
     * @param max_vel 各维度最大速度向量
     */
    virtual void setMaxVelocity(const Eigen::VectorXd& max_vel) = 0;

    /**
     * @brief 设置最大加速度约束
     * @param max_acc 各维度最大加速度向量
     *
     * 并非所有实现都支持加速度约束，未支持的实现可忽略此参数。
     */
    virtual void setMaxAcceleration(const Eigen::VectorXd& max_acc) = 0;

    /**
     * @brief 重置轨迹生成器状态
     */
    virtual void reset() = 0;

    /**
     * @brief 获取维度
     * @return 轨迹维度 (关节数)
     */
    virtual int getDimension() const = 0;

protected:
    Eigen::VectorXd max_velocity_;     ///< 各维度最大速度
    Eigen::VectorXd max_acceleration_; ///< 各维度最大加速度

    Eigen::VectorXd target_pos_;       ///< 目标位置
    Eigen::VectorXd target_vel_;       ///< 目标速度

    Eigen::VectorXd desired_pos_;      ///< 期望位置输出
    Eigen::VectorXd desired_vel_;      ///< 期望速度输出

    Eigen::VectorXd error_;            ///< 位置误差
};

}  // namespace leju

#endif  // LEJU_TRAJECTORY_GENERATOR_BASE_H_
