/**
 * @file minimum_jerk_interpolator.h
 * @brief 五次多项式插值器
 *
 * 实现五次多项式插值，确保起点和终点的位置、速度、加速度连续。
 * 公式: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5, tau = t/T
 */

#ifndef LEJU_TRAJECTORY_MIN_JERK_INTERPOLATOR_H_
#define LEJU_TRAJECTORY_MIN_JERK_INTERPOLATOR_H_

#include "leju-rl-controller/trajectory/interpolator/interpolator_base.h"

namespace leju {

/**
 * @class MinimumJerkInterpolator
 * @brief 最小急动度插值器
 *
 * 使用五次多项式实现平滑插值，特性：
 * - 默认起止速度、加速度为零（标准 min-jerk）
 * - 支持指定起止速度的通用五次多项式（用于实时 re-plan 时速度连续，避免速度突变）
 * - 位置、速度、加速度曲线连续
 */
class MinimumJerkInterpolator : public InterpolatorBase {
public:
    MinimumJerkInterpolator() = default;
    ~MinimumJerkInterpolator() override = default;

    /**
     * @brief 设置插值参数（起止速度为零的标准 min-jerk）
     * @param start_pos 起始位置向量
     * @param end_pos 目标位置向量
     * @param duration 插值时长 (秒)
     * @return 设置成功返回 true
     */
    bool setup(const Eigen::VectorXd& start_pos,
               const Eigen::VectorXd& end_pos,
               double duration) override;

    /**
     * @brief 设置插值参数（通用五次多项式，支持起止速度）
     *
     * 在标准 min-jerk 基础上放宽起始速度约束：满足
     *   p(0)=start_pos, v(0)=start_vel, a(0)=0,
     *   p(T)=end_pos,   v(T)=end_vel,   a(T)=0。
     * 用于插值过程中目标实时变化时的 re-plan：把当前插值输出位置/速度作为起点传入，
     * 保证新插值轨迹与旧轨迹在起点处位置、速度连续（消除 re-plan 时的速度突变）。
     *
     * @param start_pos 起始位置向量
     * @param start_vel 起始速度向量（需与 start_pos 同维度）
     * @param end_pos 目标位置向量
     * @param end_vel 目标速度向量（需与 end_pos 同维度）
     * @param duration 插值时长 (秒)
     * @return 设置成功返回 true
     */
    bool setup(const Eigen::VectorXd& start_pos,
               const Eigen::VectorXd& start_vel,
               const Eigen::VectorXd& end_pos,
               const Eigen::VectorXd& end_vel,
               double duration);

    /**
     * @brief 计算指定时刻的插值结果
     * @param t 当前时间 (从 setup 调用开始计时)
     * @param[out] pos 输出位置
     * @param[out] vel 输出速度
     * @return 评估成功返回 true
     */
    bool evaluate(double t, Eigen::VectorXd& pos, Eigen::VectorXd& vel) const override;

    /**
     * @brief 检查插值是否完成
     * @param t 当前时间
     * @return 当 t >= duration 时返回 true
     */
    bool isFinished(double t) const override;

    /**
     * @brief 重置插值器状态
     */
    void reset() override;

    /**
     * @brief 获取插值总时长
     */
    double getDuration() const override { return duration_; }

    /**
     * @brief 获取维度
     */
    int getDimension() const override { return static_cast<int>(start_pos_.size()); }

    /**
     * @brief 检查插值器是否已初始化
     */
    bool isInitialized() const override { return initialized_; }

    /**
     * @brief 获取插值器类型名称
     */
    const char* getName() const override;

private:
    Eigen::VectorXd start_pos_;    ///< 起始位置
    Eigen::VectorXd start_vel_;    ///< 起始速度（默认零）
    Eigen::VectorXd end_pos_;      ///< 目标位置
    Eigen::VectorXd end_vel_;      ///< 目标速度（默认零）
    double duration_{0.0};         ///< 插值时长
    bool initialized_{false};      ///< 初始化标志
};

}  // namespace leju

#endif  // LEJU_TRAJECTORY_MIN_JERK_INTERPOLATOR_H_
