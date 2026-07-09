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
 * - 起点和终点速度为零
 * - 起点和终点加速度为零
 * - 位置、速度、加速度曲线连续
 */
class MinimumJerkInterpolator : public InterpolatorBase {
public:
    MinimumJerkInterpolator() = default;
    ~MinimumJerkInterpolator() override = default;

    /**
     * @brief 设置插值参数
     * @param start_pos 起始位置向量
     * @param end_pos 目标位置向量
     * @param duration 插值时长 (秒)
     * @return 设置成功返回 true
     */
    bool setup(const Eigen::VectorXd& start_pos,
               const Eigen::VectorXd& end_pos,
               double duration) override;

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
    /**
     * @brief 计算归一化时间的插值系数
     * @param tau 归一化时间 [0, 1]
     * @return s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
     */
    static double computeS(double tau);

    /**
     * @brief 计算归一化时间的速度系数
     * @param tau 归一化时间 [0, 1]
     * @return ds/dtau = 30*tau^2 - 60*tau^3 + 30*tau^4
     */
    static double computeDsDtau(double tau);

    Eigen::VectorXd start_pos_;    ///< 起始位置
    Eigen::VectorXd end_pos_;      ///< 目标位置
    double duration_{0.0};         ///< 插值时长
    bool initialized_{false};      ///< 初始化标志
};

}  // namespace leju

#endif  // LEJU_TRAJECTORY_MIN_JERK_INTERPOLATOR_H_
