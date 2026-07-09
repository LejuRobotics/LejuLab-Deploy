/**
 * @file interpolator_base.h
 * @brief 插值器抽象基类
 *
 * 定义插值器的通用接口，支持扩展不同的插值方法：
 * - MinimumJerkInterpolator (五次多项式/最小急动度)
 * - CubicInterpolator (三次多项式，未来)
 * - BezierInterpolator (贝塞尔曲线，未来)
 */

#ifndef LEJU_TRAJECTORY_INTERPOLATOR_BASE_H_
#define LEJU_TRAJECTORY_INTERPOLATOR_BASE_H_

#include <Eigen/Dense>

namespace leju {

/**
 * @class InterpolatorBase
 * @brief 插值器抽象基类
 *
 * 定义所有插值器必须实现的核心接口。
 */
class InterpolatorBase {
public:
    virtual ~InterpolatorBase() = default;

    /**
     * @brief 设置插值参数
     * @param start_pos 起始位置向量
     * @param end_pos 目标位置向量
     * @param duration 插值时长 (秒)
     * @return 设置成功返回 true
     */
    virtual bool setup(const Eigen::VectorXd& start_pos,
                       const Eigen::VectorXd& end_pos,
                       double duration) = 0;

    /**
     * @brief 计算指定时刻的插值结果
     * @param t 当前时间 (从 setup 调用开始计时)
     * @param[out] pos 输出位置
     * @param[out] vel 输出速度
     * @return 评估成功返回 true
     */
    virtual bool evaluate(double t,
                          Eigen::VectorXd& pos,
                          Eigen::VectorXd& vel) const = 0;

    /**
     * @brief 重置插值器状态
     */
    virtual void reset() = 0;

    /**
     * @brief 检查插值是否完成
     * @param t 当前时间
     * @return 当 t >= duration 时返回 true
     */
    virtual bool isFinished(double t) const = 0;

    /**
     * @brief 检查插值器是否已初始化
     */
    virtual bool isInitialized() const = 0;

    /**
     * @brief 获取插值总时长
     */
    virtual double getDuration() const = 0;

    /**
     * @brief 获取维度
     */
    virtual int getDimension() const = 0;

    /**
     * @brief 获取插值器类型名称
     */
    virtual const char* getName() const = 0;
};

}  // namespace leju

#endif  // LEJU_TRAJECTORY_INTERPOLATOR_BASE_H_
