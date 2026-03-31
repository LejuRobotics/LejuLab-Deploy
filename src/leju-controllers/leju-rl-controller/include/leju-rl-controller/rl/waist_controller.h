/**
 * @file waist_controller.h
 * @brief 腰部控制器：支持两种控制模式
 *
 * 纯计算类，无 DDS/ROS 订阅。
 *
 * 控制模式：
 * - kAuto (0): 自动模式（行走用 RL，站立使用三次多项式插值过渡到默认姿态）
 * - kExternal (1): 外部控制（使用低通滤波平滑外部输入）
 */

#ifndef LEJU_RL_WAIST_CONTROLLER_H_
#define LEJU_RL_WAIST_CONTROLLER_H_

#include <Eigen/Dense>

#include "leju-rl-controller/utils/low_pass_filter.h"

namespace leju {

/**
 * @brief 腰部控制模式
 */
enum class WaistControlMode {
    kAuto = 0,      ///< 自动模式（行走用 RL，站立三次多项式插值到默认姿态）
    kExternal = 1,  ///< 外部控制（低通滤波平滑外部输入）
};

/**
 * @brief 腰部控制器配置
 */
struct WaistControllerConfig {
    bool enabled = true;                     ///< 是否启用腰部控制

    // 三次多项式插值参数 (用于 kAuto 模式回零)
    double interpolation_velocity = 1.0;     ///< 插值最大速度 rad/s (约 57°/s)
    double min_duration = 0.25;               ///< 插值最短时长 s
    double max_duration = 2.0;               ///< 插值最长时长 s

    // 低通滤波器参数 (用于 kExternal 模式平滑输入)
    double filter_cutoff_freq = 2.0;         ///< 截止频率 (Hz)

    // 误差阈值
    double tracking_threshold = 0.02;        ///< 跟踪完成阈值 (rad)
};

/**
 * @brief 腰部控制器
 *
 * 输入: 控制模式, 当前腰部位置/速度, (可选) 外部目标
 * 输出: 期望腰部位置和速度
 */
class WaistController {
public:
    WaistController() = default;
    explicit WaistController(const WaistControllerConfig& config);

    void setConfig(const WaistControllerConfig& config);
    const WaistControllerConfig& getConfig() const { return config_; }

    /**
     * @brief 初始化腰部控制器
     * @param waist_joint_count 腰部关节数
     * @param default_waist_pos 默认腰部姿态 (rad)
     * @param dt 控制周期 (秒)
     */
    void init(int waist_joint_count,
              const Eigen::VectorXd& default_waist_pos,
              double dt);

    /**
     * @brief 设置控制模式
     * @param mode 目标控制模式
     * @note 使用 update() 中保存的当前位置进行状态初始化
     */
    void setMode(WaistControlMode mode);

    /**
     * @brief 获取当前控制模式
     */
    WaistControlMode getMode() const { return mode_; }

    /**
     * @brief 设置外部目标（kExternal 模式使用）
     * @param q 目标位置
     */
    void setExternalTarget(const Eigen::VectorXd& q);

    /**
     * @brief 更新腰部控制
     *
     * 数据流:
     * @code
     *                         +------------------+
     *   cmd_stance ---------> |                  |
     *   current_waist_pos --> |  WaistController | ---> desire_q (期望位置)
     *   current_waist_vel --> |     update()     | ---> desire_v (期望速度)
     *                         +------------------+
     *                                 |
     *                                 v
     *                          return: bool
     *                    (是否应覆盖 RL 输出)
     *
     *   kAuto 模式:
     *   +-----------+     +-------------+     +------------------+
     *   | cmd_stance| --> | stance=1?   | --> | 三次多项式插值   | --> desire_q/v
     *   +-----------+     | (站立)      |     | 到默认姿态       |     return true
     *                     +-------------+
     *                           |
     *                           | stance=0 (行走)
     *                           v
     *                     return false (使用 RL 输出)
     *
     *   kExternal 模式:
     *   +------------------+     +-------------+
     *   | raw_target_q_    | --> | 低通滤波    | --> desire_q/v
     *   | (setExternalTarget) | +-------------+     return true (始终覆盖)
     *   +------------------+
     * @endcode
     *
     * @param cmd_stance 1=站立, 0=行走（仅 kAuto 使用）
     * @param current_waist_pos 当前腰部位置 (rad)，会被保存供 setMode() 使用
     * @param current_waist_vel 当前腰部速度 (rad/s)
     * @param desire_q [out] 期望腰部位置
     * @param desire_v [out] 期望腰部速度
     * @return 是否应覆盖 cmd 的腰部部分
     *         - kAuto: 站立时返回 true，行走返回 false
     *         - kExternal: 始终返回 true
     */
    bool update(double cmd_stance,
                const Eigen::VectorXd& current_waist_pos,
                const Eigen::VectorXd& current_waist_vel,
                Eigen::VectorXd* desire_q,
                Eigen::VectorXd* desire_v);

    /**
     * @brief 检查是否正在插值过渡中
     */
    bool isInterpolating() const { return is_interpolating_; }

    /**
     * @brief 获取期望腰部位置
     */
    const Eigen::VectorXd& getDesiredPosition() const { return desire_waist_q_; }

    /**
     * @brief 获取期望腰部速度
     */
    const Eigen::VectorXd& getDesiredVelocity() const { return desire_waist_v_; }

    /**
     * @brief 重置控制器状态
     */
    void reset();

private:
    /**
     * @brief 启动插值到目标位置
     * @param start_pos 起始位置
     * @param target_pos 目标位置
     */
    void startInterpolation(const Eigen::VectorXd& start_pos,
                            const Eigen::VectorXd& target_pos);

    /**
     * @brief 更新插值状态
     * @param desire_q [out] 期望位置
     * @param desire_v [out] 期望速度
     * @return 插值是否完成
     */
    bool updateInterpolation(Eigen::VectorXd* desire_q, Eigen::VectorXd* desire_v);

    /**
     * @brief kAuto 模式更新
     */
    bool updateAuto(double cmd_stance,
                    const Eigen::VectorXd& current_waist_pos,
                    const Eigen::VectorXd& current_waist_vel,
                    Eigen::VectorXd* desire_q,
                    Eigen::VectorXd* desire_v);

    /**
     * @brief kExternal 模式更新
     */
    bool updateExternal(const Eigen::VectorXd& current_waist_pos,
                        const Eigen::VectorXd& current_waist_vel,
                        Eigen::VectorXd* desire_q,
                        Eigen::VectorXd* desire_v);

    // 配置
    WaistControllerConfig config_;
    int waist_joint_count_{0};
    Eigen::VectorXd default_waist_pos_;
    double dt_{0.001};

    // 当前状态
    WaistControlMode mode_{WaistControlMode::kAuto};
    Eigen::VectorXd current_waist_pos_;          ///< 当前腰部位置（从 update 保存）
    Eigen::VectorXd desire_waist_q_;             ///< 期望腰部位置
    Eigen::VectorXd desire_waist_v_;             ///< 期望腰部速度

    // 三次多项式插值状态 (用于 kAuto 模式)
    Eigen::VectorXd interpolation_start_pos_;
    Eigen::VectorXd interpolation_target_pos_;
    double interpolation_time_{0.0};             ///< 内部插值时间 (秒)
    double interpolation_duration_{0.5};
    bool is_interpolating_{false};

    // 低通滤波器 (用于 kExternal 模式)
    LowPassFilter filter_;
    bool is_filtering_{false};

    // kExternal 状态
    Eigen::VectorXd raw_target_q_;               ///< 原始外部目标位置
    bool target_received_{false};                ///< 是否已收到外部目标

    bool initialized_{false};
};

}  // namespace leju

#endif  // LEJU_RL_WAIST_CONTROLLER_H_
