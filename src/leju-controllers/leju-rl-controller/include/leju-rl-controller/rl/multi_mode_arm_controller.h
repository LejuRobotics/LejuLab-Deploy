/**
 * @file multi_mode_arm_controller.h
 * @brief 多模式手臂控制器：支持三种控制模式
 *
 * 纯计算类，无 DDS 订阅。
 *
 * 控制模式：
 * - kKeepPose (0): 保持当前姿态
 * - kAuto (1): 自动模式（行走用 RL，站立插值到默认姿态）
 * - kExternal (2): 外部控制（两阶段机制：接入 -> 跟踪）
 */

#ifndef LEJU_RL_MULTI_MODE_ARM_CONTROLLER_H_
#define LEJU_RL_MULTI_MODE_ARM_CONTROLLER_H_

#include <Eigen/Dense>

#include "leju-rl-controller/utils/low_pass_filter.h"
#include "leju-rl-controller/trajectory/interpolator/minimum_jerk_interpolator.h"
#include "leju-rl-controller/trajectory/generator/velocity_limited_generator.h"

namespace leju {

/**
 * @brief 手臂控制模式
 */
enum class ArmControlMode {
    kKeepPose = 0,   ///< 保持当前姿态
    kAuto = 1,       ///< 自动模式（行走用 RL，站立插值到默认姿态）
    kExternal = 2,   ///< 外部控制（两阶段机制）
};

/**
 * @brief 多模式手臂控制器配置
 */
struct MultiModeArmControllerConfig {
    bool enabled = true;                     ///< 是否启用手臂控制

    // MODE_AUTO 参数
    double interpolation_velocity = 1.0;     ///< 插值速度 (rad/s)
    double min_duration = 0.2;               ///< 最小插值时长 (秒)
    double max_duration = 2.0;               ///< 最大插值时长 (秒)

    // MODE_EXTERNAL 参数
    double approach_velocity = 1.0;          ///< 接入阶段最大速度 (rad/s)
    double tracking_velocity = 2.0;          ///< 跟踪阶段最大速度 (rad/s)
    double approach_threshold = 0.05;        ///< 接入完成阈值 (rad)
    double filter_cutoff_freq = 5.0;         ///< 目标滤波截止频率 (Hz)

    // 模式切换参数
    double mode_interpolation_velocity = 1.5; ///< 模式切换插值速度 (rad/s)，用于动态计算过渡时长
    double mode_transition_duration = 1.0;   ///< 模式切换默认过渡时长 (秒)，当无法计算时使用
};

/**
 * @brief 多模式手臂控制器
 *
 * 输入: 控制模式, 当前手臂位置/速度, (可选) 外部目标
 * 输出: 期望手臂位置和速度
 */
class MultiModeArmController {
public:
    MultiModeArmController() = default;
    explicit MultiModeArmController(const MultiModeArmControllerConfig& config);

    void setConfig(const MultiModeArmControllerConfig& config);
    const MultiModeArmControllerConfig& getConfig() const { return config_; }

    /**
     * @brief 初始化手臂索引与默认姿态
     * @param arm_joint_count 手臂关节数
     * @param default_arm_pos 默认手臂姿态 (rad)
     * @param dt 控制周期 (秒)
     */
    void init(int arm_joint_count,
              const Eigen::VectorXd& default_arm_pos,
              double dt);

    /**
     * @brief 设置控制模式
     * @param mode 目标控制模式
     * @note 使用 update() 中保存的当前位置进行平滑过渡
     */
    void setMode(ArmControlMode mode);

    /**
     * @brief 获取当前控制模式
     * @note 如果正在模式过渡中，返回目标模式（pending_mode_）
     */
    ArmControlMode getMode() const { return mode_transitioning_ ? pending_mode_ : mode_; }

    /**
     * @brief 设置外部目标（MODE_EXTERNAL 使用）
     * @param q 目标位置
     * @param v 目标速度（可选，当前实现忽略）
     */
    void setExternalTarget(const Eigen::VectorXd& q,
                           const Eigen::VectorXd& v = Eigen::VectorXd());

    /**
     * @brief 更新手臂控制
     *
     * @code
     *   [输入]                             [输出]
     *   cmd_stance ---+
     *   current_pos --+--> update() --+--> desire_q
     *   current_vel --+               +--> desire_v
     *                                 +--> return: 是否覆盖 RL
     * @endcode
     *
     * @param cmd_stance 1=站立, 0=行走（仅 MODE_AUTO 使用）
     * @param current_arm_pos 当前手臂位置 (rad)，会被保存供 setMode() 使用
     * @param current_arm_vel 当前手臂速度 (rad/s)
     * @param desire_q [out] 期望手臂位置
     * @param desire_v [out] 期望手臂速度
     * @return 是否应覆盖 cmd 的手臂部分
     *         - MODE_KEEP_POSE: 始终返回 true
     *         - MODE_AUTO: 站立时返回 true，行走时返回 false
     *         - MODE_EXTERNAL: 始终返回 true
     */
    bool update(double cmd_stance,
                const Eigen::VectorXd& current_arm_pos,
                const Eigen::VectorXd& current_arm_vel,
                Eigen::VectorXd* desire_q,
                Eigen::VectorXd* desire_v);

    /**
     * @brief 检查是否正在模式切换过渡中
     */
    bool isModeTransitioning() const { return mode_transitioning_; }

    /**
     * @brief 检查外部模式是否在接入阶段
     */
    bool isApproaching() const { return is_approaching_; }

    /**
     * @brief 获取期望手臂位置
     */
    const Eigen::VectorXd& getDesiredPosition() const { return desire_arm_q_; }

    /**
     * @brief 获取期望手臂速度
     */
    const Eigen::VectorXd& getDesiredVelocity() const { return desire_arm_v_; }

private:
    /**
     * @brief 启动模式切换过渡
     * @param from_q 起始位置
     * @param to_q 目标位置
     */
    void startModeTransition(const Eigen::VectorXd& from_q, const Eigen::VectorXd& to_q);

    /**
     * @brief 更新模式切换过渡
     * @param desire_q [out] 期望位置
     * @param desire_v [out] 期望速度
     * @return 过渡是否完成
     */
    bool updateModeTransition(Eigen::VectorXd* desire_q, Eigen::VectorXd* desire_v);

    /**
     * @brief MODE_KEEP_POSE 更新
     */
    bool updateKeepPose(const Eigen::VectorXd& current_arm_pos,
                        Eigen::VectorXd* desire_q,
                        Eigen::VectorXd* desire_v);

    /**
     * @brief MODE_AUTO 更新
     */
    bool updateAuto(double cmd_stance,
                    const Eigen::VectorXd& current_arm_pos,
                    const Eigen::VectorXd& current_arm_vel,
                    Eigen::VectorXd* desire_q,
                    Eigen::VectorXd* desire_v);

    /**
     * @brief MODE_EXTERNAL 更新
     */
    bool updateExternal(const Eigen::VectorXd& current_arm_pos,
                        const Eigen::VectorXd& current_arm_vel,
                        Eigen::VectorXd* desire_q,
                        Eigen::VectorXd* desire_v);

    // 配置
    MultiModeArmControllerConfig config_;
    int arm_joint_count_{0};
    Eigen::VectorXd default_arm_pos_;
    double dt_{0.001};

    // 当前状态
    ArmControlMode mode_{ArmControlMode::kAuto};
    Eigen::VectorXd current_arm_pos_;            ///< 当前手臂位置（从 update 保存）
    Eigen::VectorXd desire_arm_q_;               ///< 期望手臂位置
    Eigen::VectorXd desire_arm_v_;               ///< 期望手臂速度
    Eigen::VectorXd keep_pose_q_;                ///< MODE_KEEP_POSE 保持的位置

    // MODE_AUTO 状态
    MinimumJerkInterpolator auto_interpolator_;  ///< 自动模式插值器
    double auto_interpolator_time_{0.0};         ///< 插值器时间
    bool is_interpolating_to_default_{false};    ///< 是否正在插值到默认姿态

    // MODE_EXTERNAL 状态
    VelocityLimitedGenerator external_generator_;///< 外部模式轨迹生成器
    LowPassFilter target_filter_;                ///< 目标位置滤波器
    Eigen::VectorXd raw_target_q_;               ///< 原始外部目标位置
    Eigen::VectorXd filtered_target_q_;          ///< 滤波后的目标位置
    bool is_approaching_{true};                  ///< 是否在接入阶段
    bool target_received_{false};                ///< 是否已收到外部目标

    // 模式切换过渡状态
    MinimumJerkInterpolator transition_interpolator_;  ///< 模式切换插值器
    double transition_time_{0.0};                      ///< 过渡时间
    bool mode_transitioning_{false};                   ///< 是否正在模式切换
    ArmControlMode pending_mode_{ArmControlMode::kAuto}; ///< 切换目标模式

    bool initialized_{false};
};

}  // namespace leju

#endif  // LEJU_RL_MULTI_MODE_ARM_CONTROLLER_H_
