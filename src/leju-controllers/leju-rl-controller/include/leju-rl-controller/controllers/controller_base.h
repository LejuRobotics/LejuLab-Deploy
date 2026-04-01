#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "leju-rl-controller/inference/inference_model.h"
#include "leju-rl-controller/rl/rl_controller_types.h"
#include "lejusdk-lowlevel/leju_sdk.h"

namespace leju {

/**
 * @brief 控制器抽象基类
 *
 * 定义控制器生命周期接口，所有控制器（如 GenericRLController）需继承此类。
 */
class ControllerBase {
 public:
  ControllerBase() = default;
  virtual ~ControllerBase() = default;

  // ===================== 纯虚接口 =====================

  /**
   * @brief 初始化控制器
   * @return 成功返回 true
   */
  virtual bool initialize() = 0;

  /**
   * @brief 停止控制器
   */
  virtual void stop();

  /**
   * @brief 重置内部状态
   *
   * 由 resume() 自动调用，子类按需 override。
   */
  virtual void reset();

  /**
   * @brief 执行一次控制更新
   *
   * Robot -> [RobotState/ImuData] -> ControllerManager -> controller->update() -> [RobotCmd] -> Robot
   *                                                              |
   *            +-----------------------------------------------------------------------------------------+
   *            | computeObservation() -> model_->infer() -> computeActions() -> updateRobotCmd()         |
   *            +-----------------------------------------------------------------------------------------+
   *
   * @param time 当前时间戳 [s]
   * @param state 当前关节状态
   * @param imu IMU 数据
   * @param[out] cmd 输出的控制指令
   * @return 成功返回 true
   */
  virtual bool update(double time, const RobotState& state, const ImuData& imu,
                      RobotCmd& cmd) = 0;

  /**
   * @brief 暂停控制器（kRunning -> kPaused），保留内部状态
   */
  virtual void pause();

  /**
   * @brief 恢复控制器（-> kRunning），内部调用 reset() 重置状态
   */
  virtual void resume();

  /**
   * @brief 设置速度指令（用于遥控）
   * @param cmd 包含线速度和角速度的指令
   */
  virtual void setVelocityCommand(const VelocityCommand& cmd);

  /**
   * @brief 将关节移动到默认位置
   * @param current_state 当前关节状态
   * @param elapse 过渡时间 [s]
   */
  virtual void moveToDefaultPos(const RobotState& current_state, double elapse);

  /// @brief 获取控制器名称
  std::string getName() const;

  /// @brief 获取当前状态
  ControllerState getState() const;

  /// @brief 获取默认关节位置
  const array_t& getDefaultJointPos() const;

  /// @brief 是否正在运行
  bool isActive() const;

  /// @brief 是否处于暂停状态
  bool isPaused() const;

  /// @brief 是否已初始化
  bool isInitialized() const;

  /**
   * @brief 等待到下一个控制周期
   *
   * 默认实现按 control_frequency_ 计算周期，使用 sleep_until 精确等待。
   * 子类可覆写实现自定义节拍（变频控制）。
   *
   * @param cycle_start 本次循环开始时间点
   */
  virtual void waitNextCycle(std::chrono::steady_clock::time_point cycle_start);

  /// @brief 获取控制频率 [Hz]
  int getControlFrequency() const { return control_frequency_; }

  /**
   * @brief 接收手柄原生输入
   *
   * 子类按自己的业务逻辑解读按键和摇杆含义。
   * 默认空实现。
   *
   * @param joy 当前帧手柄数据
   * @param prev_buttons 上一帧按钮状态（用于边缘检测）
   */
  virtual void onJoyInput(const JoyData& joy, const JoyData::Buttons& prev_buttons);

 protected:
  /**
   * @brief 加载 YAML 配置
   * @param config_path 配置文件路径
   * @return 成功返回 true
   */
  virtual bool loadConfig(const std::string& config_path) = 0;

  /**
   * @brief 加载策略模型
   * @param policy_path 模型文件路径
   * @return 成功返回 true
   */
  virtual bool loadPolicy(const std::string& policy_path) = 0;

  /// @brief 根据传感器数据构建观测向量
  virtual void computeObservation() = 0;

  /// @brief 执行策略推理，计算动作
  virtual void computeActions() = 0;

  /**
   * @brief 将动作转换为机器人指令
   * @param[out] cmd 输出指令
   */
  virtual void updateRobotCmd(RobotCmd& cmd) = 0;

  // ===================== 控制器标识 =====================
  std::string name_;                                          ///< 名称
  ControllerState state_ = ControllerState::kUninitialized;   ///< 当前状态

  // ===================== 时序配置 =====================
  int control_frequency_ = 1000;  ///< 控制频率 [Hz]
  int decimation_ = 1;            ///< 策略降采样因子

  // ===================== 关节配置 =====================
  array_t default_joint_pos_;   ///< 默认关节位置 [rad]
  array_t joint_kp_;            ///< 位置增益
  array_t joint_kd_;            ///< 速度增益
  array_t joint_action_scale_;  ///< 动作缩放系数
  array_t joint_torque_limit_;  ///< 力矩限制 [Nm]
  array_i joint_control_mode_;  ///< 控制模式 (0=CST, 1=CSV, 2=CSP)

  // ===================== 观测与动作 =====================
  array_t observations_;   ///< 当前观测向量
  array_t actions_;        ///< 当前动作输出
  array_t last_actions_;   ///< 上一帧动作
  matrix_t obs_history_;   ///< 观测历史缓冲

  // ===================== 模型推理 =====================
  std::unique_ptr<InferenceModel> model_;  ///< 策略模型

  // ===================== 指令输入 =====================
  mutable std::mutex cmd_mutex_;  ///< 速度指令锁
  VelocityCommand velocity_cmd_;  ///< 当前速度指令
};

}  // namespace leju
