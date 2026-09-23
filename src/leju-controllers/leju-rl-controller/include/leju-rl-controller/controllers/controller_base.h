#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "leju-rl-controller/inference/inference_model.h"
#include "leju-rl-controller/rl/multi_mode_arm_controller.h"
#include "leju-rl-controller/rl/rl_controller_types.h"
#include "leju-rl-controller/rl/waist_controller.h"
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
   * @brief 暂停控制器（kRunning -> kPaused），保留内部状态
   */
  virtual void pause();

  /**
   * @brief 恢复控制器（-> kRunning），内部调用 reset() 重置状态
   */
  virtual void resume();

  /**
   * @brief 执行一次控制更新（模板方法，不可覆写）
   *
   * Data flow:
   *
   * Robot -> [RobotState/ImuData] -> ControllerManager -> controller->update() -> [RobotCmd] -> Robot
   *                                                              |
   *     +--------------------------------------------------------------------------------------------+
   *     | 1. updateImpl() [子类: computeObservation -> infer -> computeActions -> updateRobotCmd]   |
   *     | 2. updateArmCommand()  [覆盖手臂关节]                                                      |
   *     | 3. updateWaistCommand() [覆盖腰部关节]                                                     |
   *     +--------------------------------------------------------------------------------------------+
   *
   * @param time 当前时间戳 [s]
   * @param state 当前关节状态
   * @param imu IMU 数据
   * @param[out] cmd 输出的控制指令
   * @return 成功返回 true
   */
  bool update(double time, const RobotState& state, const ImuData& imu, RobotCmd& cmd);

  /**
   * @brief 设置速度指令（用于遥控）
   * @param cmd 包含线速度和角速度的指令
   */
  virtual void setVelocityCommand(const VelocityCommand& cmd);
  virtual const VelocityCommand& getVelocityCommand() const { return velocity_cmd_; }

  /// @brief 清零速度滤波 等内部状态（Running 入口用，默认空实现）
  virtual void clearVelocityFilterState() {}

  /**
   * @brief 设置 cmd_stance 模式
   * @param stance 0=正常 AMP 行走态，1=站立/下蹲/弯腰态
   */
  void setCmdStanceMode(int stance);

  /**
   * @brief 在 0 和 1 之间切换 cmd_stance
   * @return 切换后的 cmd_stance
   */
  int toggleCmdStanceMode();

  /**
   * @brief 获取当前 cmd_stance 模式
   * @return 0=行走态，1=站立/下蹲/弯腰态
   */
  int getCmdStanceMode() const;

  /// @brief 深蹲守备：基于实际平滑高度命令，阻止退出 posture
  virtual bool isDeepSquatGuardActive() const { return false; }

  /// @brief 是否仍有残留平滑下蹲高度命令（smoothed 高度 < 0）
  virtual bool hasResidualStanceHeightCommand() const { return false; }

  /// @brief 转身退出深蹲守备：smoothed 下蹲高度 >= 阈值 且 |smoothed cmd_z| > 阈值
  virtual bool canTurnExitDeepSquatGuard(double pending_angular_z) const { return false; }

  /// @brief 行走模式下用于判定的 smoothed cmd_z（policy command unit，非 IMU）
  virtual double getSmoothedWalkingCmdZ(double pending_angular_z) const {
    return pending_angular_z;
  }

  /// @brief 转身退出守备的最小 |smoothed cmd_z| 阈值（policy command unit）
  virtual double getTurnExitGuardThreshold() const { return 0.5; }

  /**
   * @brief 将关节移动到默认位置
   * @param current_state 当前关节状态
   * @param elapse 过渡时间 [s]
   */
  virtual void moveToDefaultPos(const RobotState& current_state, double elapse);

  /**
   * @brief 设置默认姿态过渡的取消标志
   *
   * ControllerManager 在 requestStop()/stop() 时置位该标志，使控制器
   * 自身实现的阻塞式插值也能及时退出。未由管理器托管的控制器可不设置。
   */
  void setDefaultPoseStopFlag(const std::atomic<bool>* stop_flag) noexcept {
    default_pose_stop_flag_ = stop_flag;
  }

  /// @brief 获取控制器名称
  std::string getName() const;

  /// @brief 获取当前状态
  ControllerState getState() const;

  /// @brief 获取默认关节位置
  const array_t& getDefaultJointPos() const;

  /**
   * @brief 设置控制器配置文件路径
   * @param config_path 配置文件路径
   * @return 成功返回 true（如果控制器支持配置）
   *
   * 默认实现返回 false，子类如需支持配置加载应 override 此方法。
   */
  virtual bool setConfigPath(const std::string& config_path) { return false; }

  /**
   * @brief 设置部位关节名称（由 ControllerManager 从 SDK 获取后传入）
   * @param arm_joint_names 手臂关节名称列表（来自机器人硬件）
   * @param waist_joint_names 腰部关节名称列表（来自机器人硬件）
   *
   * 解耦 ControllerBase 和 SDK，避免直接依赖 GlobalRobot。
   */
  void setPartJointNames(const std::vector<std::string>& arm_joint_names,
                         const std::vector<std::string>& waist_joint_names) {
    arm_joint_names_ = arm_joint_names;
    waist_joint_names_ = waist_joint_names;
  }

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
  int getControlFrequency() const { return static_cast<int>(1.0 / loop_dt_); }

  /// @brief 策略切换时用于重建 q_target/kp/kd 的参考命令
  virtual const RobotCmd* getDualInferenceBlendReferenceCmd() const { return nullptr; }

  /// @brief 策略切换时策略关节的力矩限幅（电机空间）
  virtual const array_t* getDualInferenceTorqueLimits() const { return nullptr; }

  /// @brief 策略切换时哪些关节应使用 q_target 重算 tau
  virtual const array_i* getDualInferenceRecomputeMask() const { return nullptr; }

  /// @brief 判断当前是否处于站立状态（速度指令接近零）
  bool isStanding() const {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    return std::abs(velocity_cmd_.linear_x) < 0.01 &&
           std::abs(velocity_cmd_.linear_y) < 0.01 &&
           std::abs(velocity_cmd_.angular_z) < 0.01;
  }

  /**
   * @brief 开始播放 motion（用于舞蹈/动作播放控制器）
   * @return 成功返回 true，不支持则返回 false
   *
   * 默认实现返回 false，子类（如 GenericRLController）可 override 实现具体逻辑。
   */
  virtual bool startMotion() { return false; }

  // ===================== 部位控制器访问接口 =====================

  /**
   * @brief 获取手臂控制器
   * @return 手臂控制器指针，如果不存在则返回 nullptr
   */
  MultiModeArmController* getArmController() { return arm_controller_.get(); }

  /**
   * @brief 获取腰部控制器
   * @return 腰部控制器指针，如果不存在则返回 nullptr
   */
  WaistController* getWaistController() { return waist_controller_.get(); }

  /**
   * @brief 获取手臂关节索引（在 cmd.q 中的位置）
   * @return 手臂关节索引数组
   */
  const std::vector<int>& getArmJointIds() const { return arm_joint_ids_; }

  /**
   * @brief 获取手臂关节数量
   * @return 手臂关节数，未配置返回 0
   */
  size_t getArmJointCount() const { return arm_joint_ids_.size(); }

  /**
   * @brief 获取默认手臂姿态（从 default_joint_pos_ 中提取）
   * @return 默认手臂位置向量，未配置返回空向量
   */
  virtual Eigen::VectorXd getDefaultArmPos() const {
    return getConfigDefaultArmPos();
  }

  /**
   * @brief 获取腰部关节索引（SDK 电机索引）
   * @return 腰部关节索引列表
   */
  const std::vector<int>& getWaistJointIds() const { return waist_joint_ids_; }

  /**
   * @brief 获取腰部关节数量
   * @return 腰部关节数，未配置返回 0
   */
  size_t getWaistJointCount() const { return waist_joint_ids_.size(); }

  /**
   * @brief 获取默认腰部姿态（从 default_joint_pos_ 中提取）
   * @return 默认腰部位置向量，未配置返回空向量
   */
  virtual Eigen::VectorXd getDefaultWaistPos() const {
    return getConfigDefaultWaistPos();
  }

 protected:
  // ===================== 模板方法：子类必须实现 =====================

  /**
   * @brief 执行具体的控制更新逻辑（子类实现）
   *
   * 由 update() 调用，子类在此实现 RL 推理或其他控制逻辑。
   * 注意：current_state_ 和 current_imu_ 已由 update() 缓存。
   *
   * @param time 当前时间戳 [s]
   * @param state 当前关节状态
   * @param imu IMU 数据
   * @param[out] cmd 输出的控制指令
   * @return 成功返回 true
   */
  virtual bool updateImpl(double time, const RobotState& state,
                          const ImuData& imu, RobotCmd& cmd) = 0;

  // ===================== 部位控制器更新（有默认实现，子类可 override）=====================

  /**
   * @brief 更新手臂控制指令
   *
   * 默认实现：调用 arm_controller_->update() 并覆盖 cmd 中手臂关节的目标位置。
   * 子类可 override 实现自定义逻辑。
   *
   * @param[in,out] cmd 控制指令（会修改手臂关节部分）
   */
  virtual void updateArmCommand(RobotCmd& cmd);

  /**
   * @brief 更新腰部控制指令
   *
   * 默认实现：调用 waist_controller_->update() 并覆盖 cmd 中腰部关节的目标位置。
   * 子类可 override 实现自定义逻辑。
   *
   * @param[in,out] cmd 控制指令（会修改腰部关节部分）
   */
  virtual void updateWaistCommand(RobotCmd& cmd);

  /**
   * @brief 获取防抖后的站立/行走状态（非对称滞回）
   *
   * 进入站立需连续零速至少 100ms，恢复行走立即响应。
   * 防遥控器 SDL3 单帧丢零导致手臂/腰部控制模式反复切换。
   * v46/v52 等部位控制器 kAuto 模式：stance=1 表示站立并插值到 default pose。
   *
   * @return 1.0=站立, 0.0=行走
   */
  double getDebouncedStance() const;

  // ===================== 配置加载接口 =====================

  /**
   * @brief 加载 YAML 配置
   *
   * Base 实现解析通用配置（部位控制器开关、关节名称等）。
   * 子类 override 时应先调用 ControllerBase::loadConfig()。
   *
   * @param config_path 配置文件路径
   * @return 成功返回 true
   */
  virtual bool loadConfig(const std::string& config_path);

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

  // ===================== 部位控制器配置解析 =====================

  /**
   * @brief 初始化部位控制器
   *
   * 根据 enable_arm_controller_、enable_waist_controller_ 配置创建控制器实例。
   * 调用前必须先调用 buildPartJointMapping() 构建关节映射。
   */
  void initPartControllers();

  /**
   * @brief 构建部位关节映射
   *
   * 将 arm_joint_names_、waist_joint_names_ 映射到
   * SDK 电机索引（arm_joint_ids_、waist_joint_ids_）和策略索引。
   * 需要在 joint_names_ 和 policy_joint_ids_ 初始化后调用。
   */
  void buildPartJointMapping();

  /// @brief 按关节名查找其在 joint_names_ / default_joint_pos_ 中的策略索引
  int findPolicyJointIndex(const std::string& joint_name) const;

  /// @brief 从 YAML defaultJointState 提取手臂默认姿态（policy 空间，不受 motion 影响）
  Eigen::VectorXd getConfigDefaultArmPos() const;

  /// @brief 从 YAML defaultJointState 提取腰部默认姿态（policy 空间，不受 motion 影响）
  Eigen::VectorXd getConfigDefaultWaistPos() const;

  // ===================== 控制器标识 =====================
  std::string name_;                                          ///< 名称
  ControllerState state_ = ControllerState::kUninitialized;   ///< 当前状态

  // ===================== 时序配置 =====================
  int decimation_ = 1;            ///< 策略降采样因子
  double loop_dt_ = 0.001;        ///< 控制周期 [s]

  // ===================== 关节配置 =====================
  std::vector<std::string> joint_names_;  ///< 策略关节名称（按策略顺序）
  std::vector<int> policy_joint_ids_;     ///< 策略关节对应的 SDK 电机索引
  array_t default_joint_pos_;      ///< 策略 default pose [rad]（AMP 动作/观测中心）
  array_t start_stand_joint_pos_;  ///< 启动插值目标 [rad]；未配置时与 default_joint_pos_ 相同
  array_t joint_kp_;            ///< 位置增益
  array_t joint_kd_;            ///< 速度增益
  array_t joint_action_scale_;  ///< 动作缩放系数
  array_t joint_torque_limit_;  ///< 力矩限制 [Nm]
  array_i joint_control_mode_;  ///< 控制模式 (0=CST, 1=CSV, 2=CSP)
  array_t joint_direction_;     ///< 关节方向系数

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
  int cmd_stance_mode_ = 0;        ///< AMP cmd_stance: 0=行走态, 1=站立/下蹲/弯腰态

  // cmd_stance 防抖（手臂/腰部控制器共用，时间基准）
  // 任意频率下保证至少 100ms 零速才判定站立
  mutable std::chrono::steady_clock::time_point stance_zero_since_{};  ///< 首次零速时刻
  int stance_debounce_threshold_ = 5;  ///< 站立判定所需连续零速帧数（RL 策略观测用）

  double getCmdStanceValue() const;
  virtual double getPartControllerCmdStanceValue() const;

  // ===================== 传感器数据缓存 =====================
  RobotState current_state_;  ///< 缓存的关节状态
  ImuData current_imu_;       ///< 缓存的 IMU 数据

  // ===================== 控制步数 =====================
  uint64_t step_count_ = 0;   ///< 策略步数计数

  // ===================== 部位控制器开关 =====================
  bool enable_arm_controller_ = false;    ///< 是否启用手臂控制器
  bool enable_waist_controller_ = false;  ///< 是否启用腰部控制器

  // ===================== 部位关节映射 =====================
  //
  // 【两个空间的映射关系】
  //
  //                        Policy Space              SDK Space (硬件电机)
  //  (关节名称)            (RL策略数组)              (cmd.q 索引)
  //  ─────────────         ─────────────             ─────────────
  //                        joint_names_[]:
  //                        ┌───┬───────────┐
  //                        │ 0 │ leg_l1    │─────────┐
  //                        │ 1 │ leg_l2    │         │ policy_joint_ids_[]
  //                        │...│ ...       │         │ 将策略索引转为电机号
  //  arm_joint_names_:     │12 │ zarm_l1   │◄─┐      │
  //  ┌─────────────┐       │13 │ zarm_l2   │  │      ▼
  //  │ "zarm_l1"   │───────┼───┼───────────┤  │    cmd.q[]:
  //  │ "zarm_l2"   │       │14 │ zarm_l3   │  │    ┌───┬────────┐
  //  │ "zarm_l3"   │       │15 │ zarm_l4   │  │    │ 0 │ leg_l1 │
  //  │ "zarm_l4"   │       └───┴───────────┘  │    │...│ ...    │
  //  └─────────────┘              ▲           │    │12 │ zarm_l1│◄──写入
  //        │                      │           │    │13 │ zarm_l2│◄──写入
  //        │               arm_policy_start_  │    │14 │ zarm_l3│◄──写入
  //        │               idx_ = 12          │    │15 │ zarm_l4│◄──写入
  //        │                                  │    └───┴────────┘
  //        └──────────────────────────────────┘          ▲
  //          buildPartJointMapping() 查找                │
  //          手臂关节在 joint_names_ 中的位置      arm_joint_ids_[]
  //                                              = [12, 13, 14, 15]
  //
  // 【变量说明】
  //   arm_joint_names_     : 配置中指定的手臂关节名 (Config Space)
  //   arm_policy_start_idx_: 手臂在 default_joint_pos_[] 中的起始索引 (Policy Space)
  //   arm_joint_ids_       : 手臂关节对应的 cmd.q[] 索引 (SDK Space)
  //
  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> waist_joint_names_;

  /// @brief 管理器请求停止默认姿态过渡时的共享只读标志
  const std::atomic<bool>* default_pose_stop_flag_ = nullptr;

  bool defaultPoseStopRequested() const noexcept {
    return default_pose_stop_flag_ != nullptr &&
           default_pose_stop_flag_->load(std::memory_order_relaxed);
  }
  std::vector<int> arm_joint_ids_;
  std::vector<int> waist_joint_ids_;
  int arm_policy_start_idx_ = -1;
  int waist_policy_start_idx_ = -1;

  // ===================== 部位控制器 =====================
  std::unique_ptr<MultiModeArmController> arm_controller_;   ///< 手臂控制器
  std::unique_ptr<WaistController> waist_controller_;        ///< 腰部控制器
};

}  // namespace leju
