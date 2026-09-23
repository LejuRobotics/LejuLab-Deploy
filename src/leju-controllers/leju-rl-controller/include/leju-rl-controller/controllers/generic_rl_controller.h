#pragma once

#include <deque>
#include <map>
#include <memory>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "leju-rl-controller/controllers/controller_base.h"
#include "leju-rl-controller/motion/motion_trajectory.h"
#include "leju-rl-controller/rl/rl_controller_types.h"
#include "leju-rl-controller/utils/velocity_stop_filter.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-lowlevel/topic_logger.h"
#include "lejusdk-utils/robot_version.hpp"

namespace leju {

/// @brief 通用 RL 控制器
///
/// 基于模块化组件实现强化学习策略控制：
/// - OpenVINOModel 负责策略推理
/// - MotionTrajectory 提供参考动作轨迹（可选）
class GenericRLController : public ControllerBase {
 public:
  GenericRLController(const RobotVersion& version, const std::string& name);
  ~GenericRLController() override;

  // ==================== ControllerBase 接口 ====================

  /// @brief 初始化（加载配置、模型、轨迹数据）
  bool initialize() override;
  /// @brief 重置内部状态
  void reset() override;
  /// @brief 清零速度滤波状态
  void clearVelocityFilterState() override;
  /// @brief 移动到默认关节位置
  void moveToDefaultPos(const RobotState& current_state, double elapse) override;

  // ==================== 配置 ====================

  /// @brief 设置配置文件路径，必须在 initialize() 前调用
  bool setConfigPath(const std::string& config_path) override {
    config_path_ = config_path;
    return true;
  }

  /// @brief 设置 URDF 路径（用于手臂重力补偿），必须在 initialize() 前调用
  void setUrdfPath(const std::string& urdf_path) { urdf_path_ = urdf_path; }

  // ==================== Motion 播放 ====================

  /// @brief 开始或重新播放当前 motion
  bool startMotion();
  /// @brief 写入本次起播用于倒地起身轨迹选择的最新 IMU 快照
  void setMotionStartImu(const ImuData& imu) { current_imu_ = imu; }
  /// @brief 切换到指定 motion 并开始播放
  bool startMotion(const std::string& name);
  /// @brief 停止 motion 播放
  bool stopMotion();
  /// @brief 两阶段起身：插值到 READY 目标并保持（非阻塞，在 update 中完成）
  /// @param max_joint_velocity 最大关节速度 [rad/s]；<=0 时使用配置值（自适应时长 = max|Δq|/v）
  bool prepareToMotionStart(const RobotState& current_state,
                            const ImuData& imu,
                            double max_joint_velocity = -1.0);
  /// @brief START 后待机：锁当前关节角，不跑策略；仍需 LB+RB+X 两阶段才起播
  bool enterStandbyHold(const RobotState& current_state);
  /// @brief 是否正在插值到 CSV 首帧
  bool isPreparingMotionStart() const { return motion_prep_state_ == MotionPrepState::kInterpolating; }
  /// @brief 是否已保持在 CSV 首帧（可第二次按键起播）
  bool isHoldingMotionStart() const { return motion_prep_state_ == MotionPrepState::kHolding; }
  /// @brief 是否处于 START 后待机 hold（尚未进入两阶段 prepare）
  bool isStandbyHolding() const { return motion_prep_state_ == MotionPrepState::kStandbyHold; }
  /// @brief 获取全部可用 motion 名称
  std::vector<std::string> getMotionNames() const;
  /// @brief 获取当前 motion 名称
  std::string getCurrentMotionName() const;
  /// @brief 检查是否正在播放 motion
  bool isMotionPlaying() const { return motion_playing_; }
  /// @brief 舞蹈播完自动切回稳定控制器是否启用
  bool isAutoSwitchBackEnabled() const { return auto_switch_back_enabled_; }
  /// @brief 舞蹈播完自动切回的目标控制器名
  const std::string& getAutoSwitchBackTarget() const { return auto_switch_back_target_; }
  /// @brief 本控制周期 motion 是否刚播放完毕（边沿标志，供 ControllerManager 自动切回）
  bool isMotionJustFinished() const { return motion_just_finished_; }
  /// @brief 消费播完边沿标志（ControllerManager 触发切回后调用）
  void consumeMotionFinished() { motion_just_finished_ = false; }
  /// @brief 设置/查询 M1/M2 motion 播放期间屏蔽摇杆行走
  void setBlockVelocityInMotion(bool block) { block_velocity_in_motion_ = block; }
  bool isBlockingVelocityInMotion() const { return block_velocity_in_motion_; }

  /// @brief 倒地起身细粒度阶段（对齐 kuavo FallStandController::FallStandState）
  /// @return 0=FALL_DOWN(瘫软) 1=INTERPOLATING(prepare插值中)
  ///         2=READY_FOR_STAND_UP(READY 目标保持) 3=STAND_UP(起身播放中) 4=STANDING(轨迹播完)
  int getFallStandState() const;

  /// @brief 获取策略计算得到的参考关节位置q_ref 
  const RobotCmd* getDualInferenceBlendReferenceCmd() const override {
    return blend_reference_cmd_.isValid() ? &blend_reference_cmd_ : nullptr;
  }

  /// @brief 获取策略的力矩限幅 
  const array_t* getDualInferenceTorqueLimits() const override {
    return blend_reference_cmd_.isValid() ? &blend_torque_limits_ : nullptr;
  }

  /// @brief 重算掩码，对于某些关节（如手臂/腰部）在策略切换时不重算 tau，而是直接插值原始 tau 输出 
  const array_i* getDualInferenceRecomputeMask() const override {
    return blend_reference_cmd_.isValid() ? &blend_recompute_mask_ : nullptr;
  }

  /// @brief 获取最近一次观测提交/推理完成时间戳（steady time, s）
  double getLastObservationSubmitTimeSec() const {
    return last_observation_submit_time_sec_.load();
  }
  double getLastInferenceFinishTimeSec() const {
    return last_inference_finish_time_sec_.load();
  }

 protected:
  // ==================== 模板方法实现 ====================

  /// @brief 执行 RL 推理更新（由 ControllerBase::update() 调用）
  bool updateImpl(double time, const RobotState& state, const ImuData& imu,
                  RobotCmd& cmd) override;

  // ==================== 可重写方法 ====================

  /// @brief 加载 YAML 配置文件
  bool loadConfig(const std::string& config_path) override;
  /// @brief 加载 ONNX 策略模型
  bool loadPolicy(const std::string& policy_path) override;
  /// @brief 构建观测向量
  void computeObservation() override;
  /// @brief 执行策略推理，计算动作
  void computeActions() override;
  /// @brief 将动作转换为机器人控制指令
  void updateRobotCmd(RobotCmd& cmd) override;

  /// @brief 更新手臂控制指令（重写以发布 arm_mode 话题）
  void updateArmCommand(RobotCmd& cmd) override;

  /// @brief 更新腰部控制指令（重写以发布 waist_mode 话题）
  void updateWaistCommand(RobotCmd& cmd) override;

  /// @brief 部位控制器使用的 cmd_stance（v17 amp_hand 跟遥控器；其余走基类零速防抖）
  double getPartControllerCmdStanceValue() const override;

  /// 专用策略可覆写固定模型观测长度；0 表示使用通用 obs 配置计算结果。
  virtual int customObservationSize() const { return 0; }
  int policyJointCount() const { return policy_joint_count_; }

  /// 最近一次已完成推理的动作快照，语义与源端 getCurrentAction() 一致。
  /// 注意它不是 last_actions_（后者仅用于动作替换前的 bookkeeping）。
  array_t currentActionSnapshot() const { return getActionsSnapshot(); }

  /// 供专用 RL 控制器复用通用异步推理的安全动作缓存。
  virtual bool inferActions(const array_t& observation, array_t& inferred_actions);
  void updateActionCache(const array_t& new_actions);

  /// @brief 获取默认手臂姿态（有 motion 时返回第一帧位置）
  Eigen::VectorXd getDefaultArmPos() const override;

  /// @brief 获取默认腰部姿态（有 motion 时返回第一帧位置）
  Eigen::VectorXd getDefaultWaistPos() const override;

 protected:
  // Specialized controllers with custom observation stacks (e.g. depth)
  // can reuse the exact command scaling/filtering path without exposing the
  // implementation helpers to unrelated callers.
  void refreshVelocityCommandCache() { updateVelocityCommandCache(); }
  array_t filteredVelocityCommandSnapshot() const {
    return getVelocityCommands();
  }

 private:
  // ==================== 内部方法 ====================

  /// @brief 构建策略关节到 SDK 电机的索引映射
  bool buildJointMapping();
  /// @brief 加载全部 motion 轨迹文件
  void loadMotionTrajectories(const std::string& config_dir);
  /// @brief 获取策略关节位置（相对默认位置的偏移）
  array_t getPolicyJointPos() const;
  /// @brief 获取策略关节速度
  array_t getPolicyJointVel() const;
  /// @brief 计算缩放后的原始速度指令（使用已加锁拷贝的速度与 stance 模式）
  array_t getRawVelocityCommands(const VelocityCommand& vel_cmd,
                                 int cmd_stance_mode) const;
  array_t getRawVelocityCommands() const;
  /// @brief policy 索引是否在 policy_joint_ids_ 有效范围内
  bool isValidPolicyMotorIdx(int policy_idx) const;
  /// @brief 按固定 policy_dt 平滑行走模式的原始 cmd_x（缩放前，单位 m/s）
  double smoothLinearXCommand(double target);
  /// @brief 两阶段停车减速：先快速降到保持速度，保持 hold_time 后缓慢减速到 0（目前前后方向是共用的）
  double applyCmdXTwoPhaseDecel(double previous, double target);
  /// @brief 更新滤波后的速度指令缓存
  void updateVelocityCommandCache();
  /// @brief standing 模式下起身高度命令线性限速（max_standup_change）
  void applyStanceHeightStandUpSmoothing(double& stance_height_cmd);
  /// @brief posture 模式下钳制下蹲关节目标角（膝角/髋 pitch）
  void applySquatJointAngleLimits(array_t& q_target) const;
  /// @brief 外部手控时 leg1 角上限较 YAML 再收紧 kSquatLeg1ExternalArmReductionRad_
  double getEffectiveSquatLeg1AngleMax() const;
  /// @brief 获取滤波后的速度指令缓存
  array_t getVelocityCommands() const;
  /// @brief 深蹲守备是否生效（leg_l4/leg_r4 膝角均超过阈值）
  bool isDeepSquatGuardActive() const override;
  /// @brief 控制器端下蹲守备：深蹲时屏蔽走/转，物理起身后自动退出 posture
  void applySquatPostureDefense();
  /// @brief 计算 motion 目标姿态的相对旋转矩阵
  array_t getMotionAnchorOriB() const;
  /// @brief 计算基座角速度观测项
  array_t getBaseAngVel() const;
  /// @brief 计算投影重力观测项
  array_t getProjectedGravity() const;
  /// @brief 外部手臂接管是否生效（enable_amp_arm_enhance_v17 + 手臂模式非 Auto）
  bool isExternalArmControlActive() const;
  /// @brief 纯侧移命令判据（|cmd_x|<0.2, |cmd_angz|<0.2, |cmd_y|>0.1）
  bool isLateralMoveCommand() const;
  /// @brief |cmd_x|、|cmd_y| 均低于阈值时禁用 virtual_arm_obs
  bool isVirtualArmObsBlockedByLowCmd() const;
  /// @brief v17 amp_hand：推理 action 后处理（lateral_elbow_fix）
  void applyAmpV17ActionPostProcess(array_t& actions) const;
  /// @brief v17 amp_hand：后退摆臂增强 action 后处理
  void applyBackArmEnhancePostProcess(array_t& actions) const;
  /// @brief v17 amp_hand：起身增强 action 后处理（leg1/leg4 偏置）
  void applyStandUpEnhancePostProcess(array_t& actions) const;
  /// @brief 更新起身上升阶段判据（膝角减小）
  void updateStandUpRisingState();
  /// @brief 微小速度命令截断（env.tiny_cmd_clip）
  double applyTinyCmdxClip(double cmd_x) const;
  double applyTinyCmdYClip(double cmd_y) const;
  void applyTinyCmdClip(array_t& velocity_cmd) const;
  /// @brief 低速起步脉冲（env.low_speed_kick_start）
  void applyLowSpeedKickStart(array_t& velocity_cmd);
  /// @brief 低速原地转向脉冲（env.low_speed_yaw_kick_start）
  void applyLowSpeedYawKickStart(array_t& velocity_cmd);
  /// @brief 与 25 帧前角速度反向时清零，防止反向脉冲波动
  void applyReverseYawKickGuard(array_t& velocity_cmd);
  /// @brief 混合运动速度限制（env.mixedMotionLimits）
  void applyMixedMotionLimits(array_t& velocity_cmd) const;
  /// @brief 根据名称计算观测项
  array_t getObsTerm(const std::string& name) const;
  /// @brief 获取观测项维度
  int getObsTermShape(const std::string& name) const;
  /// @brief 重置观测历史缓冲区
  void resetObsHistory();
  /// @brief 初始化观测历史缓冲区结构
  void initObsHistory();
  /// @brief 初始化世界坐标系 yaw 偏移（首次 update 时调用）
  void initializeDummyWorldYaw();
  /// @brief 获取当前 motion，无则返回 nullptr
  MotionTrajectory* getCurrentMotion() const;

  /// @brief 启动/停止异步推理线程
  void startInferenceThread();
  void stopInferenceThread();

  /// @brief 推理线程主循环
  void inferenceThreadLoop();

  /// @brief 提交一帧观测给推理线程
  void submitObservationForInference(const array_t& observation);

  /// @brief 从缓存中读取当前动作快照
  array_t getActionsSnapshot() const;

  /// @brief 清零推理时间戳，避免切换首帧沿用旧值
  void clearInferenceTimestamps();

  /// @brief 发布实际策略推理完成频率 [Hz]
  void publishInferenceFrequency(double finish_time_sec);

  /// @brief 刷新策略切换参考命令（在手臂/腰部覆盖之后调用）
  void updateBlendReferenceCmd(const RobotCmd& final_cmd);

  // ==================== 机器人配置 ====================
  RobotVersion robot_version_;              ///< 机器人版本
  int motor_count_ = 0;                     ///< SDK 电机总数
  std::vector<std::string> motor_names_;    ///< SDK 电机名称列表
  // 非策略关节（S17 头部等）的固定保持目标。首次收到有效状态时锁存
  // 资产初始角度，之后不再把“当前角度”当作新目标，避免重力导致慢慢下坠。
  std::vector<double> held_non_policy_pos_;
  std::vector<double> head_default_pos_;
  bool held_non_policy_pos_initialized_ = false;
  int diagnostic_cmd_log_count_ = 0;
  std::string config_path_;                 ///< 配置文件路径
  std::string urdf_path_;                   ///< URDF 路径（手臂重力补偿用）

  /// 倒地起身双策略：趴着 / 仰躺（对齐 kuavo FallStandModelType）
  enum class FallStandModelType {
    kProne = 0,
    kSupine = 1,
  };

  // ==================== 控制时序 ====================
  double policy_dt_ = 0.02;                 ///< 策略推理周期 [s]
  std::string policy_path_;                 ///< ONNX 模型路径（单策略模式）
  std::string prone_policy_path_;           ///< 趴着起身策略（双策略模式）
  std::string supine_policy_path_;          ///< 仰躺起身策略（双策略模式）
  std::string inference_engine_ = "openvino"; ///< 推理引擎 (openvino, onnxruntime)
  bool dual_fall_stand_ = false;            ///< 是否启用 prone/supine 双策略
  bool limp_when_idle_ = false;             ///< 倒地起身：未准备/未起播时瘫软零力矩（对齐 kuavo FALL_DOWN 态）
  FallStandModelType fall_stand_model_type_ = FallStandModelType::kSupine;
  std::string fall_stand_motion_base_;      ///< 逻辑 motion 名，如 fall_stand
  std::unique_ptr<InferenceModel> model_prone_;
  std::unique_ptr<InferenceModel> model_supine_;

  // ==================== 关节映射 ====================
  int policy_joint_count_ = 0;              ///< 策略控制的关节数

  // ==================== 观测配置 ====================
  int obs_history_length_ = 1;              ///< 观测历史长度
  StackOrder obs_stack_order_ = StackOrder::kIsaaclab;  ///< 历史堆叠顺序
  std::vector<ObsTermConfig> obs_terms_;    ///< 观测项配置列表

  // 观测历史缓冲区（2D deque，布局取决于 stack_order）
  // kIsaaclab: obs_stacks_[term_index][time_index]
  // kClassic:  obs_stacks_[time_index][term_index]
  std::deque<std::deque<array_t>> obs_stacks_;

  // ==================== Motion 配置 ====================
  std::map<std::string, std::string> motion_paths_;     ///< motion 名称 → 文件路径
  bool motion_residual_action_ = false;     ///< 是否使用残差动作
  std::map<std::string, std::unique_ptr<MotionTrajectory>> motions_;  ///< motion 实例
  std::string current_motion_name_;         ///< 当前选中的 motion
  bool auto_switch_back_enabled_ = false;   ///< 舞蹈播完是否自动切回稳定控制器
  std::string auto_switch_back_target_;     ///< 自动切回的目标控制器名

  /// 两阶段 motion：插值到 CSV 首帧并保持；StandbyHold=START 后锁姿不跑策略
  enum class MotionPrepState {
    kIdle = 0,
    kInterpolating,
    kHolding,
    kStandbyHold,
  };
  MotionPrepState motion_prep_state_ = MotionPrepState::kIdle;
  double motion_prep_elapsed_ = 0.0;
  double motion_prep_duration_ = 0.0;
  double motion_prep_max_joint_velocity_ = 1.5;  ///< 插值最大关节速度 [rad/s]，对齐 kuavo fallStandMaxJointVelocity
  std::vector<double> motion_prep_start_q_;
  std::vector<double> motion_prep_target_q_;

  // 倒地起身 READY → 正常 CSV/策略目标的前段缓动；CSV 与策略仍按原时序推进。
  static constexpr int kFallStandEntryBlendFrames = 500;
  bool fall_stand_entry_blend_active_ = false;
  int fall_stand_entry_blend_frame_ = 0;

  /// 填充保持/插值阶段的 RobotCmd（CSP）
  void fillHoldCmd(RobotCmd& cmd, const std::vector<double>& q_target) const;

  /// 填充瘫软（零力矩、关节自由，对齐 kuavo FALL_DOWN 态）的 RobotCmd（CST 全零）
  void fillLimpCmd(RobotCmd& cmd, const RobotState& state) const;

  /// 按 IMU 机体 x 与重力点积自动选择并切换 prone/supine
  bool autoSelectFallStandModel(const ImuData& imu);
  /// 切换活跃策略网络 + 对应 CSV 轨迹
  bool switchFallStandModel(FallStandModelType model_type);

  // ==================== 速度缩放（行走模式 cmd_stance=0）====================
  double velocity_scale_linear_x_ = 1.0;               ///< 前进速度缩放
  double velocity_scale_linear_x_up_ = 1.0;            ///< 高速档前进上限（gear 模式）
  double velocity_scale_linear_x_negative_ = 1.0;      ///< 后退速度额外缩放
  /// 外部手臂控制时前进 cmd_x 额外加成 [m/s]，上限 linear_x_up + boost
  double external_arm_linear_x_boost_ = 0.0;
  /// 十字键档位模式：teleop 已输出 policy 空间前进速度，不再乘 linear_x
  bool velocity_scale_forward_direct_ = false;
  double velocity_scale_linear_y_ = 1.0;               ///< 侧向速度缩放
  double velocity_scale_angular_z_ = 1.0;              ///< 偏航角速度缩放

  // ==================== 速度缩放（站立模式 cmd_stance=1）====================
  double velocity_scale_angular_z_standing_ = 1.0;         ///< 下蹲高度缩放

  VelocityStopFilter angular_z_stop_filter_;           ///< angular_z 停止减速滤波（仅 standing）
  struct VelocitySmoothingConfig {
    bool cmd_x_smooth_enabled = false;
    double max_velocity_change_cmd_x = -1.0;        ///< 正向 cmd_x 加速单步上限 [m/s]
    double max_velocity_change_decel_cmd_x = -1.0;  ///< 正向 cmd_x 减速单步上限 [m/s]
    double cmd_x_decel_ema_tau_threshold = 0.5;     ///< 前进 EMA 分段阈值 [m/s]
    double cmd_x_decel_ema_tau_high = 0.15;         ///< 高速段 EMA 时间常数 [s]
    double cmd_x_decel_ema_tau_low = 0.35;          ///< 低速段 EMA 时间常数 [s]
    double max_velocity_change_neg_accel_cmd_x = -1.0;  ///< 后退加速单步上限 [m/s]
    double max_velocity_change_neg_cmd_x = -1.0;        ///< 后退减速单步上限 [m/s]
    double cmd_x_decel_hold_speed = -1.0;               ///< 两阶段停车保持速度 [m/s]，<=0 禁用
    double cmd_x_decel_hold_time = 5.0;                 ///< 在保持速度上停留时长 [s]
  };
  VelocitySmoothingConfig velocity_smoothing_;
  double smoothed_raw_cmd_x_ = 0.0;                    ///< 平滑后的缩放前 cmd_x [m/s]
  bool cmd_x_decel_in_hold_ = false;                   ///< 是否处于两阶段停车保持阶段
  double cmd_x_decel_hold_timer_ = 0.0;                ///< 保持阶段累计时长 [s]
  double max_standup_change_ = 0.0;                    ///< 起身单步最大高度命令变化量 [m]
  double max_velocity_change_cmd_y_ = 0.0;             ///< 单步 cmd_y 最大变化量 [m/s]，<=0 禁用
  double max_velocity_change_cmd_angz_ = 0.0;          ///< 单步 cmd_angz 减速最大变化量 [rad/s]，<=0 禁用
  double smoothed_raw_cmd_y_ = 0.0;                    ///< 限速后的 cmd_y [m/s]
  double smoothed_raw_cmd_angz_ = 0.0;                 ///< 限速后的 cmd_angz [rad/s]
  double max_stance_squat_depth_ = 0.18;               ///< 下蹲高度命令下限 [m]（无膝角限制时回退）
  double max_squat_knee_angle_ = 0.0;                  ///< 最大下蹲膝角 [rad]，leg_l4/leg_r4，>0 启用
  double max_squat_leg1_angle_ = 0.0;                  ///< 最大下蹲髋 pitch [rad]，leg_l1:-max/leg_r1:+max，>0 启用
  static constexpr double kSquatLeg1ExternalArmReductionRad_{0.05};  ///< 外部手控时 leg1 上限额外收紧量 [rad]
  double smoothed_stance_height_cmd_ = 0.0;            ///< 平滑后的下蹲高度命令
  array_t filtered_velocity_cmd_ = array_t::Zero(3);   ///< 当前观测周期使用的速度指令

  // v46/v52 观测 cmd_stance：基于 filtered_velocity_cmd_ 的帧防抖（v17 用手动 cmd_stance_mode）
  mutable int stance_debounce_counter_ = 0;

  struct SquatPostureDefenseConfig {
    bool enabled = false;
    double height_threshold = 0.7;  ///< leg_l4/leg_r4 均高于该值视为深蹲 (rad)
  };
  SquatPostureDefenseConfig squat_posture_defense_;
  bool squat_posture_deep_seen_{false};
  int leg_l4_policy_idx_{-1};
  int leg_r4_policy_idx_{-1};
  int leg_l1_policy_idx_{-1};
  int leg_r1_policy_idx_{-1};

  // v17 amp_hand 增强（YAML: enable_amp_arm_enhance_v17）：virtual_arm_obs /
  // roll_compensation_closed_loop / off_cmdy_by_cmdx / off_cmdangz_by_cmdy / lateral_elbow_fix
  bool enable_amp_arm_enhance_ = false;
  // v17 后退增强（YAML: enable_back_enhance_v17）：后退摆臂 action + 重力 pitch 补偿
  bool enable_back_enhance_v17_ = false;
  // v17 起身增强（YAML: enable_standup_enhance_v17）：projected_gravity 后仰偏置 + leg1/leg4 action
  bool enable_standup_enhance_v17_ = false;
  // v17 原地旋转增强（YAML: enable_turn_in_place_enhance_v17）：leg_l1/leg_r1 action 偏置
  bool enable_turn_in_place_enhance_v17_ = false;
  static constexpr double kStandUpGravityPitchBiasDeg_{-1.5};
  static constexpr double kStandUpPitchFullBiasKneeEnd_{1.2};
  static constexpr double kStandUpPitchFadeKneeStart_{0.6};
  static constexpr double kStandUpKneeDecreasingEpsilon_{1e-3};
  static constexpr double kStandUpKneeFullEnhanceRad_{1.2};
  static constexpr double kStandUpKneeNoEnhanceRad_{0.55};
  static constexpr double kStandUpLeg1ActionBias_{0.025};
  static constexpr double kStandUpKneeActionBias_{-0.2};
  bool stand_up_rising_active_{false};
  double prev_stand_up_knee_rad_{0.0};
  bool stand_up_knee_prev_initialized_{false};
  static constexpr double kBackArmEnhanceScale_{0.17};
  static constexpr double kBackArmEnhanceCmdXThreshold_{-0.2};
  static constexpr double kBackArmEnhanceGravityCmdXMin_{-0.3};
  static constexpr double kBackArmEnhanceGravityCmdXMax_{-0.02};
  static constexpr double kBackArmEnhanceGravityPitchDeg_{2.7};
  static constexpr double kVirtualArmObsPitchBaseDeg_{0.2};
  static constexpr double kVirtualArmObsPitchCompensationDeg_{1.0};
  static constexpr double kVirtualArmObsPitchArmForwardScale_{0.6};  
  static constexpr double kVirtualArmObsBlockedLowCmdThreshold_{0.2};
  static constexpr double kVirtualArmObsPitchDecelReductionDeg_{0.3};
  static constexpr double kVirtualArmObsPitchBaseDegNeg_{0.5};
  static constexpr double kVirtualArmObsPitchCompensationDegNeg_{0.0};
  static constexpr double kVirtualArmObsArm1BackSumMaxRad_{4.0};
  static constexpr double kVirtualArmObsArm1BackSumPitchReductionFullRad_{0.8};
  static constexpr double kVirtualArmObsArm1BackPitchReductionMaxDeg_{1.2};
  static constexpr double kVirtualArmObsArm1BackVelSumPitchReductionFullRadPerSec_{1.0};
  static constexpr double kVirtualArmObsArm1BackVelPitchReductionMaxDeg_{0.3};
  static constexpr double kRollCompensationCmdXThreshold_{0.3};
  static constexpr double kTurnRollCompensationDeg_{-1.0};        ///< 走弧线开环 roll 补偿系数 (deg/(rad/s))
  static constexpr double kTurnRollCompensationCmdXMin_{0.2};     ///< 开环补偿生效 cmd_x 下限 (m/s)
  static constexpr double kTurnRollCompensationAbsAngZMin_{0.35};  ///< 开环补偿生效 |cmd_angz| 下限 (rad/s)
  static constexpr double kTurnRollCompensationExternalArmScale_{0.5};  ///< 外部手控时开环 roll 补偿缩放
  static constexpr double kTurnPitchCompensationDeg_{-0.25};         ///< 走弧线开环 pitch 补偿系数 (deg/(rad/s))
  static constexpr double kTurnPitchCompensationCmdXMin_{0.3};   ///< 开环 pitch 补偿生效 cmd_x 下限 (m/s)
  static constexpr double kTurnPitchCompensationAbsAngZMin_{0.3};  ///< 开环 pitch 补偿生效 |cmd_angz| 下限 (rad/s)
  static constexpr double kTurnPitchCompensationExternalArmScale_{2.0};  ///< 外部手控时开环 pitch 补偿缩放
  /// 原地旋转 leg_l1/leg_r1 action 偏置（policy action 空间）
  static constexpr double kInPlaceYawAbsCmdXMax_{0.15};
  static constexpr double kInPlaceYawAbsCmdYMax_{0.20};
  static constexpr double kInPlaceYawAbsAngZMin_{0.15};
  static constexpr double kInPlaceYawLegL1ActionBias_{-0.05};
  static constexpr double kInPlaceYawLegR1ActionBias_{0.05};
  static constexpr double kOffCmdxByCmdYThreshold_{0.3};
  static constexpr double kOffCmdAngzByCmdYThreshold_{0.3};
  struct OffCmdxAfterLateralConfig {
    bool enabled = false;
    double cmd_y_threshold = 0.1;
    int history_frames = 20;
  };
  OffCmdxAfterLateralConfig off_cmdx_after_lateral_;
  std::deque<bool> lateral_cmd_y_active_history_;
  static constexpr double kElbowTargetUpperBound_{-0.2};
  static constexpr int kElbowActionIndices_[2] = {16, 20};

  struct RollCompensationClosedLoopConfig {
    bool enabled = false;
    double cmd_x_min = 0.30;
    double cmd_x_max = 1.0;
    double abs_cmd_y_max = 0.10;
    double abs_cmd_ang_z_max = 0.30;
    double filter_time_constant_sec = 1.2;
    /// 第一帧零点 |roll| 上限 (deg)，0=不限制；超上限 target_roll=0，超一半上限则 target_roll 减半
    double first_frame_max_abs_roll_deg = 0.0;
    double kp = 1.2;
    double ki = 0.06;
    double max_compensation_deg = 0.70;
    /// 外部手臂控制时禁用直行 roll 闭环；积分清零、filter 仍跟踪，target 不变
    bool unable_compensation_arm_controller = false;
  };
  RollCompensationClosedLoopConfig roll_compensation_closed_loop_;
  mutable bool roll_compensation_closed_loop_initialized_{false};
  mutable double roll_compensation_filtered_roll_rad_{0.0};
  mutable double roll_compensation_target_roll_rad_{0.0};
  mutable double roll_compensation_integral_rad_sec_{0.0};
  struct LateralYawCompensationClosedLoopConfig {
    bool enabled = false;
    double kp = 0.35;
    double kd = 0.04;
    double max_action = 0.03;  ///< 单步腰部 action 补偿限幅（policy action 空间）
  };
  LateralYawCompensationClosedLoopConfig lateral_yaw_compensation_closed_loop_;
  mutable bool lateral_yaw_compensation_initialized_{false};
  mutable double lateral_yaw_target_rad_{0.0};
  static constexpr double kLateralElbowFixScale_{0.25};
  static constexpr double kLateralLegActionScaleLow_{0.8};
  static constexpr double kLateralLegActionScaleHigh_{1.2};

  struct TinyCmdClipConfig {
    bool enabled = false;
    bool cmd_x_enabled = false;
    bool cmd_y_enabled = false;
    double cmd_x_pos_min = 0.0;
    double cmd_x_pos_max = 0.0;
    double cmd_abs_y_min = 0.0;
    double cmd_abs_y_max = 0.0;
  };
  TinyCmdClipConfig tiny_cmd_clip_;

  struct LowSpeedKickStartConfig {
    bool enabled = false;
    double kick_velocity = 0.6;
    int duration_steps = 25;
    double rest_cmd_threshold = 0.2;
    double trigger_velocity = 0.3;
    double trigger_tolerance = 0.15;
    double lateral_threshold = 0.2;
    double yaw_threshold = 0.2;
  };
  struct LowSpeedYawKickStartConfig {
    bool enabled = false;
    double kick_angular_velocity = 1.0;
    int duration_steps = 10;
    double rest_cmd_threshold = 0.2;
    double trigger_angular_velocity = 0.5;
    double trigger_tolerance = 0.25;
    double forward_threshold = 0.2;
    double lateral_threshold = 0.2;
    bool enable_reverse_kick_guard = false;
  };
  static constexpr int kReverseYawKickGuardHistorySteps_ = 25;  
  LowSpeedKickStartConfig low_speed_kick_start_;
  LowSpeedYawKickStartConfig low_speed_yaw_kick_start_;
  int low_speed_kick_remaining_steps_{0};
  int low_speed_yaw_kick_remaining_steps_{0};
  double low_speed_yaw_kick_sign_{1.0};
  std::deque<double> yaw_cmd_angz_history_;
  double prev_raw_cmd_vel_line_x_{0.0};
  double prev_raw_cmd_vel_angular_z_{0.0};
  mutable double prev_virtual_arm_obs_filtered_cmd_x_{0.0};  ///< 上一帧 filtered cmd_x，用于减速补偿削弱判据

  struct MixedMotionLimitsConfig {
    bool enable_mixed_mode = false;
    double angular_vel_threshold = 0.05;
    double max_linear_vel_with_angular = 0.85;
    double linear_vel_threshold = 0.05;
    double max_angular_vel_with_linear = 0.7;
  };
  MixedMotionLimitsConfig mixed_motion_limits_;

  // ==================== 运行时状态 ====================
  double dummy_world_yaw_ = 0.0;            ///< 世界坐标系初始 yaw
  bool motion_playing_ = false;             ///< motion 是否正在播放
  bool motion_just_finished_ = false;       ///< motion 刚播完的边沿标志（本控制周期有效）
  bool block_velocity_in_motion_ = false;   ///< M1/M2 motion 播放期间屏蔽摇杆行走

  // ==================== 异步推理线程 ====================
  mutable std::mutex action_mutex_;         ///< 保护 actions_/last_actions_
  std::mutex inference_mutex_;              ///< 保护待推理观测
  std::condition_variable inference_cv_;    ///< 推理线程唤醒条件变量
  std::thread inference_thread_;            ///< 独立推理线程
  bool inference_thread_running_ = false;   ///< 推理线程是否运行
  bool inference_stop_requested_ = false;   ///< 请求停止推理线程
  bool has_pending_observation_ = false;    ///< 是否有待推理观测
  array_t pending_observation_;             ///< 待推理观测缓存
  std::atomic<double> last_observation_submit_time_sec_{0.0};  ///< 最近一次提交观测时间
  std::atomic<double> last_inference_finish_time_sec_{0.0};    ///< 最近一次推理完成时间
  std::atomic<double> last_inference_frequency_sample_time_sec_{0.0};  ///< 上次频率采样时间

  // ==================== 策略切换参考 ====================
  array_t last_policy_q_target_;  ///< 最近一次策略关节目标位置（策略空间→电机空间）
  RobotCmd blend_reference_cmd_;  ///< 策略切换用参考命令（q 中保存 q_target）
  array_t blend_torque_limits_;   ///< 策略切换时的策略关节力矩限幅（电机空间）
  array_i blend_recompute_mask_;  ///< 1=按 q_target 重算 tau；0=沿用旧整包混合

  // ==================== 日志 ====================
  std::unique_ptr<TopicLogger> logger_;     ///< DDS 调试数据发布器
};

}  // namespace leju
