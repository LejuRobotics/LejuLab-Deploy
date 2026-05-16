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
  /// @brief 切换到指定 motion 并开始播放
  bool startMotion(const std::string& name);
  /// @brief 停止 motion 播放
  bool stopMotion();
  /// @brief 获取全部可用 motion 名称
  std::vector<std::string> getMotionNames() const;
  /// @brief 获取当前 motion 名称
  std::string getCurrentMotionName() const;
  /// @brief 检查是否正在播放 motion
  bool isMotionPlaying() const { return motion_playing_; }

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

  /// @brief 获取默认手臂姿态（有 motion 时返回第一帧位置）
  Eigen::VectorXd getDefaultArmPos() const override;

  /// @brief 获取默认腰部姿态（有 motion 时返回第一帧位置）
  Eigen::VectorXd getDefaultWaistPos() const override;

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
  /// @brief 计算缩放后的速度指令
  array_t getVelocityCommands() const;
  /// @brief 计算 motion 目标姿态的相对旋转矩阵
  array_t getMotionAnchorOriB() const;
  /// @brief 计算基座角速度观测项
  array_t getBaseAngVel() const;
  /// @brief 计算投影重力观测项
  array_t getProjectedGravity() const;
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

  /// @brief 线程安全地更新动作缓存
  void updateActionCache(const array_t& new_actions);

  /// @brief 用指定观测执行一次推理
  bool inferActions(const array_t& observation, array_t& inferred_actions);

  /// @brief 清零推理时间戳，避免切换首帧沿用旧值
  void clearInferenceTimestamps();

  /// @brief 刷新策略切换参考命令（在手臂/腰部覆盖之后调用）
  void updateBlendReferenceCmd(const RobotCmd& final_cmd);

  // ==================== 机器人配置 ====================
  RobotVersion robot_version_;              ///< 机器人版本
  int motor_count_ = 0;                     ///< SDK 电机总数
  std::vector<std::string> motor_names_;    ///< SDK 电机名称列表
  std::string config_path_;                 ///< 配置文件路径
  std::string urdf_path_;                   ///< URDF 路径（手臂重力补偿用）

  // ==================== 控制时序 ====================
  double policy_dt_ = 0.02;                 ///< 策略推理周期 [s]
  std::string policy_path_;                 ///< ONNX 模型路径
  std::string inference_engine_ = "openvino"; ///< 推理引擎 (openvino, onnxruntime)

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

  // ==================== 速度缩放 ====================
  double velocity_scale_linear_x_ = 1.0;               ///< 前进速度缩放
  double velocity_scale_linear_x_negative_ = 1.0;      ///< 后退速度额外缩放
  double velocity_scale_linear_y_ = 1.0;               ///< 侧向速度缩放
  double velocity_scale_angular_z_ = 1.0;              ///< 偏航角速度缩放

  // ==================== 运行时状态 ====================
  double dummy_world_yaw_ = 0.0;            ///< 世界坐标系初始 yaw
  bool motion_playing_ = false;             ///< motion 是否正在播放

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

  // ==================== 策略切换参考 ====================
  array_t last_policy_q_target_;  ///< 最近一次策略关节目标位置（策略空间→电机空间）
  RobotCmd blend_reference_cmd_;  ///< 策略切换用参考命令（q 中保存 q_target）
  array_t blend_torque_limits_;   ///< 策略切换时的策略关节力矩限幅（电机空间）
  array_i blend_recompute_mask_;  ///< 1=按 q_target 重算 tau；0=沿用旧整包混合

  // ==================== 日志 ====================
  std::unique_ptr<TopicLogger> logger_;     ///< DDS 调试数据发布器
};

}  // namespace leju
