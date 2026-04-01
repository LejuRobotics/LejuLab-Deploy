#pragma once

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
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
  /// @brief 执行一次控制更新
  bool update(double time, const RobotState& state, const ImuData& imu,
              RobotCmd& cmd) override;
  /// @brief 设置速度指令
  void setVelocityCommand(const VelocityCommand& cmd) override { velocity_cmd_ = cmd; }
  /// @brief 接收手柄输入
  void onJoyInput(const JoyData& joy, const JoyData::Buttons& prev) override;
  /// @brief 移动到默认关节位置
  void moveToDefaultPos(const RobotState& current_state, double elapse) override;

  // ==================== 配置 ====================

  /// @brief 设置配置文件路径，必须在 initialize() 前调用
  void setConfigPath(const std::string& config_path) { config_path_ = config_path; }

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

 protected:
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
  /// @brief 计算限幅后的速度指令
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

  // ==================== 机器人配置 ====================
  RobotVersion robot_version_;              ///< 机器人版本
  int motor_count_ = 0;                     ///< SDK 电机总数
  std::vector<std::string> motor_names_;    ///< SDK 电机名称列表
  std::string config_path_;                 ///< 配置文件路径

  // ==================== 控制时序 ====================
  double loop_dt_ = 0.001;                  ///< 控制周期 [s]
  double policy_dt_ = 0.02;                 ///< 策略推理周期 [s]
  std::string policy_path_;                 ///< ONNX 模型路径

  // ==================== 关节配置 ====================
  // 注：joint_kp_、joint_kd_、joint_action_scale_、joint_torque_limit_、
  //     joint_control_mode_、default_joint_pos_ 继承自 ControllerBase
  std::vector<std::string> joint_names_;    ///< 策略关节名称（按策略顺序）
  array_t joint_direction_;                 ///< 关节方向系数

  // ==================== 关节映射 ====================
  std::vector<int> policy_joint_ids_;       ///< 策略关节对应的 SDK 电机索引
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

  // ==================== 指令限幅 ====================
  Bounds cmd_lin_vel_x_ = {-1.0, 1.0};     ///< 前进速度范围 [m/s]
  Bounds cmd_lin_vel_y_ = {-0.6, 0.6};     ///< 侧向速度范围 [m/s]
  Bounds cmd_ang_vel_z_ = {-0.3, 0.3};     ///< 偏航角速度范围 [rad/s]

  // ==================== 运行时状态 ====================
  RobotState current_state_;                ///< 缓存的关节状态
  ImuData current_imu_;                     ///< 缓存的 IMU 数据
  VelocityCommand velocity_cmd_;            ///< 当前速度指令
  uint64_t step_count_ = 0;                 ///< 策略步数计数
  double dummy_world_yaw_ = 0.0;            ///< 世界坐标系初始 yaw
  bool motion_playing_ = false;             ///< motion 是否正在播放

  // ==================== 日志 ====================
  std::unique_ptr<TopicLogger> logger_;     ///< DDS 调试数据发布器
};

}  // namespace leju
