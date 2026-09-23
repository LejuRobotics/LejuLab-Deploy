#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <dds/dds.hpp>
#include "lejusdk-dds-idl/TactState.hpp"
#include <lejusdk-dds-idl/Float64Types.hpp>
#include <lejusdk-topic-pubsub/topic_names.h>
#include "lejusdk-topic-pubsub/topic_publisher.hpp"
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>
#include "lejusdk-utils/loop.hpp"
#include "lejusdk-utils/robot_version.hpp"
#include "lejusdk-vr/data_types.h"
#include "lejusdk-vr/vr_api/vr_base.h"

#include "leju-rl-controller/runtime/input/command_buffer.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/input/input_source.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding_config.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"

namespace leju {

// 前向声明
class ControllerManager;
class RobotBaseAPI;

namespace runtime {

// 前向声明
class Lifecycle;
class TactPlayer;

// 类型别名（与 VR SDK 保持一致）
using QuestJoystickData = vr::QuestJoystickData;
using VrVelocityCmd = vr::VrVelocityCmd;
using JointTrajectoryPoint = vr::JointTrajectoryPoint;
using ControlMode = vr::ControlMode;

/**
 * @brief 外部统一接口
 *
 * 接收外部控制请求并转换为内部语义：
 * - 控制请求 -> TriggerBuffer/CommandBuffer -> ControlLoop
 * - 查询请求 -> ControllerManager (只读)
 *
 */
class ExternalInterface : public InputSource {
 public:
  ExternalInterface();
  ~ExternalInterface() override;

  // 禁止拷贝
  ExternalInterface(const ExternalInterface&) = delete;
  ExternalInterface& operator=(const ExternalInterface&) = delete;

  /**
   * @brief 初始化
   * @param version 机器人版本
   * @param trigger_buffer 触发器缓冲区（用于输出离散事件）
   * @param controller_manager 控制器管理器（用于查询接口）
   * @param lifecycle 生命周期管理器（用于检查系统运行状态）
   * @return 是否成功
   */
  bool initialize(const RobotVersion& version,
                  TriggerBuffer& trigger_buffer,
                  ControllerManager& controller_manager,
                  Lifecycle& lifecycle);

  /**
   * @brief 关闭
   */
  void shutdown();

  /**
   * @brief 检查是否已初始化
   * @return 是否已初始化
   */
  bool isInitialized() const;

  /**
   * @brief 从 teleop 配置加载 External 归一化速度映射上限
   * @param config_path teleop_bindings.yaml 路径
   * @return 是否加载成功
   */
  bool loadVelocityLimitsFromTeleopConfig(const std::string& config_path);

  /**
   * @brief 从 AMP 控制器配置加载 posture 高度映射参数
   * @param config_path config_amp.yaml 路径
   * @param standing_height 站立参考高度 [m]
   * @param timeout_sec 流式高度命令有效期 [s]
   */
  bool loadPostureHeightConfig(const std::string& config_path,
                               double standing_height = 0.77,
                               double timeout_sec = 0.5);

  /**
   * @brief 获取当前命令快照（线程安全）
   * @return 当前命令快照
   */
  CommandBuffer::Snapshot getSnapshot() const override;

  /**
   * @brief 获取输入源优先级
   * @return 输入源优先级
   */
  InputPriority getPriority() const override { return InputPriority::kExternal; }

  /**
   * @brief 获取输入源名称
   * @return 输入源名称
   */
  const char* getName() const override { return "External"; }

  /**
   * @brief 清零外部输入源缓存的速度指令
   */
  void clearCmdVel() override;

private:
  friend class ExternalInterfaceTestPeer;

  /**
   * @brief 处理手臂关节指令
   * @param cmd 关节轨迹点
   */
  void onArmJointCmd(const JointTrajectoryPoint& cmd);

  /**
   * @brief 处理头部关节指令
   * @param cmd 关节轨迹点
   */
  void onHeadJointCmd(const JointTrajectoryPoint& cmd);

  /**
   * @brief 处理腰部关节指令
   * @param cmd 关节轨迹点
   */
  void onWaistJointCmd(const JointTrajectoryPoint& cmd);

  /**
   * @brief 处理速度指令（控制器输入速度）
   * @param cmd 速度指令
   */
  void onVelocityCmd(const vr::VelocityCmd& cmd);

  /** @brief DDS 流式 posture 高度（仅刷新时间戳，不写长期 buffer） */
  void onPostureHeightCmd(double height_m);

  /**
   * @brief 构建 posture 高度指令（新鲜窗口内返回下蹲接管指令）
   * @param now_sec 当前单调时间 [s]
   * @param base_vel 超时退出指令的速度透传基准（行走速度不被打断）
   */
  std::optional<MotionCommand> buildFreshPostureCommand(
      double now_sec, const MotionCommand& base_vel) const;

  // ========================================================================
  // RPC 处理器（同步返回）
  // ========================================================================

  /**
   * @brief 处理切换控制器请求
   * @param name 目标控制器名称
   * @param[out] message 返回消息
   * @return 是否成功
   */
  bool onSwitchControllerRequest(const std::string& name, std::string& message);

  /**
   * @brief 处理设置手臂模式请求
   * @param mode VR 控制模式
   * @param[out] message 返回消息
   * @return 是否成功
   */
  bool onSetArmModeRequest(ControlMode mode, std::string& message);

  /**
   * @brief 处理设置腰部模式请求
   * @param mode VR 控制模式
   * @param[out] message 返回消息
   * @return 是否成功
   */
  bool onSetWaistModeRequest(ControlMode mode, std::string& message);

  /**
   * @brief 处理 runtime start 请求
   * @param[out] message 返回消息
   * @return 是否成功
   */
  bool onStartRuntimeRequest(std::string& message);

  /**
   * @brief 处理 runtime stop 请求
   * @param[out] message 返回消息
   * @return 是否成功
   */
  bool onStopRuntimeRequest(std::string& message);

  /**
   * @brief 处理 motion start 请求
   * @param name motion 名称，可为空
   * @param[out] message 返回消息
   * @return 是否成功
   */
  bool onStartMotionRequest(const std::string& name, std::string& message);

  /**
   * @brief 处理 play tact 请求（独立于 CSV motion）
   *
   * tact 播放走 External arm mode 直推 cmd_buffer，与 onStartMotionRequest
   * （CSV motion，走 RL 推理观测）完全解耦，互不干扰。
   *
   * @param name tact 动作名（不含 .tact 后缀）
   * @param[out] message 返回消息
   * @return 是否已开始播放
   */
  bool onPlayTactRequest(const std::string& name, std::string& message);

  /**
   * @brief tact fallback：CSV motion 未找到时降级到 .tact 播放
   *
   * 检查 .tact 文件是否存在，存在则通过 TactPlayer 异步播放。
   * 供 ControllerManager 注册为 TactFallback 回调使用。
   *
   * @param name 动作名（不含 .tact 后缀）
   * @return 是否已开始播放
   */
  bool playTactAsMotionFallback(const std::string& name);

  /**
   * @brief 查询 runtime 原始状态
   * @return runtime 原始状态快照
   */
  vr::RuntimeState onGetRuntimeStateRequest();

  /**
   * @brief 查询 controller 原始状态
   * @return controller 原始状态快照
   */
  vr::ControllerState onGetControllerStateRequest();

  /**
   * @brief 查询 motion 原始状态
   * @return motion 原始状态快照
   */
  vr::MotionState onGetMotionStateRequest();

private:
  std::unique_ptr<vr::VRBaseAPI> vr_api_;
  bool initialized_ = false;

  /**
   * @brief 获取当前手臂/腰部关节位置（度），用于 tact 播放 t=0 平滑过渡
   */
  void getCurrentArmWaistPosDeg(std::vector<double>& arm_pos,
                                std::vector<double>& waist_pos) const;

  // 机器人 API（由调用者提供，非拥有，用于查询关节信息）
  RobotBaseAPI* robot_api_ = nullptr;

  // 缓存的关节数量（初始化时获取）
  size_t arm_joint_count_ = 0;
  size_t head_joint_count_ = 0;
  size_t waist_joint_count_ = 0;

  // 内部命令缓冲区（线程安全）
  CommandBuffer cmd_buffer_;

  // 输出目标（由调用者提供，非拥有）
  TriggerBuffer* trigger_buffer_ = nullptr;

  // 查询目标（由调用者提供，非拥有，用于只读查询接口）
  ControllerManager* controller_manager_ = nullptr;

  // 生命周期管理器（由调用者提供，非拥有，用于检查系统运行状态）
  Lifecycle* lifecycle_ = nullptr;

  // tact 播放器（独立于 CSV motion，External arm mode 直推 cmd_buffer_）
  std::unique_ptr<TactPlayer> tact_player_;

  // tact 状态 1Hz 发布（路径 A：自建 participant + TopicPublisher + Loop）。
  // 状态来源（tact_player_）与发布同处一类，Loop 每秒读取快照组装发布。
  // dds::domain::DomainParticipant 继承 dds::core::Reference，operator new 为 private，
  // 不能 make_unique/new，用 optional 延迟构造（值语义，共享 delegate）。
  std::optional<dds::domain::DomainParticipant> tact_state_participant_;
  std::unique_ptr<leju::dds_common::TopicPublisher<leju::msgs::TactState>> tact_state_pub_;
  std::unique_ptr<leju::Loop> tact_state_loop_;

  /**
   * @brief 1Hz 发布 tact 播放状态到 DDS /rt/tact_state
   *
   * 读取 tact_player_ 的状态快照（state/error_code/name/error_message），
   * 组装 leju::msgs::TactState 并发布。BestEffort QoS，丢一帧无所谓。
   */
  void publishTactState();

  // External VelocityCmd 归一化输入映射配置
  TeleopConfig velocity_limits_;

  // External 速度只在最近一次 DDS 接收后的短窗口内有效。使用本机单调时钟，
  // 避免跨设备系统时间不同步导致误判。
  static constexpr double kVelocityCmdTimeoutSec = 0.5;
  std::atomic<double> velocity_last_rx_time_sec_{0.0};

  // posture 流式高度：latest + 时间戳，超时后不再注入 merge
  double posture_standing_height_ = 0.77;
  double posture_squat_height_min_ = -0.22;
  double posture_squat_height_max_ = 0.01;
  double posture_timeout_sec_ = 0.5;
  std::atomic<double> posture_latest_height_m_{0.0};
  std::atomic<double> posture_last_rx_time_sec_{0.0};

  // 本通道是否曾进入下蹲姿态（新鲜指令到来时置位）。超时后据此补发
  // cmd_stance=0 退出指令；控制器确认已退出（getCurrentCmdStanceMode()==0）
  // 后复位并休眠，避免长期占用输入源优先级。
  mutable bool posture_was_used_ = false;

  dds::domain::DomainParticipant dds_participant_{0};
  std::unique_ptr<leju::dds_common::TopicSubscriber<leju::msgs::Float64>>
      posture_height_sub_;
};

} // namespace runtime
} // namespace leju
