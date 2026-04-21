#pragma once

#include <functional>
#include <memory>
#include <string>

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
   * @brief 从 teleop 配置加载 External 速度映射上限
   * @param config_path teleop_bindings.yaml 路径
   * @return 是否加载成功
   */
  bool loadVelocityLimitsFromTeleopConfig(const std::string& config_path);

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

private:
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

  // External VelocityCmd 归一化输入映射配置
  TeleopConfig velocity_limits_;
};

} // namespace runtime
} // namespace leju
