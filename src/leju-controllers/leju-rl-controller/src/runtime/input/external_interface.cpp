/**
 * @file external_interface.cpp
 * @brief 外部系统统一接口实现
 *
 * 输入适配器实现：
 * - 接收外部命令并转换为统一语义（ActionTrigger 和 ContinuousCommand）
 * - 写入 TriggerBuffer 与 CommandBuffer
 * - 查询请求 runtime / controller / motion 原始状态
 */

#include "leju-rl-controller/runtime/input/external_interface.h"

#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>

#include <magic_enum/magic_enum.hpp>

#include "leju-rl-controller/controllers/controller_manager.h"
#include "leju-rl-controller/runtime/lifecycle.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/input/action_trigger.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/rl/multi_mode_arm_controller.h"
#include "leju-rl-controller/rl/waist_controller.h"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-vr/vr_api/vr_base.h"

namespace leju {
namespace runtime {

// 静态辅助函数：内部枚举转换为 VR ControlMode（仅本文件使用）
static vr::ControlMode ConvertArmModeToVRControlMode(ArmControlMode mode) {
  switch (mode) {
    case ArmControlMode::kKeepPose:
      return vr::ControlMode::kKeepPose;
    case ArmControlMode::kAuto:
      return vr::ControlMode::kAuto;
    case ArmControlMode::kExternal:
      return vr::ControlMode::kExternal;
  }
  return vr::ControlMode::kAuto;  // 默认
}

static vr::ControlMode ConvertWaistModeToVRControlMode(WaistControlMode mode) {
  switch (mode) {
    case WaistControlMode::kAuto:
      return vr::ControlMode::kAuto;
    case WaistControlMode::kExternal:
      return vr::ControlMode::kExternal;
  }
  return vr::ControlMode::kAuto;  // 默认
}

static std::optional<ArmControlMode> ConvertVRControlModeToArmMode(vr::ControlMode mode) {
  switch (mode) {
    case vr::ControlMode::kKeepPose:
      return ArmControlMode::kKeepPose;
    case vr::ControlMode::kAuto:
      return ArmControlMode::kAuto;
    case vr::ControlMode::kExternal:
      return ArmControlMode::kExternal;
    default:
      return std::nullopt;
  }
}

static std::optional<WaistControlMode> ConvertVRControlModeToWaistMode(vr::ControlMode mode) {
  switch (mode) {
    case vr::ControlMode::kAuto:
      return WaistControlMode::kAuto;
    case vr::ControlMode::kExternal:
      return WaistControlMode::kExternal;
    default:
      return std::nullopt;
  }
}

static std::string ArmControlModeToString(ArmControlMode mode) {
  return std::string(magic_enum::enum_name(mode));
}

static std::string WaistControlModeToString(WaistControlMode mode) {
  return std::string(magic_enum::enum_name(mode));
}

static vr::HardwareState ConvertHardwareStateToVr(leju::HardwareState state) {
  switch (state) {
    case leju::HardwareState::UNKNOWN:
      return vr::HardwareState::kUnknown;
    case leju::HardwareState::STOPPED:
      return vr::HardwareState::kStopped;
    case leju::HardwareState::ERROR:
      return vr::HardwareState::kError;
    case leju::HardwareState::INITIALIZING:
      return vr::HardwareState::kInitializing;
    case leju::HardwareState::READY_OK:
      return vr::HardwareState::kReadyOk;
  }
  return vr::HardwareState::kUnknown;
}

static vr::LifecycleState ConvertLifecycleStateToVr(runtime::LifecycleState state) {
  switch (state) {
    case runtime::LifecycleState::kWaitingForReady:
      return vr::LifecycleState::kWaitingForReady;
    case runtime::LifecycleState::kWaitingForStart:
      return vr::LifecycleState::kWaitingForStart;
    case runtime::LifecycleState::kRunning:
      return vr::LifecycleState::kRunning;
    case runtime::LifecycleState::kExiting:
      return vr::LifecycleState::kExiting;
  }
  return vr::LifecycleState::kWaitingForReady;
}

ExternalInterface::ExternalInterface() = default;

ExternalInterface::~ExternalInterface() {
  shutdown();
}

bool ExternalInterface::initialize(const RobotVersion& version,
                                   TriggerBuffer& trigger_buffer,
                                   ControllerManager& controller_manager,
                                   Lifecycle& lifecycle) {
  if (initialized_) {
    return true;
  }

  trigger_buffer_ = &trigger_buffer;
  controller_manager_ = &controller_manager;
  lifecycle_ = &lifecycle;

  // 依赖外部初始化
  if (!GlobalRobot::is_initialized()) {
    throw std::runtime_error("ExternalInterface: GlobalRobot not initialized");
  }
  robot_api_ = &GlobalRobot::getInstance();

  // 缓存关节数量（避免每次回调都调用）
  arm_joint_count_ = robot_api_->getArmJointNames().size();
  head_joint_count_ = robot_api_->getHeadJointNames().size();
  waist_joint_count_ = robot_api_->getWaistJointNames().size();

  vr_api_ = std::make_unique<vr::VRBaseAPI>(version);
  if (!vr_api_->initialize()) {
    RL_LOGE("ExternalInterface: Failed to initialize VR API.");
    vr_api_.reset();
    trigger_buffer_ = nullptr;
    controller_manager_ = nullptr;
    lifecycle_ = nullptr;
    return false;
  }

  ///////////////////////////////////////////////////////////////////////
  // For Lejusdk Interface
  ///////////////////////////////////////////////////////////////////////
  vr_api_->subscribeArmJointCmd(
      [this](const vr::JointTrajectoryPoint& cmd) { onArmJointCmd(cmd); });

  vr_api_->subscribeHeadJointCmd(
      [this](const vr::JointTrajectoryPoint& cmd) { onHeadJointCmd(cmd); });

  vr_api_->subscribeWaistJointCmd(
      [this](const vr::JointTrajectoryPoint& cmd) { onWaistJointCmd(cmd); });

  vr_api_->subscribeVelocityCmd(
      [this](const vr::VelocityCmd& cmd) { onVelocityCmd(cmd); });

  // 注册 RPC 处理器
  vr_api_->registerSwitchControllerHandler(
      [this](const std::string& name, std::string& message) -> bool {
        return onSwitchControllerRequest(name, message);
      });
  vr_api_->registerSetArmModeHandler(
      [this](vr::ControlMode mode, std::string& message) -> bool {
        return onSetArmModeRequest(mode, message);
      });
  vr_api_->registerSetWaistModeHandler(
      [this](vr::ControlMode mode, std::string& message) -> bool {
        return onSetWaistModeRequest(mode, message);
      });
  vr_api_->registerGetRuntimeStateHandler(
      [this]() -> vr::RuntimeState {
        return onGetRuntimeStateRequest();
      });
  vr_api_->registerGetControllerStateHandler(
      [this]() -> vr::ControllerState {
        return onGetControllerStateRequest();
      });
  vr_api_->registerGetMotionStateHandler(
      [this]() -> vr::MotionState {
        return onGetMotionStateRequest();
      });
  vr_api_->registerStartRuntimeHandler(
      [this](std::string& message) -> bool {
        return onStartRuntimeRequest(message);
      });
  vr_api_->registerStopRuntimeHandler(
      [this](std::string& message) -> bool {
        return onStopRuntimeRequest(message);
      });
  vr_api_->registerStartMotionHandler(
      [this](const std::string& name, std::string& message) -> bool {
        return onStartMotionRequest(name, message);
      });

  ///////////////////////////////////////////////////////////////////////
  // Quest 手柄数据处理：由 QuestTeleopAdapter 独立订阅处理
  // ExternalInterface 不再订阅 QuestJoystickData，避免重复处理
  ///////////////////////////////////////////////////////////////////////

  initialized_ = true;
  RL_LOGI("ExternalInterface: Initialized successfully for robot version %s.",
          version.to_string().c_str());
  return true;
}

void ExternalInterface::shutdown() {
  if (vr_api_) {
    vr_api_->shutdown();
    vr_api_.reset();
  }
  trigger_buffer_ = nullptr;
  controller_manager_ = nullptr;
  lifecycle_ = nullptr;
  initialized_ = false;
}

bool ExternalInterface::isInitialized() const {
  return initialized_ && vr_api_ && vr_api_->isInitialized();
}

// ============================================================================
// 私有方法
// ============================================================================

void ExternalInterface::onArmJointCmd(const JointTrajectoryPoint& cmd) {
  if (cmd.q.empty()) {
    RL_LOGW("ExternalInterface: Arm joint cmd is empty");
    return;
  }

  // 验证关节维度（使用缓存的数量）
  if (cmd.q.size() != arm_joint_count_) {
    RL_LOGW("ExternalInterface: Arm joint count mismatch, expected %zu, got %zu",
            arm_joint_count_, cmd.q.size());
    return;
  }

  cmd_buffer_.writeArmTarget(cmd);
  // RL_LOGD("ExternalInterface: Arm target written to cmd_buffer");
}

void ExternalInterface::onHeadJointCmd(const JointTrajectoryPoint& cmd) {
  if (cmd.q.empty()) {
    return;
  }

  // 验证关节维度（使用缓存的数量）
  if (cmd.q.size() != head_joint_count_) {
    RL_LOGW("ExternalInterface: Head joint count mismatch, expected %zu, got %zu",
            head_joint_count_, cmd.q.size());
    return;
  }

  cmd_buffer_.writeHeadTarget(cmd);
}

void ExternalInterface::onWaistJointCmd(const JointTrajectoryPoint& cmd) {
  if (cmd.q.empty()) {
    return;
  }

  // 验证关节维度（使用缓存的数量）
  if (cmd.q.size() != waist_joint_count_) {
    RL_LOGW("ExternalInterface: Waist joint count mismatch, expected %zu, got %zu",
            waist_joint_count_, cmd.q.size());
    return;
  }

  cmd_buffer_.writeWaistTarget(cmd);
}

void ExternalInterface::onVelocityCmd(const vr::VelocityCmd& cmd) {
  MotionCommand motion_cmd;
  motion_cmd.linear_x = cmd.linear_x;
  motion_cmd.linear_y = cmd.linear_y;
  motion_cmd.angular_z = cmd.angular_z;
  motion_cmd.valid = true;
  cmd_buffer_.writeCmdVel(motion_cmd);

  // RL_LOGD("ExternalInterface: Velocity cmd (%.3f, %.3f, %.3f)",
  //         cmd.linear_x, cmd.linear_y, cmd.angular_z);
}

// ============================================================================
// RPC 处理器（同步返回）
// ============================================================================

bool ExternalInterface::onSwitchControllerRequest(const std::string& name,
                                                   std::string& message) {
  if (!trigger_buffer_ || !controller_manager_) {
    message = "ExternalInterface not initialized";
    RL_LOGW("ExternalInterface: %s", message.c_str());
    return false;
  }

  if (!lifecycle_ || !lifecycle_->isRunning()) {
    message = "Controller not started yet";
    RL_LOG_WARNING("ExternalInterface: %s", message.c_str());
    return false;
  }

  // 查询可用性（同步返回）
  if (!controller_manager_->hasController(name)) {
    message = "Controller '" + name + "' not found";
    RL_LOGW("ExternalInterface: %s", message.c_str());
    return false;
  }

  // 控制请求写入 TriggerBuffer，由 ControlLoop 异步处理
  ActionTrigger trigger = MakeSwitchControllerTrigger(name);
  trigger_buffer_->push(trigger);

  message = "Switch request queued for controller: " + name;
  return true;
}

bool ExternalInterface::onSetArmModeRequest(ControlMode mode,
                                             std::string& message) {
  auto arm_mode = ConvertVRControlModeToArmMode(mode);
  if (!arm_mode.has_value()) {
    message = "Invalid arm mode";
    RL_LOGW("ExternalInterface: %s", message.c_str());
    return false;
  }

  if (!trigger_buffer_ || !controller_manager_) {
    message = "ExternalInterface not initialized";
    RL_LOGW("ExternalInterface: %s", message.c_str());
    return false;
  }

  if (!lifecycle_ || !lifecycle_->isRunning()) {
    message = "Controller not started yet";
    RL_LOG_WARNING("ExternalInterface: %s", message.c_str());
    return false;
  }

  // 创建 Trigger 异步处理
  std::string mode_name = ArmControlModeToString(*arm_mode);
  trigger_buffer_->push(MakeSetArmModeTrigger(mode_name));
  RL_LOGI("ExternalInterface: Push SetArmMode trigger: %s", mode_name.c_str());

  message = "Arm mode request queued: " + mode_name;
  return true;
}

bool ExternalInterface::onSetWaistModeRequest(ControlMode mode,
                                               std::string& message) {
  auto waist_mode = ConvertVRControlModeToWaistMode(mode);
  if (!waist_mode.has_value()) {
    message = "Invalid waist mode";
    RL_LOGW("ExternalInterface: %s", message.c_str());
    return false;
  }

  if (!trigger_buffer_ || !controller_manager_) {
    message = "ExternalInterface not initialized";
    RL_LOGW("ExternalInterface: %s", message.c_str());
    return false;
  }

  if (!lifecycle_ || !lifecycle_->isRunning()) {
    message = "Controller not started yet";
    RL_LOG_WARNING("ExternalInterface: %s", message.c_str());
    return false;
  }

  // 创建 Trigger 异步处理
  std::string mode_name = WaistControlModeToString(*waist_mode);
  trigger_buffer_->push(MakeSetWaistModeTrigger(mode_name));

  message = "Waist mode request queued: " + mode_name;
  return true;
}

bool ExternalInterface::onStartRuntimeRequest(std::string& message) {
  if (!trigger_buffer_ || !lifecycle_) {
    message = "ExternalInterface not initialized";
    return false;
  }
  if (lifecycle_->state() == LifecycleState::kRunning) {
    message = "Runtime already running";
    return true;
  }
  if (lifecycle_->state() == LifecycleState::kExiting) {
    message = "Runtime is exiting";
    return false;
  }

  // start 语义对齐手柄 START，由 ControlLoop 在下一周期消费。
  trigger_buffer_->push(ActionTrigger(ActionType::Start));
  message = "Start request queued";
  return true;
}

bool ExternalInterface::onStopRuntimeRequest(std::string& message) {
  if (!trigger_buffer_ || !lifecycle_) {
    message = "ExternalInterface not initialized";
    return false;
  }
  if (lifecycle_->state() == LifecycleState::kExiting) {
    message = "Runtime already exiting";
    return true;
  }

  // stop 语义对齐手柄 BACK，由 runtime 自己执行优雅退出流程。
  trigger_buffer_->push(MakeQuitTrigger());
  message = "Stop request queued";
  return true;
}

bool ExternalInterface::onStartMotionRequest(const std::string& name,
                                             std::string& message) {
  if (!trigger_buffer_ || !controller_manager_ || !lifecycle_) {
    message = "ExternalInterface not initialized";
    return false;
  }
  if (!lifecycle_->isRunning()) {
    message = "Controller not started yet";
    return false;
  }

  // motion 请求只负责入队，是否支持该 motion 由后续控制器自行决定。
  trigger_buffer_->push(
      MakeMotionCommandTrigger(MotionCommandArgs::Operation::Start, name));
  message = name.empty() ? "Motion start request queued"
                         : "Motion start request queued: " + name;
  return true;
}

vr::RuntimeState ExternalInterface::onGetRuntimeStateRequest() {
  vr::RuntimeState state;

  if (!controller_manager_) {
    RL_LOGW("ExternalInterface: ControllerManager not available for getRuntimeState request");
    return state;
  }

  const RobotData& robot_data = controller_manager_->getRobotData();
  state.data_ready = robot_data.isDataReady();
  state.hardware_state = ConvertHardwareStateToVr(robot_data.getHardwareState());
  if (lifecycle_) {
    state.lifecycle_state = ConvertLifecycleStateToVr(lifecycle_->state());
  }

  return state;
}

vr::ControllerState ExternalInterface::onGetControllerStateRequest() {
  vr::ControllerState state;

  if (!controller_manager_) {
    RL_LOGW("ExternalInterface: ControllerManager not available for getControllerState request");
    return state;
  }

  // 组装控制器状态
  state.current_controller = controller_manager_->getCurrentControllerName();
  state.available_controllers = controller_manager_->getControllerNames();
  state.controller_transitioning = controller_manager_->isTransitioning();

  // 获取手臂模式并转换
  auto arm_mode_opt = controller_manager_->getCurrentArmMode();
  if (arm_mode_opt.has_value()) {
    state.arm_mode = ConvertArmModeToVRControlMode(arm_mode_opt.value());
  }

  // 获取腰部模式并转换
  auto waist_mode_opt = controller_manager_->getCurrentWaistMode();
  if (waist_mode_opt.has_value()) {
    state.waist_mode = ConvertWaistModeToVRControlMode(waist_mode_opt.value());
  }

  return state;
}

vr::MotionState ExternalInterface::onGetMotionStateRequest() {
  vr::MotionState state;

  if (!controller_manager_) {
    RL_LOGW("ExternalInterface: ControllerManager not available for getMotionState request");
    return state;
  }

  state.available_motion_names = controller_manager_->getAvailableMotionNames();
  state.supported = !state.available_motion_names.empty();
  state.motion_playing = controller_manager_->isCurrentMotionPlaying();
  state.current_motion_name = controller_manager_->getCurrentMotionName();

  return state;
}

CommandBuffer::Snapshot ExternalInterface::getSnapshot() const {
  return cmd_buffer_.getSnapshot();
}

} // namespace runtime
} // namespace leju
