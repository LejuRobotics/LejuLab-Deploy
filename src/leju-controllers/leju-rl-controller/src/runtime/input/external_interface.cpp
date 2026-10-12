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

#include <algorithm>
#include <chrono>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>

#include <magic_enum/magic_enum.hpp>
#include <yaml-cpp/yaml.h>

#include "leju-rl-controller/controllers/controller_manager.h"
#include "leju-rl-controller/runtime/lifecycle.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/input/action_trigger.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/rl/multi_mode_arm_controller.h"
#include "leju-rl-controller/rl/waist_controller.h"
#include "leju-rl-controller/rl_log.h"
#include "leju-rl-controller/motion/tact_player.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/time_utils.hpp"
#include "lejusdk-vr/vr_api/vr_base.h"

#include <pwd.h>
#include <unistd.h>

namespace leju {
namespace runtime {

namespace {
// tact 状态使用 BEST_EFFORT QoS（状态类，避免 write 阻塞等待 ACK）。
// 与 dds_communicator.cpp:14-19 的 makeBestEffortQos 同实现。
dds::pub::qos::DataWriterQos makeBestEffortQos() {
  dds::pub::qos::DataWriterQos qos;
  qos << dds::core::policy::Reliability::BestEffort();
  return qos;
}

// tact 文件所在目录：$HOME/.config/lejuconfig/action_files（与 lejulab 其他配置一致）
std::string GetTactActionDir() {
  const char* home = getenv("HOME");
  std::string home_dir = home ? home : "";
  if (home_dir.empty()) {
    struct passwd* pw = getpwuid(getuid());
    if (pw) home_dir = pw->pw_dir;
  }
  return home_dir + "/.config/lejuconfig/action_files";
}

// tact servos 数组中腰部关节的索引：
//   roban (major=1): 22，kuavo 5.x (major=5): 28，kuavo 4.x (major=4): 无腰部
// 仅当机器人确有腰部关节 (waist_joint_count>0) 时返回索引，否则 -1。
int GetTactWaistIndex(const RobotVersion& version, size_t waist_joint_count) {
  if (waist_joint_count == 0) return -1;
  if (version.start_with(5)) return 28;   // kuavo 5.x
  return 22;                              // roban 等带腰部机型
}

// tact servos 数组中手部的起始索引，-1 表示无手部。
// Roban tact 布局: servos[0:8] 手臂, servos[8:20] 双手, servos[22] 腰部。
int GetTactHandIndex(const RobotVersion& version) {
  if (version.start_with(1)) return 8;   // roban
  return -1;
}

// 解码 "<name>\t<target_ns>"；无 '\t' 为纯动作名，target=0 立即播。
std::string SplitNameTarget(const std::string& data, int64_t* target_ns) {
  if (target_ns) *target_ns = 0;
  const std::size_t tab = data.find('\t');
  if (tab == std::string::npos) {
    return data;
  }
  const std::string name = data.substr(0, tab);
  try {
    const int64_t ns = std::stoll(data.substr(tab + 1));
    if (target_ns) *target_ns = ns;
  } catch (const std::exception&) {
    // target 段非法时按立即播处理，不因编码损坏阻塞动作
  }
  return name;
}
}  // namespace

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

  // 从 topic 订阅 tact 播放命令（NX bridge 从 DDS topic 传动作名，绕开跨语言 RPC）
  vr_api_->subscribeTactCommand(
      [this](const std::string& name) {
        std::string msg;
        onPlayTactRequest(name, msg);
      });

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

  // tact 播放器（独立于 CSV motion）。复用 cmd_buffer_ / trigger_buffer_，
  // arm_dof 用关节数，waist_index 按机器人版本自动判断。
  tact_player_ = std::make_unique<TactPlayer>(
      &cmd_buffer_, trigger_buffer_, arm_joint_count_,
      GetTactWaistIndex(version, waist_joint_count_),
      GetTactHandIndex(version),
      GetTactActionDir());

  // tact 状态 1Hz 发布（路径 A：自建 participant + TopicPublisher + Loop）。
  // 仿 main.cpp:259-266 audio publisher 先例。BestEffort QoS（状态类，避免 write 阻塞）。
  // Loop 回调每 1000ms 读 tact_player_ 状态快照组装发布，严格 1Hz，状态变化不额外补发。
  tact_state_participant_.emplace(0);
  tact_state_pub_ = std::make_unique<leju::dds_common::TopicPublisher<leju::msgs::TactState>>(
      *tact_state_participant_, leju::dds_topics::kTactState, makeBestEffortQos());
  tact_state_loop_ = std::make_unique<leju::Loop>(
      "tact_state_pub",
      [this]() { publishTactState(); },
      1000);  // period_ms = 1000 -> 1Hz
  tact_state_loop_->start();

  vr_api_->registerPlayTactHandler(
      [this](const std::string& name, std::string& message) -> bool {
        return onPlayTactRequest(name, message);
      });

  // 注册 tact fallback：CSV motion 找不到时降级到 .tact 播放
  controller_manager_->setTactFallback(
      [this](const std::string& name) {
        return playTactAsMotionFallback(name);
      });

  // 注册手臂冻结回调：LB+B keep_pose 时通知 TactPlayer 冻结
  controller_manager_->setArmFreezeCallback([this]() {
    if (tact_player_ && tact_player_->IsPlaying()) {
      RL_LOGI("ExternalInterface: LB+B freeze, stopping tact playback without Auto restore");
      tact_player_->Freeze();
    }
  });
  // 接收手柄发送高度指令
  posture_height_sub_ = std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::Float64>>(
      dds_participant_,
      leju::dds_topics::kPostureHeightCmd,
      [this](const leju::msgs::Float64& msg) { onPostureHeightCmd(msg.data()); });
  RL_LOGI("ExternalInterface: Subscribed to %s", leju::dds_topics::kPostureHeightCmd);

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
  // 先停 tact 状态发布 Loop（stop + join 线程），再释放 publisher/participant，
  // 避免 Loop 线程访问已释放的对象。Loop::shutdown() 安全停止并等待线程结束。
  if (tact_state_loop_) {
    tact_state_loop_->shutdown();
    tact_state_loop_.reset();
  }
  tact_state_pub_.reset();
  tact_state_participant_.reset();

  posture_height_sub_.reset();
  if (tact_player_) {
    tact_player_->Stop();
    tact_player_.reset();
  }
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

bool ExternalInterface::loadVelocityLimitsFromTeleopConfig(const std::string& config_path) {
  try {
    TeleopConfig config = velocity_limits_;
    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node velocity_limits = root["velocity_limits"];

    if (!velocity_limits) {
      RL_LOGW("ExternalInterface: 'velocity_limits' missing in %s, using TeleopConfig"
              " defaults",
              config_path.c_str());
    } else {
      if (velocity_limits["stick_deadzone"]) {
        config.stick_deadzone = velocity_limits["stick_deadzone"].as<float>();
      }
      if (velocity_limits["linear_x"]) {
        config.max_linear_x = velocity_limits["linear_x"].as<double>();
      }
      if (velocity_limits["linear_y"]) {
        config.max_linear_y = velocity_limits["linear_y"].as<double>();
      }
      if (velocity_limits["angular_z"]) {
        config.max_angular_z = velocity_limits["angular_z"].as<double>();
      }
    }

    velocity_limits_ = config;
    RL_LOGI("ExternalInterface: Loaded normalized velocity limits from %s"
            " (linear_x=%.3f, linear_y=%.3f, angular_z=%.3f)",
            config_path.c_str(),
            velocity_limits_.max_linear_x,
            velocity_limits_.max_linear_y,
            velocity_limits_.max_angular_z);
    return true;
  } catch (const std::exception& e) {
    RL_LOGW("ExternalInterface: Failed to parse velocity limits from %s: %s",
            config_path.c_str(), e.what());
    return false;
  }
}

bool ExternalInterface::loadPostureHeightConfig(const std::string& config_path,
                                                double standing_height,
                                                double timeout_sec) {
  try {
    YAML::Node root = YAML::LoadFile(config_path);
    posture_standing_height_ = standing_height;
    posture_timeout_sec_ = timeout_sec;

    YAML::Node env = root["HumanoidRobotCfg"]["env"];
    YAML::Node posture_axis = env ? env["amp_hand_posture_axis"] : YAML::Node();
    if (posture_axis) {
      if (posture_axis["squat_height_min"]) {
        posture_squat_height_min_ = posture_axis["squat_height_min"].as<double>();
      }
      if (posture_axis["squat_height_max"]) {
        posture_squat_height_max_ = posture_axis["squat_height_max"].as<double>();
      }
    } else {
      RL_LOGW("ExternalInterface: amp_hand_posture_axis missing in %s, using defaults",
              config_path.c_str());
    }

    RL_LOGI("ExternalInterface: Posture height mapping loaded from %s "
            "(standing=%.3f m, squat=[%.3f, %.3f], timeout=%.2f s)",
            config_path.c_str(),
            posture_standing_height_,
            posture_squat_height_min_,
            posture_squat_height_max_,
            posture_timeout_sec_);
    return true;
  } catch (const std::exception& e) {
    RL_LOGW("ExternalInterface: Failed to load posture height config from %s: %s",
            config_path.c_str(), e.what());
    return false;
  }
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
  const double normalized_x = std::clamp(cmd.linear_x, -1.0, 1.0);
  const double normalized_y = std::clamp(cmd.linear_y, -1.0, 1.0);
  const double normalized_z = std::clamp(cmd.angular_z, -1.0, 1.0);
  motion_cmd.linear_x = normalized_x * velocity_limits_.max_linear_x;
  motion_cmd.linear_y = normalized_y * velocity_limits_.max_linear_y;
  motion_cmd.angular_z = normalized_z * velocity_limits_.max_angular_z;
  motion_cmd.valid = true;
  cmd_buffer_.writeCmdVel(motion_cmd);
  velocity_last_rx_time_sec_.store(common::GetSteadyTimestampNs() * 1e-9);

  // RL_LOGD("ExternalInterface: Velocity cmd (%.3f, %.3f, %.3f)",
  //         cmd.linear_x, cmd.linear_y, cmd.angular_z);
}

void ExternalInterface::onPostureHeightCmd(double height_m) {
  posture_latest_height_m_.store(height_m);
  posture_last_rx_time_sec_.store(common::GetSteadyTimestampNs() * 1e-9);
}

std::optional<MotionCommand> ExternalInterface::buildFreshPostureCommand(
    double now_sec, const MotionCommand& base_vel) const {
  const double last_rx = posture_last_rx_time_sec_.load();
  if (last_rx <= 0.0 || now_sec - last_rx > posture_timeout_sec_) {
    // 超时/清零：若此前曾进入下蹲（cmd_stance=1），必须补发站立退出指令。
    // control_logic 只在 cmd_stance_valid=true 时调用 setCmdStanceMode，
    // 若这里直接返回 nullopt，缺少退出指令会导致 cmd_stance 永久卡在 1（下蹲）。
    if (posture_was_used_) {
      const int cur_stance =
          controller_manager_ ? controller_manager_->getCurrentCmdStanceMode() : 1;
      if (cur_stance != 0) {
        MotionCommand exit_cmd;
        exit_cmd.valid = true;
        exit_cmd.cmd_stance_valid = true; // 触发站立恢复
        exit_cmd.cmd_stance_mode = 0;
        exit_cmd.linear_x = base_vel.linear_x;
        exit_cmd.linear_y = base_vel.linear_y;
        exit_cmd.angular_z = base_vel.angular_z;
        return exit_cmd;
      }
      // 控制器已确认退出下蹲，恢复休眠，让出输入源优先级。
      posture_was_used_ = false;
    }
    return std::nullopt;
  }

  const double height_m = posture_latest_height_m_.load();
  const double squat_cmd = std::clamp(height_m - posture_standing_height_,
                                      posture_squat_height_min_,
                                      posture_squat_height_max_);

  MotionCommand cmd;
  cmd.valid = true;
  cmd.cmd_stance_valid = true;
  cmd.cmd_stance_mode = 1;
  cmd.linear_x = 0.0;
  cmd.linear_y = 0.0;
  cmd.angular_z = squat_cmd;
  posture_was_used_ = true;
  return cmd;
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

bool ExternalInterface::onPlayTactRequest(const std::string& data,
                                          std::string& message) {
  if (!tact_player_ || !lifecycle_) {
    message = "ExternalInterface not initialized";
    return false;
  }
  if (!lifecycle_->isRunning()) {
    const std::string name = SplitNameTarget(data, nullptr);
    // controller 未启动：Play 不会被调用，在此显式标记 failed + controller_not_running，
    // 供 1Hz Loop 上报（终态保持到下次 Play 成功覆盖）。
    tact_player_->markFailed(leju::runtime::kTactErrControllerNotRunning,
                             "controller not running", name);
    message = "Controller not started yet";
    return false;
  }

  // 解码 "<name>\t<target_ns>"；无 \t 为纯动作名，target=0 立即播
  int64_t target_execution_time_ns = 0;
  const std::string name = SplitNameTarget(data, &target_execution_time_ns);

  // 读取当前手臂/腰部关节位置（度），用于 t=0 平滑过渡，避免关节阶跃
  std::vector<double> init_arm_pos;
  std::vector<double> init_waist_pos;
  getCurrentArmWaistPosDeg(init_arm_pos, init_waist_pos);

  if (target_execution_time_ns > 0) {
    RL_LOGI("ExternalInterface: tact '%s' target_execution_time_ns=%lld (%.1f s ahead)",
            name.c_str(), static_cast<long long>(target_execution_time_ns),
            (target_execution_time_ns - std::chrono::system_clock::now().time_since_epoch().count())
                / 1e9);
  }

  // tact 播放走 External arm mode 直推 cmd_buffer_，与 CSV motion 完全独立。
  return tact_player_->Play(name, init_arm_pos, init_waist_pos, &message,
                            target_execution_time_ns);
}

void ExternalInterface::getCurrentArmWaistPosDeg(
    std::vector<double>& arm_pos, std::vector<double>& waist_pos) const {
  arm_pos.clear();
  waist_pos.clear();
  if (!controller_manager_ || !robot_api_) return;

  RobotState state;
  if (!controller_manager_->getRobotData().getRobotState(state) || state.q.empty()) return;

  const auto motor_names = robot_api_->getMotorNames();

  const auto arm_names = robot_api_->getArmJointNames();
  for (const auto& arm_name : arm_names) {
    auto it = std::find(motor_names.begin(), motor_names.end(), arm_name);
    if (it != motor_names.end()) {
      size_t idx = static_cast<size_t>(std::distance(motor_names.begin(), it));
      if (idx < state.q.size()) {
        arm_pos.push_back(state.q[idx] * 180.0 / M_PI);  // rad → deg
      }
    }
  }

  const auto waist_names = robot_api_->getWaistJointNames();
  for (const auto& waist_name : waist_names) {
    auto it = std::find(motor_names.begin(), motor_names.end(), waist_name);
    if (it != motor_names.end()) {
      size_t idx = static_cast<size_t>(std::distance(motor_names.begin(), it));
      if (idx < state.q.size()) {
        waist_pos.push_back(state.q[idx] * 180.0 / M_PI);  // rad → deg
      }
    }
  }
}

void ExternalInterface::publishTactState() {
  if (!tact_player_ || !tact_state_pub_) {
    return;
  }

  // 读状态快照（tact_player_ 内部线程安全：state_/error_code_ atomic，
  // last_name_/error_message_ 由 state_mutex_ 保护）。
  leju::msgs::TactState msg;
  int32_t sec = 0;
  uint32_t nsec = 0;
  leju::common::GetUnixTimestampSecNsec(sec, nsec);
  msg.header_sec(sec);
  msg.header_nanosec(nsec);
  msg.name(tact_player_->getLastName());
  msg.state(tact_player_->getState());
  msg.error_code(tact_player_->getErrorCode());
  msg.error_message(tact_player_->getErrorMessage());

  tact_state_pub_->publish(msg);
}

bool ExternalInterface::playTactAsMotionFallback(const std::string& name) {
  if (!tact_player_) return false;

  // 检查 .tact 文件是否存在
  std::string tact_path = GetTactActionDir() + "/" + name + ".tact";
  std::ifstream f(tact_path);
  if (!f.good()) {
    RL_LOGW("MotionCommand '%s': CSV not found, tact file not found at %s",
            name.c_str(), tact_path.c_str());
    return false;
  }
  f.close();

  std::vector<double> init_arm_pos;
  std::vector<double> init_waist_pos;
  getCurrentArmWaistPosDeg(init_arm_pos, init_waist_pos);

  RL_LOGI("MotionCommand '%s': CSV not found, falling back to tact playback", name.c_str());
  std::string msg;
  return tact_player_->Play(name, init_arm_pos, init_waist_pos, &msg);
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
  CommandBuffer::Snapshot snapshot = cmd_buffer_.getSnapshot();
  const double now_sec = common::GetSteadyTimestampNs() * 1e-9;
  const double velocity_last_rx = velocity_last_rx_time_sec_.load();
  if (velocity_last_rx <= 0.0 ||
      now_sec - velocity_last_rx > kVelocityCmdTimeoutSec) {
    snapshot.cmd_vel = MotionCommand{};
  }
  auto posture_cmd = buildFreshPostureCommand(now_sec, snapshot.cmd_vel);
  if (posture_cmd != std::nullopt) {
    snapshot.cmd_vel = *posture_cmd;
  }
  return snapshot;
}

void ExternalInterface::clearCmdVel() {
  cmd_buffer_.clearCmdVel();
  velocity_last_rx_time_sec_.store(0.0);
  posture_last_rx_time_sec_.store(0.0);
}

} // namespace runtime
} // namespace leju
