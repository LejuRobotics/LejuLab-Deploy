#include "leju-rl-controller/controllers/controller_manager.h"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "leju-rl-controller/controllers/controller_registry.h"
#include "leju-rl-controller/controllers/generic_rl_controller.h"
#include "leju-rl-controller/controllers/depth_walk_controller.h"
#include "leju-rl-controller/controllers/hold_pose_controller.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/rl_log.h"
#include "leju-rl-controller/utils/uri_path_resolver.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/time_utils.hpp"

namespace leju {

namespace {
struct HoldPoseForceLink {
  HoldPoseForceLink() { ForceLinkHoldPoseController(); }
} g_hold_pose_force_link;
}  // namespace

bool IsFallenPose(const ImuData& imu, double threshold_deg) {
  Eigen::Quaterniond quat(imu.quat[0], imu.quat[1], imu.quat[2], imu.quat[3]);
  const Eigen::Matrix3d R = quat.normalized().toRotationMatrix();
  // ZYX 约定：pitch = atan2(-R20, sqrt(R21^2+R22^2)), roll = atan2(R21, R22)
  const double pitch = std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
  const double roll = std::atan2(R(2, 1), R(2, 2));
  const double pitch_deg = std::abs(pitch) * 180.0 / M_PI;
  const double roll_deg = std::abs(roll) * 180.0 / M_PI;
  return (roll_deg > threshold_deg) || (pitch_deg > threshold_deg);
}

namespace {

//五次多项式插值
double QuinticBlend(double x) {
  if (x <= 0.0) return 0.0;
  if (x >= 1.0) return 1.0;
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

double BlendScalar(double source, double target, double blend) {
  return source + (target - source) * blend;
}

//插值力矩计算
RobotCmd BlendRobotCmd(const RobotCmd& source_cmd,
                       const RobotCmd& target_cmd,
                       const RobotCmd* source_ref_cmd,
                       const RobotCmd* target_ref_cmd,
                       const array_t* source_torque_limits,
                       const array_t* target_torque_limits,
                       const array_i* source_recompute_mask,
                       const array_i* target_recompute_mask,
                       const RobotState& state,
                       double alpha) {
  if (!source_cmd.isValid() || !target_cmd.isValid() ||
      source_cmd.q.size() != target_cmd.q.size()) {
    return target_cmd;
  }

  const double blend = QuinticBlend(alpha);
  RobotCmd blended = target_cmd;
  const size_t joint_count = source_cmd.q.size();
  blended.resize(joint_count);

  if (!source_ref_cmd || !target_ref_cmd ||
      !source_torque_limits || !target_torque_limits ||
      !source_recompute_mask || !target_recompute_mask ||
      !source_ref_cmd->isValid() || !target_ref_cmd->isValid() ||
      source_ref_cmd->q.size() != joint_count ||
      target_ref_cmd->q.size() != joint_count ||
      state.q.size() != joint_count ||
      state.v.size() != joint_count ||
      source_torque_limits->size() != joint_count ||
      target_torque_limits->size() != joint_count ||
      source_recompute_mask->size() != joint_count ||
      target_recompute_mask->size() != joint_count) {
    return blended;
  }

  for (size_t i = 0; i < joint_count; ++i) {
    //针对手臂和腰这种不是纯RL控制的关节，对输入到驱动器的力矩进行插值平滑输出
    if ((*source_recompute_mask)[i] == 0 || (*target_recompute_mask)[i] == 0){
      blended.tau[i] = BlendScalar(source_cmd.tau[i], target_cmd.tau[i], blend);
      blended.kp[i] = BlendScalar(source_cmd.kp[i], target_cmd.kp[i], blend);
      blended.kd[i] = BlendScalar(source_cmd.kd[i], target_cmd.kd[i], blend);
      blended.modes[i] = (alpha < 1.0) ? source_cmd.modes[i] : target_cmd.modes[i];
      continue;
    }

    const double q_target =
        BlendScalar((*source_ref_cmd).q[i], (*target_ref_cmd).q[i], blend);
    const double kp =
        BlendScalar((*source_ref_cmd).kp[i], (*target_ref_cmd).kp[i], blend);
    const double kd =
        BlendScalar((*source_ref_cmd).kd[i], (*target_ref_cmd).kd[i], blend);
    const int mode = (alpha < 0.5) ? (*source_ref_cmd).modes[i] : (*target_ref_cmd).modes[i];
    const double q_feedback = state.q[i];
    const double v_feedback = state.v[i];

    double tau = kp * (q_target - q_feedback);
    if (mode == 0) {
      tau += kd * (-v_feedback);
    }

    const double source_limit = (*source_torque_limits)[i];
    const double target_limit = (*target_torque_limits)[i];
    const double tau_limit = (source_limit > 0.0 && target_limit > 0.0)
                                 ? BlendScalar(source_limit, target_limit, blend)
                                 : std::max(source_limit, target_limit);
    if (tau_limit > 0.0) {
      tau = std::clamp(tau, -tau_limit, tau_limit);
    }

    blended.timestamp = target_cmd.timestamp;
    blended.q[i] = q_feedback;
    blended.v[i] = 0.0;
    blended.tau[i] = tau;
    blended.modes[i] = (mode == 0) ? 0 : 2;
    // if (mode == 0) {
    //   blended.kp[i] = 0.0;
    //   blended.kd[i] = 0.0;
    // } else {
    //   blended.kp[i] = kp;
    //   blended.kd[i] = kd;
    // }
  }

  return blended;
}

}  // namespace

ControllerManager::~ControllerManager() {
  // 停止 DDS 通信，确保回调不再被触发
  // 必须在 controllers_ 和 robot_data_ 析构之前调用
  // 否则回调会访问已释放的内存
  if (GlobalRobot::is_initialized()) {
    GlobalRobot::getInstance().shutdown();
  }
}

bool ControllerManager::initialize(const std::string& config_file, const std::string& urdf_path) {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  // 缓存路径用于后续热重载
  config_file_ = config_file;
  urdf_path_ = urdf_path;

  // 初始化 RobotData（订阅传感器数据）- 这会创建 DDS participant
  if (!robot_data_.initialize()) {
    RL_LOGE("Failed to initialize RobotData");
    return false;
  }

  // 从配置文件加载控制器
  if (!loadControllersFromConfig(config_file, urdf_path)) {
    RL_LOGE("Failed to load controllers from config");
    return false;
  }

  // 检查是否至少加载了一个控制器
  if (controllers_.empty()) {
    RL_LOG_FAILURE("No controllers loaded, exiting");
    return false;
  }

  GlobalRobot::getInstance().subscribeStopRobot(
      [this](const StringDataConstPtr&) { this->stop(); });

  // 订阅手柄数据：RT + 右摇杆控制头部（阈值与 teleop_bindings.yaml trigger_threshold 对齐）
  GlobalRobot::getInstance().subscribeJoyData(
      [this](const JoyDataConstPtr& joy) {
        std::lock_guard<std::mutex> lock(joy_mutex_);
        constexpr float kHeadJoyRtThreshold = 0.50f;
        constexpr float kDeadzone = 0.15f;
        constexpr float kHeadSpeed = 0.03f;
        constexpr float kHeadReturnSpeedRadPerSec = 1.5f;
        constexpr float kYawMax = static_cast<float>(M_PI_2);
        constexpr float kPitchMax = static_cast<float>(M_PI_4);

        const auto now = std::chrono::steady_clock::now();
        float dt = 0.01f;
        if (head_joy_time_initialized_) {
          dt = static_cast<float>(
              std::chrono::duration<double>(now - last_head_joy_update_).count());
          dt = std::clamp(dt, 0.001f, 0.1f);
        } else {
          head_joy_time_initialized_ = true;
        }
        last_head_joy_update_ = now;

        const bool rt_now = joy->axes.right_trigger >= kHeadJoyRtThreshold;
        const bool was_active = head_joy_active_;
        head_joy_active_ = rt_now;

        if (head_joy_active_) {
          head_return_active_ = false;
          float rx =
              std::abs(joy->axes.right_x) > kDeadzone ? joy->axes.right_x : 0.0f;
          float ry =
              std::abs(joy->axes.right_y) > kDeadzone ? joy->axes.right_y : 0.0f;
          head_joy_yaw_ -= rx * kHeadSpeed;
          head_joy_pitch_ += ry * kHeadSpeed;
          head_joy_yaw_ = std::clamp(head_joy_yaw_, -kYawMax, kYawMax);
          head_joy_pitch_ = std::clamp(head_joy_pitch_, -kPitchMax, kPitchMax);
          return;
        }

        if (was_active && !rt_now) {
          head_return_active_ = true;
        }

        if (!head_return_active_) {
          return;
        }

        const float max_delta = kHeadReturnSpeedRadPerSec * dt;
        auto stepTowardZero = [max_delta](float& angle) {
          if (std::abs(angle) <= max_delta) {
            angle = 0.0f;
            return;
          }
          angle -= std::copysign(max_delta, angle);
        };
        stepTowardZero(head_joy_yaw_);
        stepTowardZero(head_joy_pitch_);
        if (std::abs(head_joy_yaw_) < 1e-4f && std::abs(head_joy_pitch_) < 1e-4f) {
          head_joy_yaw_ = 0.0f;
          head_joy_pitch_ = 0.0f;
          head_return_active_ = false;
        }
      });

  return true;
}

bool ControllerManager::addController(const std::string& name,
                                       std::unique_ptr<ControllerBase> controller,
                                       const std::string& config_path) {
  // 检查是否已存在同名控制器
  if (hasController(name)) {
    RL_LOGE("Controller already exists: %s", name.c_str());
    return false;
  }

  if (controller) {
    controller->setDefaultPoseStopFlag(&default_pose_stop_requested_);
  }

  // 添加控制器（initialize 后已处于 kPaused 待命状态）
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);
  controllers_.push_back({name, config_path, std::move(controller), {}, 0.0});
  RL_LOGI("Added controller: %s", name.c_str());
  return true;
}

std::string ControllerManager::getControllerConfigPath(
    const std::string& name) const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);
  for (const auto& entry : controllers_) {
    if (entry.name == name) {
      return entry.config_path;
    }
  }
  return {};
}

bool ControllerManager::hasController(const std::string& name) const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  for (const auto& entry : controllers_) {
    if (entry.name == name) {
      return true;
    }
  }
  return false;
}

ControllerBase* ControllerManager::getControllerByName(const std::string& name) {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  for (auto& entry : controllers_) {
    if (entry.name == name) {
      return entry.controller.get();
    }
  }
  return nullptr;
}

size_t ControllerManager::getControllerCount() const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);
  return controllers_.size();
}

bool ControllerManager::requestSwitch(const std::string& name, double now, bool auto_start_motion,
                                      bool instant_commit) {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  // 查找目标控制器
  int target_index = -1;
  for (size_t i = 0; i < controllers_.size(); ++i) {
    if (controllers_[i].name == name) {
      target_index = static_cast<int>(i);
      break;
    }
  }

  if (target_index < 0) {
    RL_LOGE("Controller not found: %s", name.c_str());
    return false;
  }

  auto* depth_target =
      dynamic_cast<DepthWalkController*>(controllers_[target_index].controller.get());
  if (depth_target && !depth_target->depthReady()) {
    RL_LOGW("Cannot switch to depth controller: model/depth history is not ready");
    return false;
  }

  // 如果已经是当前控制器，直接返回
  if (target_index == active_index_) {
    RL_LOGW("Controller '%s' is already active, skip switch", name.c_str());
    return true;
  }

  // 如果当前控制器正在播放动作，不允许切换
  if (active_index_ >= 0) {
    auto* current_controller = controllers_[active_index_].controller.get();
    auto* generic_rl = dynamic_cast<GenericRLController*>(current_controller);
    if (generic_rl && generic_rl->isMotionPlaying()) {
      RL_LOG_WARNING("Cannot switch controller: '%s' is currently playing motion",
              controllers_[active_index_].name.c_str());
      return false;
    }
  }

  // 如果正在过渡中，不允许新的切换请求
  if (transition_.state == SwitchState::kTransitioning) {
    RL_LOGW("Controller switch already in progress: %s -> %s",
            transition_.from_controller.c_str(),
            transition_.to_controller.c_str());
    return false;
  }

  // 只有站立状态才允许切换控制器，行走中切换可能导致机器人失稳
  if (active_index_ >= 0) {
    auto* current_controller = controllers_[active_index_].controller.get();
    if (current_controller && !current_controller->isStanding()) {
      RL_LOG_WARNING("Cannot switch controller: robot is not standing (velocity commands non-zero)");
      return false;
    }
  }

  // 启动过渡流程
  std::string from_name = (active_index_ >= 0) ? controllers_[active_index_].name : "(none)";

  ControllerBase* from_controller = (active_index_ >= 0) ? controllers_[active_index_].controller.get() : nullptr;
  ControllerBase* to_controller = controllers_[target_index].controller.get();

  transition_.from_controller = from_name;
  transition_.to_controller = name;
  transition_.source_index = active_index_;
  transition_.target_index = target_index;
  transition_.start_time = now;
  transition_.pending_auto_start_motion = auto_start_motion;
  transition_.target_prestarted = false;

  // 立即 commit：跳过混合（倒地起身交接等场景，切后状态由目标控制器自身管理）
  if (instant_commit) {
    transition_.state = SwitchState::kTransitioning;
    transition_.rl_to_rl_dual_inference_active = false;
    RL_LOGI("RequestSwitch: %s -> %s (instant commit, skip blend %.3fs)",
            from_name.c_str(), name.c_str(), transition_.duration);
    commitSwitch();
    return true;
  }

  transition_.state = SwitchState::kTransitioning;
  transition_.rl_to_rl_dual_inference_active =
      (from_controller != nullptr && to_controller != nullptr);

  if (transition_.rl_to_rl_dual_inference_active) {
    RL_LOGI("Starting RL->RL dual inference switch: %s -> %s",
            from_name.c_str(), name.c_str());
    to_controller->resume();
    transition_.target_prestarted = true;
  }

  RL_LOGI("RequestSwitch: %s -> %s (start_time=%.3f, duration=%.3fs)",
          from_name.c_str(), name.c_str(), now, transition_.duration);

  return true;
}

void ControllerManager::commitSwitch() {
  int target_index = transition_.target_index;

  if (target_index < 0 || target_index >= static_cast<int>(controllers_.size())) {
    RL_LOGE("CommitSwitch: Target controller not found: %s",
            transition_.to_controller.c_str());
    transition_.state = SwitchState::kIdle;
    transition_.rl_to_rl_dual_inference_active = false;
    transition_.target_prestarted = false;
    transition_.source_index = -1;
    transition_.target_index = -1;
    return;
  }

  const int source_index = transition_.source_index;
  std::string from_name =
      (source_index >= 0 && source_index < static_cast<int>(controllers_.size()))
          ? controllers_[source_index].name
          : "(none)";

  // 暂停当前控制器（OnExit）
  if (source_index >= 0 && source_index < static_cast<int>(controllers_.size())) {
    RL_LOGI("Pausing current controller: %s", from_name.c_str());
    controllers_[source_index].controller->pause();
  }

  // 记录上一个控制器索引，切换到新控制器
  last_index_ = source_index;
  active_index_ = target_index;

  // 恢复新控制器（OnEnter）
  auto* new_controller = controllers_[active_index_].controller.get();
  if (!transition_.target_prestarted) {
    RL_LOGI("Resuming new controller: %s", transition_.to_controller.c_str());
    new_controller->resume();
  } else {
    RL_LOGI("Dual inference switch committed, target controller kept running: %s",
            transition_.to_controller.c_str());
  }

  // 重置过渡状态
  // 注意：pending_auto_start_motion 不在此处重置，留给 update() 做稳定性检测后再起播
  transition_.state = SwitchState::kIdle;
  transition_.rl_to_rl_dual_inference_active = false;
  transition_.target_prestarted = false;
  transition_.stable_ticks = 0;   // 开始新一轮稳定计数
  transition_.source_index = -1;
  transition_.target_index = -1;

  RL_LOG_SUCCESS("CommitSwitch complete: %s -> %s", from_name.c_str(),
          transition_.to_controller.c_str());
}

bool ControllerManager::isTransitioning() const {
  return transition_.state == SwitchState::kTransitioning;
}

void ControllerManager::Start() {
  default_pose_stop_requested_.store(false, std::memory_order_relaxed);
  running_.store(true, std::memory_order_relaxed);

  // 恢复当前控制器（从 PAUSED -> RUNNING）
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);
  if (active_index_ >= 0) {
    controllers_[active_index_].controller->resume();
    RL_LOGI("ControllerManager started, controller '%s' resumed",
            controllers_[active_index_].name.c_str());
  } else {
    RL_LOGW("ControllerManager started, but no active controller");
  }
}

bool ControllerManager::waitForDataReady() {
  RL_LOGI("Waiting for sensor data...");

  int wait_count = 0;
  while (running_ && !robot_data_.isDataReady()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (++wait_count % (10 * 3) == 0) {  // 每3秒打印一次
      RL_LOGI("Waiting for sensor data... (%d s)", wait_count / 10);
    }
  }
  if (!running_) return false;

  // 等待硬件就绪
  while (running_ && !robot_data_.isHardwareReady()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!running_) return false;

  RL_LOGI("Sensor data ready");
  return true;
}

void ControllerManager::stop() {
  default_pose_stop_requested_.store(true, std::memory_order_relaxed);
  const bool was_running = running_.exchange(false);

  if (!was_running) {
    return;
  }

  // 停止当前控制器
  {
    std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);
    if (active_index_ >= 0 && active_index_ < static_cast<int>(controllers_.size())) {
      controllers_[active_index_].controller->stop();
    }
  }

  GlobalRobot::getInstance().publishStopRobot();

  std::cout << std::endl;
  std::cout << "\033[1;33m------------------------------------------------\033[0m" << std::endl;
  std::cout << "\033[1;33m- RL policy stopped\033[0m" << std::endl;
  std::cout << "\033[1;33m------------------------------------------------\033[0m" << std::endl;
  std::cout.flush();
}

RobotCmd ControllerManager::update(const RobotState& state,
                                   const ImuData& imu_state,
                                   const runtime::CommandBuffer::Snapshot& command) {
  RobotCmd cmd;

  if (!running_.load()) {
    return cmd;
  }

  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  // 处理过渡状态
  double transition_elapsed = -1.0;  // < 0 表示不在过渡中
  if (transition_.state == SwitchState::kTransitioning) {
    double time = common::GetSteadyTimestampNs() * 1e-9;
    transition_elapsed = time - transition_.start_time;
    double progress = transition_elapsed / transition_.duration;

    // 每 10% 打印一次进度
    static int last_progress_printed = -1;
    int current_progress = static_cast<int>(progress * 10);
    if (current_progress > last_progress_printed && current_progress < 10) {
      RL_LOGI("Controller switch progress: %s -> %s (%d%%)",
              transition_.from_controller.c_str(),
              transition_.to_controller.c_str(),
              current_progress * 10);
      last_progress_printed = current_progress;
    }

    if (progress >= 1.0) {
      // 过渡完成，提交切换
      commitSwitch();
      transition_elapsed = -1.0;
      last_progress_printed = -1;
    }
  }

  // 自动起播：切换完成后等待机器人物理稳定（gyro 检测），再起播舞蹈
  // 用 IMU 陀螺仪数据判断机身是否静止，避免固定时间延迟在不同工况下不可靠
  if (transition_.pending_auto_start_motion && transition_.state == SwitchState::kIdle) {
    constexpr double kGyroStableThresh = 0.3;   // rad/s, 机身角速度低于此值视为稳定
    constexpr int kStableTicksRequired = 500;    // 连续稳定帧数（500 帧 @ 1kHz = 0.5s）

    double gyro_mag = std::sqrt(
        imu_state.gyro[0] * imu_state.gyro[0] +
        imu_state.gyro[1] * imu_state.gyro[1] +
        imu_state.gyro[2] * imu_state.gyro[2]);

    if (gyro_mag < kGyroStableThresh) {
      transition_.stable_ticks++;
      if (transition_.stable_ticks >= kStableTicksRequired) {
        RL_LOGI("CommitSwitch: robot stabilized (gyro=%.3f rad/s), auto-starting motion",
                gyro_mag);
        startMotion();
        transition_.pending_auto_start_motion = false;
        transition_.stable_ticks = 0;
      }
    } else {
      if (transition_.stable_ticks > 0) {
        RL_LOGD("CommitSwitch: gyro=%.3f rad/s exceeded threshold, reset stable counter",
                gyro_mag);
      }
      transition_.stable_ticks = 0;
    }
  }

  if (active_index_ < 0) {
    return cmd;
  }

  // 从 CommandSnapshot 中提取速度指令（输入优先级已在 ControlLoop::mergeAllCmdVel() 中选择）
  auto* controller = controllers_[active_index_].controller.get();
  VelocityCommand vel_cmd;
  vel_cmd.linear_x = command.cmd_vel.linear_x;
  vel_cmd.linear_y = command.cmd_vel.linear_y;
  vel_cmd.angular_z = command.cmd_vel.angular_z;

  // 深蹲防护：激活后限制行走线速度，只保留角速度做转向
  if (controller->isDeepSquatGuardActive()) {
    const bool exit_posture =
        command.cmd_vel.cmd_stance_valid &&
        command.cmd_vel.cmd_stance_mode == 0;
    vel_cmd.linear_x = 0.0;
    vel_cmd.linear_y = 0.0;
    if (exit_posture) {
      vel_cmd.angular_z = 0.0;
    } else if (command.cmd_vel.cmd_stance_valid &&
               command.cmd_vel.cmd_stance_mode != 0) {
      vel_cmd.angular_z = command.cmd_vel.angular_z;
    } else {
      vel_cmd.angular_z = 0.0;
    }
  }

  // 控制器切换过渡期间不响应速度指令，保持站立状态确保切换安全
  if (transition_.state == SwitchState::kTransitioning) {
    vel_cmd.setZero();
  }

  // RT + 右摇杆控制头部时，禁用行走角速度；posture 模式下 angular_z 为下蹲高度，需保留
  if (head_joy_active_) {
    const bool in_posture =
        command.cmd_vel.cmd_stance_valid && command.cmd_vel.cmd_stance_mode == 1;
    if (!in_posture) {
      vel_cmd.angular_z = 0.0;
    }
  }

  // M1/M2 组合键动作播放期间屏蔽所有摇杆速度输入
  // 注意：M1/M2 动作走 TactPlayer 播放，不走 GenericRLController::startMotion()，
  // 因此不检查 motion_playing_（该标志对 tact 播放始终为 false）
  {
    auto* generic_rl = dynamic_cast<GenericRLController*>(controller);
    if (generic_rl && generic_rl->isBlockingVelocityInMotion()) {
      vel_cmd.setZero();
    }
  }

  controller->setVelocityCommand(vel_cmd);

  double time = common::GetSteadyTimestampNs() * 1e-9;
  bool update_ok = false;
  if (transition_.state == SwitchState::kTransitioning &&
      transition_.rl_to_rl_dual_inference_active &&
      transition_.source_index >= 0 &&
      transition_.target_index >= 0 &&
      transition_.source_index < static_cast<int>(controllers_.size()) &&
      transition_.target_index < static_cast<int>(controllers_.size()) &&
      transition_elapsed >= 0.0 && transition_elapsed <= transition_.duration) {
    auto* source_controller = controllers_[transition_.source_index].controller.get();
    auto* target_controller = controllers_[transition_.target_index].controller.get();
    RobotCmd source_cmd;
    RobotCmd target_cmd;
    VelocityCommand zero_vel_cmd;
    zero_vel_cmd.setZero();

    if (source_controller) source_controller->setVelocityCommand(zero_vel_cmd);
    if (target_controller) target_controller->setVelocityCommand(zero_vel_cmd);

    const bool source_ok =
        source_controller && source_controller->update(time, state, imu_state, source_cmd);
    const bool target_ok =
        target_controller && target_controller->update(time, state, imu_state, target_cmd);

    if (source_ok && target_ok) {
      const double alpha = std::clamp(transition_elapsed / transition_.duration, 0.0, 1.0);
      cmd = BlendRobotCmd(source_cmd,
                          target_cmd,
                          source_controller->getDualInferenceBlendReferenceCmd(),
                          target_controller->getDualInferenceBlendReferenceCmd(),
                          source_controller->getDualInferenceTorqueLimits(),
                          target_controller->getDualInferenceTorqueLimits(),
                          source_controller->getDualInferenceRecomputeMask(),
                          target_controller->getDualInferenceRecomputeMask(),
                          state,
                          alpha);
      update_ok = true;
    } else {
      RL_LOGE("RL->RL dual inference update failed: source_ok=%d, target_ok=%d",
              static_cast<int>(source_ok), static_cast<int>(target_ok));
      if (source_ok) {
        cmd = source_cmd;
        update_ok = true;
      }
    }
  } else {
    update_ok = controller->update(time, state, imu_state, cmd);
  }

  if (update_ok) {
    // 头部控制（头部是最后 2 个关节）
    {
      if (cmd.q.size() >= 2) {
        size_t head_start = cmd.q.size() - 2;

        // 头部始终使用 CSP 模式 + 非零 kp/kd，使头部电机主动保持位置
        cmd.modes[head_start]     = 2;   // CSP
        cmd.modes[head_start + 1] = 2;   // CSP
        cmd.kp[head_start]     = 10.0;   // head_yaw kp
        cmd.kd[head_start]     = 1.0;    // head_yaw kd
        cmd.kp[head_start + 1] = 5.0;    // head_pitch kp
        cmd.kd[head_start + 1] = 1.0;    // head_pitch kd

        // VR 头部指令透传（优先级高于手柄；超过超时未刷新则回退到手柄；
        // 模式切换先经五次多项式插值过渡，避免头部跳变）
        std::lock_guard<std::mutex> lock(head_cmd_mutex_);
        const auto head_now = std::chrono::steady_clock::now();
        const bool head_cmd_fresh =
            head_cmd_received_ && head_cmd_.q.size() >= 2 &&
            (head_now - head_cmd_rx_time_) <
                std::chrono::duration<double>(kHeadCmdTimeoutSec);

        // 目标状态：0=手柄（VR 停发/未收到），2=quest（VR 新鲜）
        const int head_want_state = head_cmd_fresh ? 2 : 0;

        float head_yaw_out = head_out_yaw_;
        float head_pitch_out = head_out_pitch_;

        if (head_state == 1) {
          // ---- 中间态：五次多项式插值过渡 ----
          if (head_state_target_ != head_want_state) {
            // 方向中途反转（如插值到 quest 时 VR 又停发）：以当前位置为新起点重新插值
            head_state_target_ = head_want_state;
            head_interp_start_time_ = head_now;
            head_interp_start_yaw_ = head_out_yaw_;
            head_interp_start_pitch_ = head_out_pitch_;
            if (head_want_state == 2) {
              head_interp_target_yaw_ = head_cmd_.q[0];
              head_interp_target_pitch_ = head_cmd_.q[1];
            } else {
              std::lock_guard<std::mutex> jlock(joy_mutex_);
              head_interp_target_yaw_ = head_joy_yaw_;
              head_interp_target_pitch_ = head_joy_pitch_;
            }
          }

          double alpha =
              std::chrono::duration<double>(head_now - head_interp_start_time_).count() /
              kHeadCmdInterpDurationSec;
          if (alpha >= 1.0) {
            alpha = 1.0;
            head_state = head_state_target_;
          }
          const double qb = QuinticBlend(alpha);
          head_yaw_out =
              head_interp_start_yaw_ +
              static_cast<float>((head_interp_target_yaw_ - head_interp_start_yaw_) * qb);
          head_pitch_out =
              head_interp_start_pitch_ +
              static_cast<float>((head_interp_target_pitch_ - head_interp_start_pitch_) * qb);
        } else if (head_state == head_want_state) {
          // ---- 稳态：直接输出目标源 ----
          if (head_state == 2) {
            // quest/VR 态：透传头显指令
            head_yaw_out = head_cmd_.q[0];
            head_pitch_out = head_cmd_.q[1];
            if (head_cmd_.v.size() >= 2) {
              cmd.v[head_start] = head_cmd_.v[0];
              cmd.v[head_start + 1] = head_cmd_.v[1];
            }
            // 清零手柄累积值，避免 VR 期间残留，切回手柄时从回中开始
            std::lock_guard<std::mutex> jlock(joy_mutex_);
            head_joy_yaw_ = 0;
            head_joy_pitch_ = 0;
          } else {
            // 手柄态：RT + 右摇杆控制头部，左右 = yaw，上下 = pitch；松 RT 后平滑回正
            std::lock_guard<std::mutex> jlock(joy_mutex_);
            head_yaw_out = head_joy_yaw_;
            head_pitch_out = head_joy_pitch_;
          }
        } else {
          // ---- 检测到模式切换：进入中间态，起点=当前实际角度，终点=目标源角度 ----
          head_state = 1;
          head_state_target_ = head_want_state;
          head_interp_start_time_ = head_now;
          head_interp_start_yaw_ = head_out_yaw_;
          head_interp_start_pitch_ = head_out_pitch_;
          if (head_want_state == 2) {
            head_interp_target_yaw_ = head_cmd_.q[0];
            head_interp_target_pitch_ = head_cmd_.q[1];
          } else {
            std::lock_guard<std::mutex> jlock(joy_mutex_);
            head_interp_target_yaw_ = head_joy_yaw_;
            head_interp_target_pitch_ = head_joy_pitch_;
          }
          // 首帧输出起点（alpha=0）
          head_yaw_out = head_interp_start_yaw_;
          head_pitch_out = head_interp_start_pitch_;
        }

        cmd.q[head_start] = head_yaw_out;
        cmd.q[head_start + 1] = head_pitch_out;
        head_out_yaw_ = head_yaw_out;
        head_out_pitch_ = head_pitch_out;
      }
    }

    // 舞蹈播完自动切回稳定控制器（边沿触发：motion 刚播完的本控制周期生效）
    auto* generic_rl = dynamic_cast<GenericRLController*>(controller);
    if (generic_rl && generic_rl->isAutoSwitchBackEnabled() &&
        generic_rl->isMotionJustFinished()) {
      const std::string& target = generic_rl->getAutoSwitchBackTarget();
      if (!target.empty()) {
        RL_LOGI("Motion finished, auto switching back to stable controller: %s",
                target.c_str());
        if (requestSwitch(target, time, /*auto_start_motion=*/false)) {
          generic_rl->consumeMotionFinished();
        }
      }
    }

  }

  return cmd;
}

void ControllerManager::waitNextCycle(std::chrono::steady_clock::time_point cycle_start) {
  // 先在锁内获取当前控制器指针，然后释放锁再 sleep
  // 避免 sleep 期间持锁导致 joy 回调被阻塞
  ControllerBase* controller = nullptr;
  {
    std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);
    if (active_index_ >= 0 && active_index_ < static_cast<int>(controllers_.size())) {
      controller = controllers_[active_index_].controller.get();
    }
  }
  if (controller) {
    controller->waitNextCycle(cycle_start);
  }
}

bool ControllerManager::isRunning() const {
  return running_.load();
}


bool ControllerManager::setDefaultController(const std::string& name) {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  for (size_t i = 0; i < controllers_.size(); ++i) {
    if (controllers_[i].name == name) {
      active_index_ = static_cast<int>(i);
      RL_LOGI("Default controller overridden to: %s", name.c_str());
      return true;
    }
  }
  RL_LOGW("setDefaultController: controller '%s' not found", name.c_str());
  return false;
}

bool ControllerManager::applyStartupPosePolicy() {
  ImuData imu;
  if (!robot_data_.getImuData(imu)) {
    RL_LOGW("applyStartupPosePolicy: no IMU data, keep current default '%s'",
            getCurrentControllerName().c_str());
    return false;
  }

  const bool fallen = IsFallenPose(imu);
  std::string target;
  if (fallen && hasController("mimic_fall_stand")) {
    target = "mimic_fall_stand";
  } else if (hasController("amp")) {
    target = "amp";
  } else {
    RL_LOGW("applyStartupPosePolicy: fallen=%d but no suitable controller "
            "(need mimic_fall_stand or amp)",
            fallen ? 1 : 0);
    return false;
  }

  const std::string prev = getCurrentControllerName();
  if (prev == target) {
    RL_LOGI("applyStartupPosePolicy: pose=%s, keep controller '%s'",
            fallen ? "fallen" : "upright", target.c_str());
    return true;
  }

  ControllerBase* old_ctrl = getCurrentController();
  if (!setDefaultController(target)) {
    return false;
  }
  // Start() 可能已 resume 旧默认控制器；切换后 pause 旧的、resume 新的
  if (old_ctrl) {
    old_ctrl->pause();
  }
  if (ControllerBase* cur = getCurrentController()) {
    cur->resume();
  }
  RL_LOGI("applyStartupPosePolicy: pose=%s → controller '%s' (was '%s')",
          fallen ? "fallen" : "upright", target.c_str(), prev.c_str());
  return true;
}

std::string ControllerManager::getCurrentControllerName() const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  if (active_index_ >= 0 && active_index_ < static_cast<int>(controllers_.size())) {
    return controllers_[active_index_].name;
  }
  return "";
}

std::vector<std::string> ControllerManager::getControllerNames() const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  std::vector<std::string> names;
  names.reserve(controllers_.size());
  for (const auto& entry : controllers_) {
    names.push_back(entry.name);
  }
  return names;
}

bool ControllerManager::getCurrentControllerMusic(std::string& music, double& music_delay) const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  if (active_index_ < 0 || active_index_ >= static_cast<int>(controllers_.size())) {
    return false;
  }
  const auto& entry = controllers_[active_index_];
  if (entry.music.empty()) {
    return false;
  }
  music = entry.music;
  music_delay = entry.music_delay;
  return true;
}

bool ControllerManager::getControllerMusic(const std::string& name,
                                           std::string& music, double& music_delay) const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  for (const auto& entry : controllers_) {
    if (entry.name == name) {
      if (entry.music.empty()) {
        return false;
      }
      music = entry.music;
      music_delay = entry.music_delay;
      return true;
    }
  }
  return false;
}

void ControllerManager::setControllerMusic(const std::string& name, const std::string& music,
                                            double music_delay) {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  for (auto& entry : controllers_) {
    if (entry.name == name) {
      entry.music = music;
      entry.music_delay = music_delay;
      return;
    }
  }
}

ControllerBase* ControllerManager::getCurrentController() const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  if (active_index_ >= 0 && active_index_ < static_cast<int>(controllers_.size())) {
    return controllers_[active_index_].controller.get();
  }
  return nullptr;
}

ControllerBase* ControllerManager::getLastController() const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  if (last_index_ >= 0 && last_index_ < static_cast<int>(controllers_.size())) {
    return controllers_[last_index_].controller.get();
  }
  return nullptr;
}

bool ControllerManager::loadControllersFromConfig(const std::string& config_file, const std::string& urdf_path) {
  RL_LOGI("Loading config from: %s", config_file.c_str());

  // 检查文件是否存在
  if (!std::filesystem::exists(config_file)) {
    RL_LOG_FAILURE("Config file not found: %s", config_file.c_str());
    return false;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file);
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("Failed to parse YAML file: %s\n  Error: %s",
                   config_file.c_str(), e.what());
    return false;
  }

  try {
    config_dir_ = std::filesystem::path(config_file).parent_path().string();

    // 从 SDK 获取机器人关节名称（硬件权威数据源）
    // 解耦：ControllerManager 负责获取，通过 setPartJointNames 传递给 Controller
    auto& robot_api = GlobalRobot::getInstance();
    std::vector<std::string> robot_arm_joints = robot_api.getArmJointNames();
    std::vector<std::string> robot_waist_joints = robot_api.getWaistJointNames();

    // 加载切换插值配置（kp/kd）
    loadSwitchInterpolationConfig(config);

    // 读取默认控制器名称
    std::string default_controller;
    if (config["default_controller"]) {
      default_controller = config["default_controller"].as<std::string>();
    }

    // 读取控制器列表
    if (!config["controllers"]) {
      RL_LOGW("No controllers defined in config");
      return true;
    }

    YAML::Node controllers_node = config["controllers"];
    for (const auto& ctrl_node : controllers_node) {
      std::string name = ctrl_node["name"].as<std::string>();
      std::string type = ctrl_node["type"].as<std::string>();
      std::string ctrl_config = ctrl_node["config"].as<std::string>();

      // 检查是否启用
      bool enabled = true;
      if (ctrl_node["enabled"]) {
        enabled = ctrl_node["enabled"].as<bool>();
      }
      if (!enabled) {
        RL_LOGI("Controller disabled: %s", name.c_str());
        continue;
      }

      // 构建控制器配置文件的完整路径（支持 URI 和绝对/相对路径）
      std::string ctrl_config_path;
      try {
        ctrl_config_path = UriPathResolver::resolve(ctrl_config, config_dir_);
      } catch (const UriResolveError& e) {
        RL_LOG_FAILURE("Failed to resolve config path '%s': %s",
                       ctrl_config.c_str(), e.what());
        continue;
      }

      // 根据类型创建控制器（使用 ControllerRegistry）
      auto controller = ControllerRegistry::create(type, RobotVersion::from_env(), name);
      if (!controller) {
        RL_LOGW("Unknown controller type: %s", type.c_str());
        continue;
      }

      // GenericRLController 需要 URDF 路径用于手臂重力补偿
      if (auto* generic_ctrl = dynamic_cast<GenericRLController*>(controller.get())) {
        generic_ctrl->setUrdfPath(urdf_path);
      }

      // 设置部位关节名称（从 SDK 获取，解耦 ControllerBase 和 SDK）
      controller->setPartJointNames(robot_arm_joints, robot_waist_joints);

      // 设置配置路径（支持配置的控制器会返回 true）
      controller->setConfigPath(ctrl_config_path);

      // 初始化控制器
      if (!controller->initialize()) {
        RL_LOG_FAILURE("Failed to initialize controller '%s' (config: %s)",
                       name.c_str(), ctrl_config_path.c_str());
        continue;
      }

      // 添加到管理器
      addController(name, std::move(controller), ctrl_config_path);

      // 配套配乐（可选）：舞蹈类控制器各自配置，供共享的"启动"按键播放时回退查询
      if (ctrl_node["music"]) {
        std::string music = ctrl_node["music"].as<std::string>();
        double music_delay = ctrl_node["music_delay"] ? ctrl_node["music_delay"].as<double>() : 0.0;
        setControllerMusic(name, music, music_delay);
      }
    }

    // 设置默认控制器
    if (!default_controller.empty() && hasController(default_controller)) {
      for (size_t i = 0; i < controllers_.size(); ++i) {
        if (controllers_[i].name == default_controller) {
          active_index_ = static_cast<int>(i);
          RL_LOGI("Default controller set to: %s", default_controller.c_str());
          break;
        }
      }
    }

    return true;

  } catch (const std::exception& e) {
    RL_LOG_FAILURE("Failed to load config: %s", e.what());
    return false;
  }
}

bool ControllerManager::reloadControllersFromConfig() {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  if (config_file_.empty()) {
    RL_LOGW("reloadControllersFromConfig: no config_file_ cached, skip");
    return false;
  }

  RL_LOGI("Reloading controllers from: %s", config_file_.c_str());

  if (!std::filesystem::exists(config_file_)) {
    RL_LOGW("Reload: config file not found: %s", config_file_.c_str());
    return false;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file_);
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("Reload: failed to parse YAML: %s", e.what());
    return false;
  }

  try {
    auto& robot_api = GlobalRobot::getInstance();
    std::vector<std::string> robot_arm_joints = robot_api.getArmJointNames();
    std::vector<std::string> robot_waist_joints = robot_api.getWaistJointNames();

    if (!config["controllers"]) {
      RL_LOGI("Reload: no controllers section, skip");
      return true;
    }

    size_t added = 0;
    size_t skipped = 0;
    for (const auto& ctrl_node : config["controllers"]) {
      std::string name = ctrl_node["name"].as<std::string>();

      // 已存在的控制器: 不重新创建/初始化 (避免打断正在运行的控制器), 但同步其
      // 配乐字段 (music/music_delay)——配乐是独立于 controller 对象的小状态,
      // 更新它不影响控制器运行. 这样上位机改 controller_manager.yaml 的 music 后,
      // 热重载也能让已存在舞蹈的配乐生效.
      if (hasController(name)) {
        if (ctrl_node["music"]) {
          std::string music = ctrl_node["music"].as<std::string>();
          double music_delay = ctrl_node["music_delay"] ? ctrl_node["music_delay"].as<double>() : 0.0;
          setControllerMusic(name, music, music_delay);
          RL_LOGI("Reload: controller '%s' exists, updated music='%s' (delay=%.2f)",
                  name.c_str(), music.c_str(), music_delay);
        }
        RL_LOGI("Reload: controller '%s' already exists, skip re-init", name.c_str());
        skipped++;
        continue;
      }

      bool enabled = true;
      if (ctrl_node["enabled"]) {
        enabled = ctrl_node["enabled"].as<bool>();
      }
      if (!enabled) {
        RL_LOGI("Reload: controller '%s' disabled, skip", name.c_str());
        skipped++;
        continue;
      }

      std::string type = ctrl_node["type"].as<std::string>();
      std::string ctrl_config = ctrl_node["config"].as<std::string>();

      std::string ctrl_config_path;
      try {
        ctrl_config_path = UriPathResolver::resolve(ctrl_config, config_dir_);
      } catch (const UriResolveError& e) {
        RL_LOG_FAILURE("Reload: failed to resolve config path '%s': %s",
                       ctrl_config.c_str(), e.what());
        continue;
      }

      auto controller = ControllerRegistry::create(type, RobotVersion::from_env(), name);
      if (!controller) {
        RL_LOGW("Reload: unknown controller type '%s' for '%s'", type.c_str(), name.c_str());
        continue;
      }

      if (auto* generic_ctrl = dynamic_cast<GenericRLController*>(controller.get())) {
        generic_ctrl->setUrdfPath(urdf_path_);
      }

      controller->setPartJointNames(robot_arm_joints, robot_waist_joints);
      controller->setConfigPath(ctrl_config_path);

      if (!controller->initialize()) {
        RL_LOG_FAILURE("Reload: failed to initialize controller '%s'", name.c_str());
        continue;
      }

      addController(name, std::move(controller), ctrl_config_path);

      if (ctrl_node["music"]) {
        std::string music = ctrl_node["music"].as<std::string>();
        double music_delay = ctrl_node["music_delay"] ? ctrl_node["music_delay"].as<double>() : 0.0;
        setControllerMusic(name, music, music_delay);
      }

      RL_LOGI("Reload: added controller '%s'", name.c_str());
      added++;
    }

    RL_LOGI("Reload complete: %zu added, %zu skipped", added, skipped);
    return true;

  } catch (const std::exception& e) {
    RL_LOG_FAILURE("Reload: failed: %s", e.what());
    return false;
  }
}

void ControllerManager::moveToDefaultPos(double elapse) {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  // 保留旧实现的语义：管理器已停止时不启动阻塞式姿态过渡。
  if (!running_.load(std::memory_order_relaxed)) {
    RL_LOGW("ControllerManager is stopped, cannot move to default position");
    return;
  }

  if (active_index_ < 0 || active_index_ >= static_cast<int>(controllers_.size())) {
    RL_LOGW("No active controller, cannot move to default position");
    return;
  }

  auto* controller = controllers_[active_index_].controller.get();
  if (!controller) {
    RL_LOGW("No active controller, cannot move to default position");
    return;
  }

  RobotState state;
  if (!robot_data_.getRobotState(state)) {
    RL_LOGE("Failed to get robot state");
    return;
  }

  // 必须使用控制器自己的虚拟 moveToDefaultPos：它按 joint_names 建立
  // 策略关节到 SDK 电机索引映射。这里不能按 default_pos 的前 N 项直接
  // 填整机数组，否则 waist/头部等非策略关节会收到错误目标并撞限位。
  controller->moveToDefaultPos(state, elapse);
}

// ============================================================================
// 切换插值配置加载
// ============================================================================

void ControllerManager::loadSwitchInterpolationConfig(const YAML::Node& config) {
  // 注意：所有配置必须从YAML加载，代码中不提供默认值
  // 直接加载到 transition_ 中，避免后续复制

  if (config["switch_interpolation"]) {
    const auto& switch_config = config["switch_interpolation"];

    if (switch_config["duration"]) {
      transition_.duration = switch_config["duration"].as<double>();
    }

    RL_LOGI("Loaded switch_interpolation config:");
    RL_LOGI("  duration: %.3f s", transition_.duration);
  } else {
    RL_LOGW("No switch_interpolation config found, using built-in transition defaults");
  }
}

// ============================================================================
// 外部控制接口（由 ControlLogic 调用）
// ============================================================================

void ControllerManager::setArmTarget(const vr::JointTrajectoryPoint& cmd) {
  if (isTransitioning()) {
    RL_LOGW("ControllerManager: Ignoring arm target during controller transition");
    return;
  }

  auto* controller = getCurrentController();
  if (!controller) return;

  auto* arm_ctrl = controller->getArmController();
  if (!arm_ctrl) return;

  // 将 std::vector 转换为 Eigen::VectorXd
  Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(cmd.q.data(), cmd.q.size());
  Eigen::VectorXd v;
  if (!cmd.v.empty()) {
    v = Eigen::Map<const Eigen::VectorXd>(cmd.v.data(), cmd.v.size());
  }

  arm_ctrl->setExternalTarget(q, v);
}

void ControllerManager::setWaistTarget(const vr::JointTrajectoryPoint& cmd) {
  if (isTransitioning()) {
    RL_LOGW("ControllerManager: Ignoring waist target during controller transition");
    return;
  }

  auto* controller = getCurrentController();
  if (!controller) return;

  auto* waist_ctrl = controller->getWaistController();
  if (!waist_ctrl) return;

  Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(cmd.q.data(), cmd.q.size());
  waist_ctrl->setExternalTarget(q);
}

void ControllerManager::setHeadTarget(const vr::JointTrajectoryPoint& cmd) {
  // 头部有 2 个关节：head_yaw, head_pitch
  constexpr size_t kHeadJointCount = 2;
  if (cmd.q.size() != kHeadJointCount) {
    RL_LOG_WARNING("Head external target dimension mismatch: expected %d, got %d",
                   static_cast<int>(kHeadJointCount), static_cast<int>(cmd.q.size()));
    return;
  }

  // 头部指令直接缓存，由 update() 时透传到 RobotCmd
  std::lock_guard<std::mutex> lock(head_cmd_mutex_);
  head_cmd_ = cmd;
  head_cmd_received_ = true;
  head_cmd_rx_time_ = std::chrono::steady_clock::now();
}

void ControllerManager::setVelocityCommand(const VelocityCommand& cmd) {
  if (isTransitioning()) {
    // RL_LOGW("ControllerManager: Ignoring velocity command during controller transition");
    return;
  }

  auto* controller = getCurrentController();
  if (!controller) return;

  controller->setVelocityCommand(cmd);
}

void ControllerManager::clearVelocityOnRunningEntry() {
  auto* controller = getCurrentController();
  if (!controller) {
    return;
  }

  VelocityCommand zero_cmd;
  zero_cmd.setZero();
  controller->setVelocityCommand(zero_cmd);
  controller->clearVelocityFilterState();

  // 清除 M1/M2 组合键速度屏蔽标记（Running 入口复位）
  if (auto* generic_rl = dynamic_cast<GenericRLController*>(controller)) {
    generic_rl->setBlockVelocityInMotion(false);
  }
}

bool ControllerManager::setCmdStanceMode(int mode, std::string& message) {
  if (isTransitioning()) {
    message = "Controller is transitioning, cannot set cmd_stance";
    RL_LOGW("ControllerManager: Ignoring cmd_stance set during controller transition");
    return false;
  }

  auto* controller = getCurrentController();
  if (!controller) {
    message = "No active controller";
    return false;
  }

  const int stance = (mode != 0) ? 1 : 0;
  if (stance == 0 && controller->isDeepSquatGuardActive()) {
    message =
        "Squat posture defense active, cannot exit posture control mode";
    RL_LOGW("ControllerManager: %s", message.c_str());
    return false;
  }
  controller->setCmdStanceMode(stance);
  message = "cmd_stance set to " + std::to_string(stance);
  return true;
}

bool ControllerManager::isDeepSquatGuardActive() const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);
  if (active_index_ < 0 ||
      active_index_ >= static_cast<int>(controllers_.size())) {
    return false;
  }
  return controllers_[active_index_].controller->isDeepSquatGuardActive();
}

int ControllerManager::getCurrentCmdStanceMode() const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);
  if (active_index_ < 0 ||
      active_index_ >= static_cast<int>(controllers_.size())) {
    return 0;
  }
  return controllers_[active_index_].controller->getCmdStanceMode();
}

bool ControllerManager::hasResidualStanceHeightCommand() const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);
  if (active_index_ < 0 ||
      active_index_ >= static_cast<int>(controllers_.size())) {
    return false;
  }
  return controllers_[active_index_].controller->hasResidualStanceHeightCommand();
}

bool ControllerManager::canTurnExitDeepSquatGuard(double pending_angular_z) const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);
  if (active_index_ < 0 ||
      active_index_ >= static_cast<int>(controllers_.size())) {
    return false;
  }
  return controllers_[active_index_].controller->canTurnExitDeepSquatGuard(
      pending_angular_z);
}

double ControllerManager::getSmoothedWalkingCmdZ(double pending_angular_z) const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);
  if (active_index_ < 0 ||
      active_index_ >= static_cast<int>(controllers_.size())) {
    return pending_angular_z;
  }
  return controllers_[active_index_].controller->getSmoothedWalkingCmdZ(
      pending_angular_z);
}

double ControllerManager::getTurnExitGuardThreshold() const {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);
  if (active_index_ < 0 ||
      active_index_ >= static_cast<int>(controllers_.size())) {
    return 0.5;
  }
  return controllers_[active_index_].controller->getTurnExitGuardThreshold();
}

bool ControllerManager::setArmMode(ArmControlMode mode, std::string& message) {
  if (isTransitioning()) {
    message = "Controller is transitioning, cannot set arm mode";
    RL_LOGW("ControllerManager: Ignoring arm mode change during controller transition");
    return false;
  }

  auto* controller = getCurrentController();
  if (!controller) {
    message = "No active controller";
    return false;
  }

  auto* arm_ctrl = controller->getArmController();
  if (!arm_ctrl) {
    message = "Arm controller not available";
    return false;
  }

  arm_ctrl->setMode(mode);
  message = "Arm mode set to " + std::to_string(static_cast<int>(mode));
  RL_LOG_SUCCESS("ControllerManager: Arm mode set to %d", static_cast<int>(mode));

  // keep_pose 时通知 TactPlayer 冻结（LB+B 锁定手臂）
  if (mode == ArmControlMode::kKeepPose && arm_freeze_callback_) {
    arm_freeze_callback_();
  }

  return true;
}

bool ControllerManager::setWaistMode(WaistControlMode mode, std::string& message) {
  if (isTransitioning()) {
    message = "Controller is transitioning, cannot set waist mode";
    RL_LOGW("ControllerManager: Ignoring waist mode change during controller transition");
    return false;
  }

  auto* controller = getCurrentController();
  if (!controller) {
    message = "No active controller";
    return false;
  }

  auto* waist_ctrl = controller->getWaistController();
  if (!waist_ctrl) {
    message = "Waist controller not available";
    return false;
  }

  waist_ctrl->setMode(mode);
  message = "Waist mode set to " + std::to_string(static_cast<int>(mode));
  RL_LOG_SUCCESS("Waist mode set to %d", static_cast<int>(mode));
  return true;
}

std::optional<ArmControlMode> ControllerManager::getCurrentArmMode() const {
  auto* controller = getCurrentController();
  if (!controller) {
    return std::nullopt;
  }
  auto* arm_ctrl = controller->getArmController();
  if (!arm_ctrl) {
    return std::nullopt;
  }
  return arm_ctrl->getMode();
}

std::optional<WaistControlMode> ControllerManager::getCurrentWaistMode() const {
  auto* controller = getCurrentController();
  if (!controller) {
    return std::nullopt;
  }
  auto* waist_ctrl = controller->getWaistController();
  if (!waist_ctrl) {
    return std::nullopt;
  }
  return waist_ctrl->getMode();
}

bool ControllerManager::isCurrentMotionPlaying() const {
  auto* controller = getCurrentController();
  auto* generic_rl = dynamic_cast<GenericRLController*>(controller);
  if (!generic_rl) {
    return false;
  }
  return generic_rl->isMotionPlaying();
}

std::string ControllerManager::getCurrentMotionName() const {
  auto* controller = getCurrentController();
  auto* generic_rl = dynamic_cast<GenericRLController*>(controller);
  if (!generic_rl) {
    return "";
  }
  return generic_rl->getCurrentMotionName();
}

std::vector<std::string> ControllerManager::getAvailableMotionNames() const {
  auto* controller = getCurrentController();
  auto* generic_rl = dynamic_cast<GenericRLController*>(controller);
  if (!generic_rl) {
    return {};
  }
  return generic_rl->getMotionNames();
}

bool ControllerManager::startMotion(const std::string& name) {
  auto* controller = getCurrentController();
  if (!controller) {
    RL_LOGW("ControllerManager::startMotion: No active controller");
    return false;
  }

  // 起播前同步读取最新 IMU：倒地起身在此刻按肚皮法线选择 prone/supine。
  auto* generic_rl = dynamic_cast<GenericRLController*>(controller);
  if (generic_rl) {
    ImuData imu;
    if (robot_data_.getImuData(imu)) {
      generic_rl->setMotionStartImu(imu);
    } else {
      RL_LOGW("ControllerManager::startMotion: latest IMU unavailable; using cached sample");
    }
  }

  bool success = false;
  if (!name.empty()) {
    // 尝试 dynamic_cast 到 GenericRLController 调用带 name 版本
    if (generic_rl) {
      success = generic_rl->startMotion(name);
    } else {
      RL_LOGW("ControllerManager::startMotion: Controller '%s' does not support named motion",
              getCurrentControllerName().c_str());
      return false;
    }
  } else {
    success = controller->startMotion();
  }

  // CSV motion 未找到，尝试 tact fallback（由 ExternalInterface 提供 TactPlayer）
  if (!success && tact_fallback_) {
    RL_LOGI("ControllerManager::startMotion: CSV motion '%s' not found, trying tact fallback",
            name.c_str());
    success = tact_fallback_(name);
  }

  if (success) {
    if (name.empty()) {
      RL_LOGI("ControllerManager::startMotion: Motion started on controller '%s'",
              getCurrentControllerName().c_str());
    } else {
      RL_LOGI("ControllerManager::startMotion: Motion '%s' started on controller '%s'",
              name.c_str(), getCurrentControllerName().c_str());
    }
  } else {
    RL_LOGW("ControllerManager::startMotion: Controller '%s' failed to start motion",
            getCurrentControllerName().c_str());
  }
  return success;
}

bool ControllerManager::prepareToMotionStart(double max_joint_velocity) {
  auto* generic_rl = dynamic_cast<GenericRLController*>(getCurrentController());
  if (!generic_rl) {
    RL_LOGW("ControllerManager::prepareToMotionStart: active controller is not GenericRL");
    return false;
  }
  RobotState state;
  if (!robot_data_.getRobotState(state)) {
    return false;
  }
  ImuData imu;
  if (!robot_data_.getImuData(imu)) {
    return false;
  }
  return generic_rl->prepareToMotionStart(state, imu, max_joint_velocity);
}

bool ControllerManager::enterStandbyHold() {
  auto* generic_rl = dynamic_cast<GenericRLController*>(getCurrentController());
  if (!generic_rl) {
    RL_LOGW("ControllerManager::enterStandbyHold: active controller is not GenericRL");
    return false;
  }
  RobotState state;
  if (!robot_data_.getRobotState(state)) {
    return false;
  }
  return generic_rl->enterStandbyHold(state);
}

bool ControllerManager::isPreparingMotionStart() const {
  auto* generic_rl = dynamic_cast<const GenericRLController*>(getCurrentController());
  return generic_rl && generic_rl->isPreparingMotionStart();
}

bool ControllerManager::isHoldingMotionStart() const {
  auto* generic_rl = dynamic_cast<const GenericRLController*>(getCurrentController());
  return generic_rl && generic_rl->isHoldingMotionStart();
}

bool ControllerManager::isStandbyHolding() const {
  auto* generic_rl = dynamic_cast<const GenericRLController*>(getCurrentController());
  return generic_rl && generic_rl->isStandbyHolding();
}

}  // namespace leju
