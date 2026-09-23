#include "leju-rl-controller/controllers/generic_rl_controller.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <thread>

#include "leju-rl-controller/controllers/controller_registry.h"
#include "leju-rl-controller/inference/model_factory.h"
#include "leju-rl-controller/rl_log.h"
#include "leju-rl-controller/utils/uri_path_resolver.h"
#include "lejusdk-utils/cpu_affinity.hpp"
#include "lejusdk-utils/time_utils.hpp"

namespace leju {
namespace {

constexpr double kFallStandFlatCos = std::cos(15.0 * M_PI / 180.0);

}  // namespace

// ============================================================================
// 构造 / 析构
// ============================================================================

GenericRLController::GenericRLController(const RobotVersion& version,
                                         const std::string& name)
    : robot_version_(version) {
  name_ = name;
}

GenericRLController::~GenericRLController() {
  stopInferenceThread();
}

// ============================================================================
// ControllerBase 生命周期
// ============================================================================

bool GenericRLController::initialize() {
  RL_LOG_INFO("Initializing GenericRLController...");

  try {
    if (config_path_.empty()) {
      RL_LOG_FAILURE("Config path not set");
      return false;
    }

    // 1. 加载 YAML 配置（关节、观测、motion 等）
    if (!loadConfig(config_path_)) {
      RL_LOG_FAILURE("Failed to load config");
      return false;
    }

    // 2. 构建策略关节 → SDK 电机索引映射
    if (!buildJointMapping()) {
      RL_LOG_FAILURE("Failed to build joint mapping");
      return false;
    }

    // 2.5 初始化部位控制器（手臂、腰部）
    // 调用基类方法，会自动调用 buildPartJointMapping()
    initPartControllers();

    std::filesystem::path config_dir = std::filesystem::path(config_path_).parent_path();

    // 2.6 初始化手臂力矩补偿（URDF 路径从 main 传入）
    if (arm_controller_ && !urdf_path_.empty()) {
      int n_arm = static_cast<int>(arm_joint_names_.size());
      Eigen::VectorXd arm_direction(n_arm);
      Eigen::VectorXd arm_kp(n_arm);
      Eigen::VectorXd arm_kd(n_arm);
      for (int i = 0; i < n_arm; ++i) {
        const int policy_idx = findPolicyJointIndex(arm_joint_names_[i]);
        if (policy_idx < 0) {
          continue;
        }
        arm_direction[i] = joint_direction_[policy_idx];
        arm_kp[i] = joint_kp_[policy_idx];
        arm_kd[i] = joint_kd_[policy_idx];
      }
      if (!arm_controller_->initGravityCompensation(urdf_path_, arm_joint_names_,
                                                     arm_direction, arm_kp, arm_kd)) {
        RL_LOG_WARNING("Arm torque compensation init failed, continuing without it");
      }
    }

    // 3. 加载所有 motion 轨迹文件
    loadMotionTrajectories(config_dir.string());

    // 4. 初始化观测历史缓冲区（obs_stacks_ 和 observations_）
    initObsHistory();
    if (customObservationSize() > 0) {
      observations_ = array_t::Zero(customObservationSize());
    }

    // 5. 加载 ONNX 策略模型（支持 URI 和绝对/相对路径）
    if (dual_fall_stand_) {
      const std::string prone_full =
          UriPathResolver::resolve(prone_policy_path_, config_dir.string());
      const std::string supine_full =
          UriPathResolver::resolve(supine_policy_path_, config_dir.string());
      model_prone_ = ModelFactory::create(inference_engine_);
      model_supine_ = ModelFactory::create(inference_engine_);
      if (!model_prone_ || !model_prone_->load(prone_full) ||
          !model_supine_ || !model_supine_->load(supine_full)) {
        RL_LOG_FAILURE("Failed to load dual fall-stand policies:\n  prone=%s\n  supine=%s",
                       prone_full.c_str(), supine_full.c_str());
        return false;
      }
      RL_LOG_SUCCESS("Dual fall-stand policies loaded");
      // 默认挂 supine；prepare/start 时再按姿态切换
      if (!switchFallStandModel(FallStandModelType::kSupine)) {
        return false;
      }
    } else {
      std::string policy_full_path =
          UriPathResolver::resolve(policy_path_, config_dir.string());
      if (!loadPolicy(policy_full_path)) {
        RL_LOG_FAILURE("Failed to load policy: %s", policy_full_path.c_str());
        return false;
      }
    }

  // 6. 初始化动作向量
  actions_.resize(policy_joint_count_);
  actions_.setZero();
  last_actions_.resize(policy_joint_count_);
  last_actions_.setZero();
  last_policy_q_target_.resize(policy_joint_count_);
  last_policy_q_target_.setZero();
  pending_observation_.resize(0);
  filtered_velocity_cmd_ = array_t::Zero(3);
  angular_z_stop_filter_.reset(0.0);
  smoothed_raw_cmd_x_ = 0.0;
  smoothed_raw_cmd_y_ = 0.0;
  smoothed_raw_cmd_angz_ = 0.0;
  prev_virtual_arm_obs_filtered_cmd_x_ = 0.0;
  clearInferenceTimestamps();

    // 7. 创建 TopicLogger
    logger_ = TopicLogger::create();

    // 8. 计算 decimation（控制循环每 decimation_ 步执行一次策略推理）
    decimation_ = static_cast<int>(std::round(policy_dt_ / loop_dt_));
    startInferenceThread();

    state_ = ControllerState::kPaused;
    RL_LOG_SUCCESS("GenericRLController initialized");
    return true;
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("Exception during initialization: %s", e.what());
    return false;
  }
}


bool GenericRLController::updateImpl(double time, const RobotState& state,
                                      const ImuData& imu, RobotCmd& cmd) {
  (void)time;
  (void)imu;

  // 瘫软（对齐 kuavo FALL_DOWN 态：零力矩、关节自由）：仅 limp_when_idle_ 配置启用。
  // 未 prepare、未起播且未播完时为初始态；播完（STANDING）不瘫软，
  // 策略继续以最后一帧为参考保持站姿，等上层切走。
  if (limp_when_idle_ && motion_prep_state_ == MotionPrepState::kIdle &&
      !motion_playing_) {
    MotionTrajectory* loader = getCurrentMotion();
    const bool playback_finished = (loader != nullptr) && !loader->hasNext();
    if (!playback_finished) {
      fillLimpCmd(cmd, state);
      return true;
    }
  }

  // 两阶段准备：非阻塞插值 / 保持 CSV 首帧（不跑策略、不推进 motion）
  // StandbyHold：START 后锁当前姿态，等 LB+RB+X 再进 prepare/起播
  if (motion_prep_state_ == MotionPrepState::kInterpolating) {
    motion_prep_elapsed_ += loop_dt_;
    double phase = 1.0;
    if (motion_prep_duration_ > 0.0) {
      phase = std::clamp(motion_prep_elapsed_ / motion_prep_duration_, 0.0, 1.0);
    }
    std::vector<double> q(motor_count_, 0.0);
    for (int i = 0; i < motor_count_; ++i) {
      q[static_cast<size_t>(i)] =
          motion_prep_start_q_[static_cast<size_t>(i)] +
          phase * (motion_prep_target_q_[static_cast<size_t>(i)] -
                   motion_prep_start_q_[static_cast<size_t>(i)]);
    }
    fillHoldCmd(cmd, q);
    if (phase >= 1.0) {
      motion_prep_state_ = MotionPrepState::kHolding;
      RL_LOG_INFO("prepareToMotionStart: reached CSV first frame, holding (press again to start)");
    }
    return true;
  }
  if (motion_prep_state_ == MotionPrepState::kHolding ||
      motion_prep_state_ == MotionPrepState::kStandbyHold) {
    // Holding：CSV 首帧；StandbyHold：START 后当前姿态。均不跑策略。
    fillHoldCmd(cmd, motion_prep_target_q_);
    return true;
  }

  // 首次 update 时初始化 dummy_world_yaw_（与 rl_mimic_controller 时序一致）
  MotionTrajectory* loader = getCurrentMotion();
  if (step_count_ == 0 && loader && loader->isLoaded()) {
    RL_LOG_INFO("\033[33m[step 0] initializeDummyWorldYaw at startup\033[0m");
    initializeDummyWorldYaw();
  }

  // 按 decimation 降频执行策略推理
  if (step_count_ % decimation_ == 0) {
    computeObservation();
    submitObservationForInference(observations_);
  }

  // 每个控制周期都更新电机命令
  updateRobotCmd(cmd);

  // 推进 motion 帧
  if (step_count_ % decimation_ == 0) {
    if (loader && motion_playing_) {
      if (loader->hasNext()) {
        loader->next();
      } else {
        // motion 播放完毕，停止播放
        motion_playing_ = false;
        motion_just_finished_ = true;  // 播完边沿标志，供 ControllerManager 自动切回
        block_velocity_in_motion_ = false;
        RL_LOG_INFO("Motion '%s' playback finished", current_motion_name_.c_str());
      }
    }
  }

  return true;
}

void GenericRLController::reset() {
  // 调用基类 reset（会重置 step_count_）
  ControllerBase::reset();
  resetObsHistory();
  {
    std::lock_guard<std::mutex> lock(action_mutex_);
    actions_.setZero();
    last_actions_.setZero();
  }
  last_policy_q_target_.setZero();
  motion_playing_ = false;
  motion_just_finished_ = false;
  block_velocity_in_motion_ = false;
  motion_prep_state_ = MotionPrepState::kIdle;
  motion_prep_elapsed_ = 0.0;
  fall_stand_entry_blend_active_ = false;
  fall_stand_entry_blend_frame_ = 0;
  motion_prep_start_q_.clear();
  motion_prep_target_q_.clear();
  dummy_world_yaw_ = 0.0;
  velocity_cmd_.setZero();
  filtered_velocity_cmd_.setZero();
  angular_z_stop_filter_.reset(0.0);
  diagnostic_cmd_log_count_ = 0;
  stance_debounce_counter_ = 0;
  roll_compensation_closed_loop_initialized_ = false;
  roll_compensation_filtered_roll_rad_ = 0.0;
  roll_compensation_target_roll_rad_ = 0.0;
  roll_compensation_integral_rad_sec_ = 0.0;
  lateral_yaw_compensation_initialized_ = false;
  lateral_yaw_target_rad_ = 0.0;
  lateral_cmd_y_active_history_.clear();
  squat_posture_deep_seen_ = false;
  stand_up_rising_active_ = false;
  prev_stand_up_knee_rad_ = 0.0;
  stand_up_knee_prev_initialized_ = false;
  prev_virtual_arm_obs_filtered_cmd_x_ = 0.0;
  clearInferenceTimestamps();
  held_non_policy_pos_.clear();
  held_non_policy_pos_initialized_ = false;

  MotionTrajectory* loader = getCurrentMotion();
  if (loader) {
    loader->reset();
    RL_LOG_INFO("Available motions: %zu. Press `guide` to start '%s'",
                motions_.size(), current_motion_name_.c_str());
  }
}

void GenericRLController::clearVelocityFilterState() {
  filtered_velocity_cmd_.setZero();
  angular_z_stop_filter_.reset(0.0);
  smoothed_stance_height_cmd_ = 0.0;
  smoothed_raw_cmd_x_ = 0.0;
  smoothed_raw_cmd_y_ = 0.0;
  smoothed_raw_cmd_angz_ = 0.0;
  cmd_x_decel_in_hold_ = false;
  cmd_x_decel_hold_timer_ = 0.0;
  stance_debounce_counter_ = 0;
  low_speed_kick_remaining_steps_ = 0;
  low_speed_yaw_kick_remaining_steps_ = 0;
  low_speed_yaw_kick_sign_ = 1.0;
  yaw_cmd_angz_history_.clear();
  lateral_cmd_y_active_history_.clear();
  prev_raw_cmd_vel_line_x_ = 0.0;
  prev_raw_cmd_vel_angular_z_ = 0.0;
  prev_virtual_arm_obs_filtered_cmd_x_ = 0.0;
}

// ============================================================================
// Motion 播放控制
// ============================================================================

bool GenericRLController::startMotion() {
  // 倒地起身仅允许 READY 状态起播，避免绕过协调器从瘫软或插值中直接起身。
  if (dual_fall_stand_ && motion_prep_state_ != MotionPrepState::kHolding) {
    RL_LOG_WARNING("startMotion: fall-stand requires READY_FOR_STAND_UP");
    return false;
  }

  // 使用 ControllerManager 在起播瞬间采样的 IMU：仅躺平/趴平时放行，并据此选择策略。
  if (dual_fall_stand_ && !autoSelectFallStandModel(current_imu_)) {
    return false;
  }

  MotionTrajectory* loader = getCurrentMotion();
  if (!loader) {
    return false;
  }

  // 结束两阶段准备态，进入正式播放
  motion_prep_state_ = MotionPrepState::kIdle;
  motion_prep_elapsed_ = 0.0;

  // 如果已经在播放，返回 false 表示重复触发，避免重置导致动作跳回起始位置
  if (motion_playing_) {
    RL_LOG_WARNING("Motion '%s' already playing, skipping restart", current_motion_name_.c_str());
    return false;
  }

  // 如果播放完毕，重置到起始帧
  if (!loader->hasNext()) {
    loader->reset();
    RL_LOG_INFO("Motion '%s' reset to beginning", current_motion_name_.c_str());
  }

  if (dual_fall_stand_) {
    fall_stand_entry_blend_active_ = true;
    fall_stand_entry_blend_frame_ = 0;
    RL_LOG_INFO("Fall-stand entry blend started: READY -> advancing CSV/policy over %d frames",
                kFallStandEntryBlendFrames);
  }

  // 在开始播放时重新初始化 dummy_world_yaw_（与 humanoidController 一致）
  RL_LOG_INFO("\033[32m[startMotion] initializeDummyWorldYaw before motion play\033[0m");
  initializeDummyWorldYaw();
  motion_just_finished_ = false;
  motion_playing_ = true;
  RL_LOG_INFO("\033[32mMotion '%s' playback started (dummy_world_yaw: %.3f)\033[0m",
              current_motion_name_.c_str(), dummy_world_yaw_);
  return true;
}

bool GenericRLController::startMotion(const std::string& name) {
  if (dual_fall_stand_ && motion_prep_state_ != MotionPrepState::kHolding) {
    RL_LOG_WARNING("startMotion('%s'): fall-stand requires READY_FOR_STAND_UP", name.c_str());
    return false;
  }
  if (dual_fall_stand_ && !autoSelectFallStandModel(current_imu_)) {
    return false;
  }

  auto it = motions_.find(name);
  if (it == motions_.end()) {
    RL_LOG_WARNING("Motion '%s' not found", name.c_str());
    return false;
  }

  // 切换到新 motion
  if (current_motion_name_ != name) {
    current_motion_name_ = name;
    RL_LOG_INFO("Switched to motion: %s", name.c_str());
  }

  // 重置并开始播放
  it->second->reset();
  motion_prep_state_ = MotionPrepState::kIdle;
  motion_prep_elapsed_ = 0.0;
  // 在开始播放时重新初始化 dummy_world_yaw_（与 humanoidController 一致）
  RL_LOG_INFO("\033[32m[startMotion(%s)] initializeDummyWorldYaw before motion play\033[0m", name.c_str());
  initializeDummyWorldYaw();
  motion_just_finished_ = false;
  motion_playing_ = true;
  RL_LOG_INFO("\033[32mMotion '%s' playback started (dummy_world_yaw: %.3f)\033[0m",
              name.c_str(), dummy_world_yaw_);
  return true;
}

bool GenericRLController::stopMotion() {
  if (motion_playing_) {
    motion_playing_ = false;
    block_velocity_in_motion_ = false;
    MotionTrajectory* loader = getCurrentMotion();
    if (loader) {
      loader->reset();
    }
    motion_just_finished_ = false;
    RL_LOG_INFO("Motion playback stopped");
    return true;
  }
  return false;
}

bool GenericRLController::enterStandbyHold(const RobotState& current_state) {
  if (motion_playing_) {
    RL_LOG_WARNING("enterStandbyHold: motion already playing, skip");
    return false;
  }
  if (current_state.q.empty() ||
      static_cast<int>(current_state.q.size()) < motor_count_) {
    RL_LOG_WARNING("enterStandbyHold: no valid robot state yet");
    return false;
  }

  motion_prep_target_q_ = current_state.q;
  if (static_cast<int>(motion_prep_target_q_.size()) < motor_count_) {
    motion_prep_target_q_.resize(static_cast<size_t>(motor_count_), 0.0);
  }
  motion_prep_start_q_ = motion_prep_target_q_;
  motion_prep_elapsed_ = 0.0;
  motion_prep_duration_ = 0.0;
  motion_prep_state_ = MotionPrepState::kStandbyHold;
  RL_LOG_INFO("enterStandbyHold: locking current pose (no policy until LB+RB+X two-stage)");
  return true;
}

bool GenericRLController::prepareToMotionStart(const RobotState& current_state,
                                               const ImuData& imu,
                                               double max_joint_velocity) {
  current_imu_ = imu;

  MotionTrajectory* motion = getCurrentMotion();
  if (!motion || !motion->isLoaded()) {
    RL_LOG_WARNING("prepareToMotionStart: no loaded motion on '%s'", name_.c_str());
    return false;
  }
  if (motion_playing_) {
    RL_LOG_WARNING("prepareToMotionStart: motion already playing, skip");
    return false;
  }
  // 倒地起身只允许从 FALL_DOWN 进入 PREPARE；READY、插值中和起身完成均拒绝。
  if (dual_fall_stand_ &&
      (motion_prep_state_ != MotionPrepState::kIdle || !motion->hasNext())) {
    RL_LOG_WARNING("prepareToMotionStart: fall-stand requires FALL_DOWN");
    return false;
  }
  if (current_state.q.empty() ||
      static_cast<int>(current_state.q.size()) < motor_count_) {
    RL_LOG_WARNING("prepareToMotionStart: no valid robot state yet");
    return false;
  }

  motion->reset();
  array_t ready_first_frame;
  if (!dual_fall_stand_) {
    ready_first_frame = motion->getFirstFrameJointPos();
    if (ready_first_frame.size() == 0) {
      RL_LOG_WARNING("prepareToMotionStart: empty first frame");
      return false;
    }
  }

  motion_prep_start_q_ = current_state.q;
  if (static_cast<int>(motion_prep_start_q_.size()) < motor_count_) {
    motion_prep_start_q_.resize(static_cast<size_t>(motor_count_), 0.0);
  }
  motion_prep_target_q_ = motion_prep_start_q_;
  if (dual_fall_stand_) {
    // READY 是与 prone/supine 无关的固定准备姿态；CSV 仅在 startMotion() 后起播。
    for (int i = 0; i < policy_joint_count_; ++i) {
      const int motor_idx = policy_joint_ids_[i];
      if (motor_idx >= 0 && motor_idx < motor_count_) {
        motion_prep_target_q_[static_cast<size_t>(motor_idx)] = 0.0;
      }
    }

    auto set_ready_joint_target = [this](const char* joint_name, double target_rad) {
      const int policy_idx = findPolicyJointIndex(joint_name);
      if (policy_idx < 0 || policy_idx >= policy_joint_count_) {
        RL_LOG_WARNING("prepareToMotionStart: READY joint '%s' is not in policy", joint_name);
        return;
      }
      const int motor_idx = policy_joint_ids_[policy_idx];
      if (motor_idx < 0 || motor_idx >= motor_count_) {
        RL_LOG_WARNING("prepareToMotionStart: READY joint '%s' has invalid motor index %d",
                       joint_name, motor_idx);
        return;
      }
      motion_prep_target_q_[static_cast<size_t>(motor_idx)] =
          joint_direction_[policy_idx] * target_rad;
    };

    constexpr double kDegToRad = M_PI / 180.0;
    // 实际 MJCF 的 leg_*3_link 在世界系前摆 15°的完整 IK 解。
    set_ready_joint_target("leg_l1_joint", -20.753571 * kDegToRad);
    set_ready_joint_target("leg_l2_joint", -1.920483 * kDegToRad);
    set_ready_joint_target("leg_l3_joint", -14.510819 * kDegToRad);
    set_ready_joint_target("leg_r1_joint", 20.753571 * kDegToRad);
    set_ready_joint_target("leg_r2_joint", 1.920483 * kDegToRad);
    set_ready_joint_target("leg_r3_joint", 14.510819 * kDegToRad);
    set_ready_joint_target("leg_l4_joint", 30.0 * kDegToRad);
    set_ready_joint_target("leg_r4_joint", 30.0 * kDegToRad);
    set_ready_joint_target("leg_l5_joint", -15.0 * kDegToRad);
    set_ready_joint_target("leg_r5_joint", -15.0 * kDegToRad);
    // 按 prepare 时刻实时 IMU 的肚皮法线调整肩膀后摆：趴加 15°，躺减 15°(归零)，其它不变。
    // 与 autoSelectFallStandModel 同一判据(body_x·gravity)，±0.5 死区区分侧躺/斜置。
    const Eigen::Quaterniond prep_quat(imu.quat[0], imu.quat[1], imu.quat[2], imu.quat[3]);
    const Eigen::Vector3d prep_body_x = prep_quat.toRotationMatrix().col(0);
    const double prep_cos = prep_body_x.dot(Eigen::Vector3d(0.0, 0.0, -1.0));
    double shoulder_deg = 15.0;
    const char* pose_tag = "OTHER";
    if (prep_cos > 0.5) {
      shoulder_deg = 30.0;
      pose_tag = "PRONE";
    } else if (prep_cos < -0.5) {
      shoulder_deg = 0.0;
      pose_tag = "SUPINE";
    }
    set_ready_joint_target("zarm_l1_joint", shoulder_deg * kDegToRad);
    set_ready_joint_target("zarm_r1_joint", shoulder_deg * kDegToRad);
    set_ready_joint_target("zarm_l4_joint", -30.0 * kDegToRad);
    set_ready_joint_target("zarm_r4_joint", -30.0 * kDegToRad);
    RL_LOG_INFO(
        "prepareToMotionStart: using fixed fall-stand READY posture "
        "(pose=%s, body_x·gravity=%.3f, shoulder=%.1f deg)",
        pose_tag, prep_cos, shoulder_deg);
  } else {
    // 普通 motion 的 READY 目标仍为 CSV 首帧。
    for (int i = 0; i < policy_joint_count_; ++i) {
      const int motor_idx = policy_joint_ids_[i];
      if (motor_idx >= 0 && motor_idx < motor_count_ && i < ready_first_frame.size()) {
        motion_prep_target_q_[static_cast<size_t>(motor_idx)] =
            joint_direction_[i] * ready_first_frame[i];
      }
    }
  }
  {
    auto& robot = GlobalRobot::getInstance();
    const auto head_names = robot.getHeadJointNames();
    for (const auto& head_name : head_names) {
      for (int j = 0; j < motor_count_; ++j) {
        if (j < static_cast<int>(motor_names_.size()) && motor_names_[j] == head_name) {
          motion_prep_target_q_[static_cast<size_t>(j)] = 0.0;
          break;
        }
      }
    }
  }

  // 对齐 kuavo FallStand：duration = max|Δq| / v_max
  const double v_max = (max_joint_velocity > 0.0) ? max_joint_velocity
                                                  : motion_prep_max_joint_velocity_;
  double max_delta = 0.0;
  for (int i = 0; i < motor_count_; ++i) {
    const double delta = std::abs(motion_prep_target_q_[static_cast<size_t>(i)] -
                                  motion_prep_start_q_[static_cast<size_t>(i)]);
    if (delta > max_delta) {
      max_delta = delta;
    }
  }
  if (v_max > 0.0 && max_delta > 1e-6) {
    motion_prep_duration_ = max_delta / v_max;
  } else {
    motion_prep_duration_ = 0.0;
  }

  motion_prep_elapsed_ = 0.0;
  motion_prep_state_ = MotionPrepState::kInterpolating;
  RL_LOG_INFO(
      "prepareToMotionStart: interpolating to %s over %.3fs "
      "(max_delta=%.4f rad, v_max=%.3f rad/s)",
      dual_fall_stand_ ? "fixed fall-stand READY posture" : "CSV first frame",
      motion_prep_duration_, max_delta, v_max);
  return true;
}

void GenericRLController::fillHoldCmd(RobotCmd& cmd,
                                      const std::vector<double>& q_target) const {
  if (cmd.q.size() != static_cast<size_t>(motor_count_)) {
    cmd.resize(static_cast<size_t>(motor_count_));
  }
  for (int i = 0; i < motor_count_; ++i) {
    cmd.q[i] = (i < static_cast<int>(q_target.size())) ? q_target[static_cast<size_t>(i)] : 0.0;
    cmd.v[i] = 0.0;
    cmd.tau[i] = 0.0;
    cmd.kp[i] = 100.0;
    cmd.kd[i] = 10.0;
    cmd.modes[i] = 2;
  }
  for (int i = 0; i < policy_joint_count_; ++i) {
    int motor_idx = policy_joint_ids_[i];
    if (motor_idx >= 0 && motor_idx < motor_count_) {
      cmd.kp[motor_idx] = joint_kp_[i];
      cmd.kd[motor_idx] = joint_kd_[i];
    }
  }
  cmd.timestamp = leju::common::GetUnixTimestampS();
}

void GenericRLController::fillLimpCmd(RobotCmd& cmd, const RobotState& state) const {
  if (cmd.q.size() != static_cast<size_t>(motor_count_)) {
    cmd.resize(static_cast<size_t>(motor_count_));
  }
  for (int i = 0; i < motor_count_; ++i) {
    // q 填当前反馈位仅作兜底：CST + kp/kd/tau 全零时电机零力矩，q 不被消费
    cmd.q[i] = (i < static_cast<int>(state.q.size())) ? state.q[static_cast<size_t>(i)] : 0.0;
    cmd.v[i] = 0.0;
    cmd.tau[i] = 0.0;
    cmd.kp[i] = 0.0;
    cmd.kd[i] = 0.0;
    cmd.modes[i] = static_cast<uint8_t>(MotorControlMode::CST);
  }
  cmd.timestamp = leju::common::GetUnixTimestampS();
}

int GenericRLController::getFallStandState() const {
  // 对齐 kuavo FallStandController::FallStandState
  if (motion_prep_state_ == MotionPrepState::kInterpolating) {
    return 1;  // INTERPOLATING：prepare 插值到 READY 目标中
  }
  if (motion_prep_state_ == MotionPrepState::kHolding) {
    return 2;  // READY_FOR_STAND_UP：READY 目标保持，等待 STAND_UP
  }
  if (motion_playing_) {
    return 3;  // STAND_UP：起身轨迹播放中
  }
  MotionTrajectory* loader = getCurrentMotion();
  if (loader && !loader->hasNext()) {
    return 4;  // STANDING：轨迹播完（对齐 kuavo 轨迹结束进 STANDING）
  }
  return 0;  // FALL_DOWN：kIdle 零力矩瘫软
}

bool GenericRLController::switchFallStandModel(FallStandModelType model_type) {
  if (!dual_fall_stand_) {
    return true;
  }
  if (fall_stand_motion_base_.empty()) {
    RL_LOG_WARNING("switchFallStandModel: dual mode without motion base name");
    return false;
  }

  const char* tag = (model_type == FallStandModelType::kProne) ? "prone" : "supine";
  const std::string motion_key = fall_stand_motion_base_ + "_" + tag;
  if (model_type == fall_stand_model_type_ && current_motion_name_ == motion_key) {
    return true;
  }

  auto it = motions_.find(motion_key);
  if (it == motions_.end() || !it->second || !it->second->isLoaded()) {
    RL_LOG_WARNING("switchFallStandModel: motion '%s' not loaded", motion_key.c_str());
    return false;
  }
  InferenceModel* policy =
      (model_type == FallStandModelType::kProne) ? model_prone_.get() : model_supine_.get();
  if (!policy || !policy->isLoaded()) {
    RL_LOG_WARNING("switchFallStandModel: %s policy not loaded", tag);
    return false;
  }

  {
    // 与推理线程互斥，避免切换中途 forward
    std::lock_guard<std::mutex> lock(inference_mutex_);
    fall_stand_model_type_ = model_type;
    current_motion_name_ = motion_key;
    it->second->reset();
  }

  RL_LOG_INFO("Fall-stand model switched to %s (motion=%s)", tag, motion_key.c_str());
  return true;
}

bool GenericRLController::autoSelectFallStandModel(const ImuData& imu) {
  if (!dual_fall_stand_) {
    return true;
  }

  Eigen::Quaterniond quat(imu.quat[0], imu.quat[1], imu.quat[2], imu.quat[3]);
  const Eigen::Matrix3d mat = quat.toRotationMatrix();
  const Eigen::Vector3d body_x = mat.col(0);
  const Eigen::Vector3d gravity_world(0.0, 0.0, -1.0);
  const double cos_angle = body_x.dot(gravity_world);

  // 第二阶段只允许肚皮法线与重力对齐（躺平或趴平，偏差不超过 15°）。
  // 同一份实时 IMU 既用于门控，也用于选择 prone/supine，避免侧躺被误判为仰躺。
  if (std::fabs(cos_angle) <= kFallStandFlatCos) {
    RL_LOG_WARNING("autoSelectFallStandModel: stand-up rejected, not flat "
                   "(body_x·gravity=%.3f, need |cos|>%.3f)",
                   cos_angle, kFallStandFlatCos);
    return false;
  }

  const FallStandModelType desired =
      (cos_angle > 0.0) ? FallStandModelType::kProne : FallStandModelType::kSupine;

  if (!switchFallStandModel(desired)) {
    RL_LOG_WARNING("autoSelectFallStandModel: switch failed (desired=%s, cos=%.3f)",
                   (desired == FallStandModelType::kProne) ? "PRONE" : "SUPINE", cos_angle);
    return false;
  }

  RL_LOG_INFO("autoSelectFallStandModel: %s (body_x·gravity=%.3f)",
              (desired == FallStandModelType::kProne) ? "PRONE" : "SUPINE", cos_angle);
  return true;
}

std::vector<std::string> GenericRLController::getMotionNames() const {
  std::vector<std::string> names;
  names.reserve(motions_.size());
  for (const auto& [name, _] : motions_) {
    names.push_back(name);
  }
  return names;
}

std::string GenericRLController::getCurrentMotionName() const {
  return current_motion_name_;
}

// ============================================================================
// 默认姿态获取（有 motion 时返回第一帧）
// ============================================================================

Eigen::VectorXd GenericRLController::getDefaultArmPos() const {
  if (arm_joint_ids_.empty()) {
    return Eigen::VectorXd();
  }

  // 获取目标位置：有 motion 用首帧，否则用默认配置
  // 必须用 getFirstFrameJointPos()：motion 播放完毕后 current_frame_ 停在末帧，
  // 切换控制器时若用末帧作为插值目标会把上一次播放的最终姿态带入新一轮，
  // 例如 mimic_dance 跳完手臂张开 → 切到 amp → 再切回 mimic_dance 时手臂会维持张开。
  MotionTrajectory* motion = getCurrentMotion();
  array_t target_joint_pos = (motion && motion->isLoaded())
      ? motion->getFirstFrameJointPos()
      : default_joint_pos_;

  // 构建电机空间的手臂位置（应用方向系数）
  Eigen::VectorXd arm_pos(arm_joint_ids_.size());
  for (size_t i = 0; i < arm_joint_ids_.size(); ++i) {
    int motor_idx = arm_joint_ids_[i];
    // 找到该电机对应的策略索引
    int policy_idx = -1;
    for (int j = 0; j < policy_joint_count_; ++j) {
      if (policy_joint_ids_[j] == motor_idx) {
        policy_idx = j;
        break;
      }
    }
    if (policy_idx >= 0 && policy_idx < static_cast<int>(target_joint_pos.size())) {
      arm_pos[i] = joint_direction_[policy_idx] * target_joint_pos[policy_idx];
    } else {
      arm_pos[i] = 0.0;
    }
  }
  return arm_pos;
}

Eigen::VectorXd GenericRLController::getDefaultWaistPos() const {
  if (waist_joint_ids_.empty()) {
    return Eigen::VectorXd();
  }

  // 获取目标位置：有 motion 用首帧，否则用默认配置
  // 同 getDefaultArmPos：必须用首帧而非 current_frame_，否则切换控制器时会把上一次播放的末帧位置作为插值目标。
  MotionTrajectory* motion = getCurrentMotion();
  array_t target_joint_pos = (motion && motion->isLoaded())
      ? motion->getFirstFrameJointPos()
      : default_joint_pos_;

  // 构建电机空间的腰部位置（应用方向系数）
  Eigen::VectorXd waist_pos(waist_joint_ids_.size());
  for (size_t i = 0; i < waist_joint_ids_.size(); ++i) {
    int motor_idx = waist_joint_ids_[i];
    // 找到该电机对应的策略索引
    int policy_idx = -1;
    for (int j = 0; j < policy_joint_count_; ++j) {
      if (policy_joint_ids_[j] == motor_idx) {
        policy_idx = j;
        break;
      }
    }
    if (policy_idx >= 0 && policy_idx < static_cast<int>(target_joint_pos.size())) {
      waist_pos[i] = joint_direction_[policy_idx] * target_joint_pos[policy_idx];
    } else {
      waist_pos[i] = 0.0;
    }
  }
  return waist_pos;
}

double GenericRLController::getPartControllerCmdStanceValue() const {
  // v17 amp_hand 增强：stance=0 行走/站立，stance=1 下蹲；跟遥控器 cmd_stance_mode 同步。
  // 对齐闭源 ArmController::updateMode1：cmd_stance=1 时插值到 default_arm_pos 并覆盖关节输出，
  // 下蹲时手臂几乎不动；cmd_stance=0 时手臂仍由 RL 驱动。
  if (enable_amp_arm_enhance_) {
    return static_cast<double>(getCmdStanceMode());
  }
  // v46/v52 等：沿用基类零速防抖，stance=1 表示站立
  return ControllerBase::getPartControllerCmdStanceValue();
}

// ============================================================================
// 关节初始化
// ============================================================================

/// 线性插值过渡到启动站立位置（start_stand_pos；无 motion 且无 start_stand_pos 时用 joint_default_pos）
void GenericRLController::moveToDefaultPos(const RobotState& current_state, double elapse) {
  auto& robot = GlobalRobot::getInstance();

  if (current_state.q.empty()) {
    return;
  }

  // 构建目标位置（全电机）
  // 有 motion 时使用首帧 joint_pos，使机器人在播放前就站在 motion 起始姿态
  std::vector<double> joint_current_pos = current_state.q;
  std::vector<double> joint_target_pos = current_state.q;
  MotionTrajectory* motion = getCurrentMotion();
  array_t target_joint_pos = (motion && motion->isLoaded())
      ? motion->getJointPos()   // motion 首帧关节位置
      : start_stand_joint_pos_;

  // 仅设置策略控制的关节目标（应用方向系数）
  for (int i = 0; i < policy_joint_count_; ++i) {
    int motor_idx = policy_joint_ids_[i];
    if (motor_idx >= 0 && motor_idx < static_cast<int>(joint_target_pos.size())) {
      joint_target_pos[motor_idx] = joint_direction_[i] * target_joint_pos[i];
    }
  }

  if (head_default_pos_.size() == 2) {
    for (int i = 0; i < motor_count_; ++i) {
      if (motor_names_[i] == "zhead_1_link" || motor_names_[i] == "zhead_1_joint") {
        joint_target_pos[i] = head_default_pos_[0];
      } else if (motor_names_[i] == "zhead_2_link" || motor_names_[i] == "zhead_2_joint") {
        joint_target_pos[i] = head_default_pos_[1];
      }
    }
  }
  RL_LOG_INFO("Moving to default position over %.1f seconds...", elapse);

  // 线性插值
  int motor_count = static_cast<int>(current_state.q.size());
  for (double t = 0.; t < elapse && !defaultPoseStopRequested(); t += loop_dt_) {
    double phase = t / elapse;

    RobotCmd cmd(motor_count);
    for (int i = 0; i < motor_count; ++i) {
      cmd.q[i] = joint_current_pos[i] + phase * (joint_target_pos[i] - joint_current_pos[i]);
      cmd.kp[i] = 100.0;
      cmd.kd[i] = 10.0;
      cmd.modes[i] = 2;  // CSP
      cmd.v[i] = 0.0;
      cmd.tau[i] = 0.0;
    }
    // 策略关节使用配置文件中的 actuator_kp/kd (适用于 CANFD 和 EC 电机)
    for (int i = 0; i < policy_joint_count_; ++i) {
      int motor_idx = policy_joint_ids_[i];
      if (motor_idx >= 0 && motor_idx < motor_count) {
        cmd.kp[motor_idx] = joint_kp_[i];
        cmd.kd[motor_idx] = joint_kd_[i];
      }
    }

    robot.publishRobotCmd(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(loop_dt_ * 1000)));
  }

  if (defaultPoseStopRequested()) {
    RL_LOGW("Default position transition interrupted by stop request");
  } else {
    RL_LOG_INFO("Default position reached");
  }
}

// ============================================================================
// 配置加载
// ============================================================================

bool GenericRLController::loadConfig(const std::string& config_path) {
  // 检查文件是否存在
  if (!std::filesystem::exists(config_path)) {
    RL_LOG_FAILURE("Config file not found: %s", config_path.c_str());
    return false;
  }

  // 先调用基类解析通用配置（loop_dt_, 部位控制器配置等）
  if (!ControllerBase::loadConfig(config_path)) {
    return false;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("Failed to parse YAML: %s\n  Error: %s", config_path.c_str(), e.what());
    return false;
  }

  try {
    YAML::Node cfg = root["HumanoidRobotCfg"];
    if (!cfg) {
      RL_LOG_FAILURE("Missing 'HumanoidRobotCfg' section in: %s", config_path.c_str());
      return false;
    }

    // --- RL 特有时序配置 ---
    policy_dt_ = cfg["env"]["policy_dt"].as<double>();
    if (cfg["prone_policy_path"] && cfg["supine_policy_path"]) {
      prone_policy_path_ = cfg["prone_policy_path"].as<std::string>();
      supine_policy_path_ = cfg["supine_policy_path"].as<std::string>();
      RL_LOG_INFO("Dual fall-stand policy paths configured");
    }
    if (cfg["policy_path"]) {
      policy_path_ = cfg["policy_path"].as<std::string>();
    } else if (!prone_policy_path_.empty() && !supine_policy_path_.empty()) {
      policy_path_ = supine_policy_path_;
    } else {
      RL_LOG_FAILURE("Missing 'policy_path' (and no prone/supine pair) in: %s",
                     config_path.c_str());
      return false;
    }

    // --- 推理引擎选择 (可选，未指定时默认使用 OpenVINO) ---
    if (cfg["inference_engine"]) {
      inference_engine_ = cfg["inference_engine"].as<std::string>();
      RL_LOG_INFO("Inference engine: %s", inference_engine_.c_str());
    } else {
      inference_engine_ = "openvino";  // 未指定时默认使用 OpenVINO
    }

    // limp_when_idle：未准备/未起播时瘫软（对齐 kuavo FALL_DOWN 态）
    if (cfg["limp_when_idle"]) {
      limp_when_idle_ = cfg["limp_when_idle"].as<bool>();
      RL_LOG_INFO("limp_when_idle: %s", limp_when_idle_ ? "true" : "false");
    }

    // --- 关节配置 ---
    YAML::Node robot = cfg["env"]["robot"];
    joint_names_ = robot["joint_names"].as<std::vector<std::string>>();

    auto direction_vec = robot["joint_direction"].as<std::vector<double>>();
    auto default_pos_vec = robot["joint_default_pos"].as<std::vector<double>>();
    auto torque_limit_vec = robot["joint_torque_limit"].as<std::vector<double>>();
    auto kp_vec = robot["actuator_kp"].as<std::vector<double>>();
    auto kd_vec = robot["actuator_kd"].as<std::vector<double>>();
    auto control_mode_vec = robot["actuator_control_mode"].as<std::vector<int>>();
    auto action_scale_vec = robot["action_scale"].as<std::vector<double>>();
    if (robot["head_default_pos"] &&
        robot["head_default_pos"].IsSequence()) {
      head_default_pos_ = robot["head_default_pos"].as<std::vector<double>>();
      if (head_default_pos_.size() != 2) head_default_pos_.clear();
    }

    joint_direction_ = Eigen::Map<const Eigen::ArrayXd>(direction_vec.data(), direction_vec.size());
    default_joint_pos_ = Eigen::Map<const Eigen::ArrayXd>(default_pos_vec.data(), default_pos_vec.size());
    if (robot["start_stand_pos"] && robot["start_stand_pos"].IsSequence()) {
      auto start_stand_vec = robot["start_stand_pos"].as<std::vector<double>>();
      start_stand_joint_pos_ =
          Eigen::Map<const Eigen::ArrayXd>(start_stand_vec.data(), start_stand_vec.size());
      if (start_stand_joint_pos_.size() != default_joint_pos_.size()) {
        RL_LOG_WARNING("start_stand_pos size (%ld) != joint_default_pos size (%ld), "
                       "falling back to joint_default_pos for startup",
                       start_stand_joint_pos_.size(), default_joint_pos_.size());
        start_stand_joint_pos_ = default_joint_pos_;
      } else {
        RL_LOG_INFO("Using start_stand_pos for startup interpolation "
                    "(joint_default_pos unchanged for policy)");
      }
    } else {
      start_stand_joint_pos_ = default_joint_pos_;
    }
    joint_torque_limit_ = Eigen::Map<const Eigen::ArrayXd>(torque_limit_vec.data(), torque_limit_vec.size());
    joint_kp_ = Eigen::Map<const Eigen::ArrayXd>(kp_vec.data(), kp_vec.size());
    joint_kd_ = Eigen::Map<const Eigen::ArrayXd>(kd_vec.data(), kd_vec.size());
    joint_control_mode_ = Eigen::Map<const Eigen::ArrayXi>(control_mode_vec.data(), control_mode_vec.size());
    joint_action_scale_ = Eigen::Map<const Eigen::ArrayXd>(action_scale_vec.data(), action_scale_vec.size());

    // --- 观测配置 ---
    YAML::Node obs = cfg["env"]["observations"];
    obs_history_length_ = obs["history_length"].as<int>();
    std::string stack_order_str = obs["stack_order"].as<std::string>();
    obs_stack_order_ = (stack_order_str == "isaaclab") ? StackOrder::kIsaaclab : StackOrder::kClassic;

    for (const auto& pair : obs["terms"]) {
      ObsTermConfig term;
      term.name = pair.first.as<std::string>();
      term.scale = pair.second["scale"].as<double>();
      auto clip = pair.second["clip"].as<std::vector<double>>();
      term.clip = {clip[0], clip[1]};
      obs_terms_.push_back(term);
    }

    // --- Motion 配置（可选） ---
    if (cfg["motion"]) {
      YAML::Node motion = cfg["motion"];

      // 对齐 kuavo fallStandMaxJointVelocity：插值到 CSV 首帧的最大关节速度
      if (motion["prep_max_joint_velocity"]) {
        motion_prep_max_joint_velocity_ =
            motion["prep_max_joint_velocity"].as<double>();
        RL_LOG_INFO("Motion prep max joint velocity: %.3f rad/s",
                    motion_prep_max_joint_velocity_);
      }

      if (motion["motions"]) {
        for (const auto& item : motion["motions"]) {
          std::string motion_name = item["name"].as<std::string>();

          if (item["prone_file"] && item["supine_file"]) {
            fall_stand_motion_base_ = motion_name;
            motion_paths_[motion_name + "_prone"] =
                item["prone_file"].as<std::string>();
            motion_paths_[motion_name + "_supine"] =
                item["supine_file"].as<std::string>();
            RL_LOG_INFO("Dual fall-stand motions for '%s' (prone/supine files)",
                        motion_name.c_str());
          } else {
            std::string motion_path;
            if (item["file"]) {
              motion_path = item["file"].as<std::string>();
            } else if (item["path"]) {
              motion_path = item["path"].as<std::string>();
            }
            motion_paths_[motion_name] = motion_path;
          }
        }

      }

      // 舞蹈播完自动切回稳定控制器（可选，配置在对应舞蹈 yaml 中）
      if (motion["auto_switch_back"]) {
        auto_switch_back_enabled_ =
            motion["auto_switch_back"]["enabled"].as<bool>(false);
        if (motion["auto_switch_back"]["target_controller"]) {
          auto_switch_back_target_ =
              motion["auto_switch_back"]["target_controller"].as<std::string>();
        }
        if (auto_switch_back_enabled_) {
          RL_LOG_INFO("Motion auto switch back enabled, target: %s",
                      auto_switch_back_target_.c_str());
        }
      }
    }
    if (robot["residual_action"]) {
      motion_residual_action_ = robot["residual_action"].as<bool>();
    }
    if (robot["enable_amp_arm_enhance_v17"]) {
      enable_amp_arm_enhance_ = robot["enable_amp_arm_enhance_v17"].as<bool>();
      RL_LOGI("AMP arm enhance (v17): %s",
              enable_amp_arm_enhance_ ? "enabled" : "disabled");
    }
    if (robot["enable_back_enhance_v17"]) {
      enable_back_enhance_v17_ = robot["enable_back_enhance_v17"].as<bool>();
      RL_LOGI("Back enhance (v17): %s",
              enable_back_enhance_v17_ ? "enabled" : "disabled");
    }
    if (robot["enable_standup_enhance_v17"]) {
      enable_standup_enhance_v17_ = robot["enable_standup_enhance_v17"].as<bool>();
      RL_LOGI("Stand-up enhance (v17): %s",
              enable_standup_enhance_v17_ ? "enabled" : "disabled");
    }
    if (robot["enable_turn_in_place_enhance_v17"]) {
      enable_turn_in_place_enhance_v17_ =
          robot["enable_turn_in_place_enhance_v17"].as<bool>();
      RL_LOGI("Turn-in-place enhance (v17): %s",
              enable_turn_in_place_enhance_v17_ ? "enabled" : "disabled");
    }

    // 双策略仅在「双 policy + 双 CSV」齐全时启用
    dual_fall_stand_ = !prone_policy_path_.empty() && !supine_policy_path_.empty() &&
                       !fall_stand_motion_base_.empty();
    if (dual_fall_stand_) {
      RL_LOG_INFO("Dual fall-stand mode enabled for motion '%s'",
                  fall_stand_motion_base_.c_str());
    }

    // --- AMP 速度缩放（可选） ---
    // 支持两种格式：
    //   1) walking/standing 子段：cmd_stance=0/1 使用不同命令语义与缩放
    //   2) 平铺 linear_x/linear_y/angular_z：两种模式共用同一组参数
    YAML::Node velocity_scale = cfg["env"]["velocity_scale"];
    if (velocity_scale) {
      auto parseOneScale = [&](const YAML::Node& node, const std::string& context,
                               double& linear_x, double& linear_x_negative,
                               double& linear_y, double& angular_z) -> bool {
        if (!node["linear_x"] || !node["linear_x_negative_scale"] ||
            !node["linear_y"] || !node["angular_z"]) {
          RL_LOG_FAILURE("Incomplete 'HumanoidRobotCfg.env.velocity_scale.%s' in: %s",
                         context.c_str(), config_path.c_str());
          return false;
        }
        linear_x = node["linear_x"].as<double>();
        linear_x_negative = node["linear_x_negative_scale"].as<double>();
        linear_y = node["linear_y"].as<double>();
        angular_z = node["angular_z"].as<double>();
        return true;
      };

      if (velocity_scale["walking"] || velocity_scale["standing"]) {
        if (!velocity_scale["walking"] || !velocity_scale["standing"]) {
          RL_LOG_FAILURE("velocity_scale must have both 'walking' and 'standing' in: %s",
                         config_path.c_str());
          return false;
        }
        if (!parseOneScale(velocity_scale["walking"], "walking",
                           velocity_scale_linear_x_, velocity_scale_linear_x_negative_,
                           velocity_scale_linear_y_, velocity_scale_angular_z_)) {
          return false;
        }
        const YAML::Node& walking_scale = velocity_scale["walking"];
        velocity_scale_forward_direct_ =
            walking_scale["linear_x_low"] && walking_scale["linear_x_up"];
        velocity_scale_linear_x_up_ =
            walking_scale["linear_x_up"]
                ? walking_scale["linear_x_up"].as<double>()
                : velocity_scale_linear_x_;
        if (walking_scale["external_arm_linear_x_boost"]) {
          external_arm_linear_x_boost_ =
              walking_scale["external_arm_linear_x_boost"].as<double>();
        }
        if (!velocity_scale["standing"]["angular_z"]) {
          RL_LOG_FAILURE(
              "Incomplete 'HumanoidRobotCfg.env.velocity_scale.standing' "
              "(requires angular_z) in: %s",
              config_path.c_str());
          return false;
        }
        velocity_scale_angular_z_standing_ =
            velocity_scale["standing"]["angular_z"].as<double>();
        RL_LOGI(
            "Velocity scale: walking(lx=%.2f, lx_up=%.2f, az=%.2f), "
            "standing(az=%.2f), external_arm_lx_boost=%.2f",
            velocity_scale_linear_x_, velocity_scale_linear_x_up_,
            velocity_scale_angular_z_, velocity_scale_angular_z_standing_,
            external_arm_linear_x_boost_);
      } else {
        if (!parseOneScale(velocity_scale, "velocity_scale",
                           velocity_scale_linear_x_, velocity_scale_linear_x_negative_,
                           velocity_scale_linear_y_, velocity_scale_angular_z_)) {
          return false;
        }
        velocity_scale_angular_z_standing_ = velocity_scale_angular_z_;
        RL_LOGI("Velocity scale: shared(lx=%.2f, az=%.2f)",
                velocity_scale_linear_x_, velocity_scale_angular_z_);
      }
    }

    YAML::Node angular_z_stop_filter = cfg["env"]["angular_z_stop_filter"];
    if (angular_z_stop_filter) {
      VelocityStopFilterConfig filter_config;
      filter_config.enabled = angular_z_stop_filter["enabled"].as<bool>(false);
      filter_config.deceleration =
          angular_z_stop_filter["angular_z_deceleration"].as<double>(1.0);
      filter_config.stop_target_threshold =
          angular_z_stop_filter["stop_target_threshold"].as<double>(0.01);
      angular_z_stop_filter_.setConfig(filter_config);
      if (angular_z_stop_filter["max_standup_change"]) {
        max_standup_change_ =
            angular_z_stop_filter["max_standup_change"].as<double>();
      }
      RL_LOGI("Stance height stand-up smoothing: max_change=%.4f m/step",
              max_standup_change_);
    }

    YAML::Node velocity_smoothing = cfg["env"]["velocity_smoothing"];
    if (velocity_smoothing) {
      velocity_smoothing_.cmd_x_smooth_enabled =
          velocity_smoothing["cmd_x_smooth_enabled"].as<bool>(false);
      velocity_smoothing_.max_velocity_change_cmd_x =
          velocity_smoothing["max_velocity_change_cmd_x"].as<double>(-1.0);
      velocity_smoothing_.max_velocity_change_decel_cmd_x =
          velocity_smoothing["max_velocity_change_decel_cmd_x"].as<double>(-1.0);
      velocity_smoothing_.cmd_x_decel_ema_tau_threshold =
          velocity_smoothing["cmd_x_decel_ema_tau_threshold"].as<double>(0.5);
      velocity_smoothing_.cmd_x_decel_ema_tau_high =
          velocity_smoothing["cmd_x_decel_ema_tau_high"].as<double>(0.15);
      velocity_smoothing_.cmd_x_decel_ema_tau_low =
          velocity_smoothing["cmd_x_decel_ema_tau_low"].as<double>(0.35);
      velocity_smoothing_.max_velocity_change_neg_accel_cmd_x =
          velocity_smoothing["max_velocity_change_neg_accel_cmd_x"].as<double>(-1.0);
      velocity_smoothing_.max_velocity_change_neg_cmd_x =
          velocity_smoothing["max_velocity_change_neg_cmd_x"].as<double>(-1.0);
      velocity_smoothing_.cmd_x_decel_hold_speed =
          velocity_smoothing["cmd_x_decel_hold_speed"].as<double>(-1.0);
      velocity_smoothing_.cmd_x_decel_hold_time =
          velocity_smoothing["cmd_x_decel_hold_time"].as<double>(5.0);
      max_velocity_change_cmd_y_ =
          velocity_smoothing["max_velocity_change_cmd_y"].as<double>(0.0);
      max_velocity_change_cmd_angz_ =
          velocity_smoothing["max_velocity_change_cmd_angz"].as<double>(0.0);
      RL_LOGI(
          "Velocity smoothing: cmd_x=%s, pos_step=%.4f, pos_decel_step=%.4f, "
          "tau(threshold/high/low)=%.3f/%.3f/%.3f, "
          "neg_accel_step=%.4f, neg_decel_step=%.4f, y_step=%.4f, angz_decel_step=%.4f, "
          "hold_speed=%.3f, hold_time=%.1f",
          velocity_smoothing_.cmd_x_smooth_enabled ? "enabled" : "disabled",
          velocity_smoothing_.max_velocity_change_cmd_x,
          velocity_smoothing_.max_velocity_change_decel_cmd_x,
          velocity_smoothing_.cmd_x_decel_ema_tau_threshold,
          velocity_smoothing_.cmd_x_decel_ema_tau_high,
          velocity_smoothing_.cmd_x_decel_ema_tau_low,
          velocity_smoothing_.max_velocity_change_neg_accel_cmd_x,
          velocity_smoothing_.max_velocity_change_neg_cmd_x,
          max_velocity_change_cmd_y_,
          max_velocity_change_cmd_angz_,
          velocity_smoothing_.cmd_x_decel_hold_speed,
          velocity_smoothing_.cmd_x_decel_hold_time);
    }

    YAML::Node posture_axis = cfg["env"]["amp_hand_posture_axis"];
    if (posture_axis) {
      if (posture_axis["squat_knee_angle_max"]) {
        max_squat_knee_angle_ =
            posture_axis["squat_knee_angle_max"].as<double>();
        RL_LOGI("AmpHandPosture: max squat knee angle=%.3f rad",
                max_squat_knee_angle_);
      }
      if (posture_axis["squat_leg1_angle_max"]) {
        max_squat_leg1_angle_ =
            posture_axis["squat_leg1_angle_max"].as<double>();
        RL_LOGI("AmpHandPosture: max squat leg1 angle=%.3f rad "
                "(leg_l1<=-%.3f, leg_r1>=+%.3f)",
                max_squat_leg1_angle_, max_squat_leg1_angle_,
                max_squat_leg1_angle_);
      }
      if (posture_axis["squat_height_min"] && max_squat_knee_angle_ <= 0.0) {
        max_stance_squat_depth_ =
            std::abs(posture_axis["squat_height_min"].as<double>());
      }
    }

    YAML::Node squat_posture_defense = cfg["env"]["squat_posture_defense"];
    if (squat_posture_defense) {
      squat_posture_defense_.enabled =
          squat_posture_defense["enabled"].as<bool>(false);
      squat_posture_defense_.height_threshold =
          squat_posture_defense["height_threshold"].as<double>(
              squat_posture_defense_.height_threshold);
      if (squat_posture_defense_.enabled) {
        RL_LOGI("SquatPostureDefense: enabled, height_threshold=%.3f rad",
                squat_posture_defense_.height_threshold);
      }
    }
    leg_l4_policy_idx_ = findPolicyJointIndex("leg_l4_joint");
    leg_r4_policy_idx_ = findPolicyJointIndex("leg_r4_joint");
    leg_l1_policy_idx_ = findPolicyJointIndex("leg_l1_joint");
    leg_r1_policy_idx_ = findPolicyJointIndex("leg_r1_joint");
    if (squat_posture_defense_.enabled &&
        (leg_l4_policy_idx_ < 0 || leg_r4_policy_idx_ < 0)) {
      RL_LOGW("SquatPostureDefense enabled but leg_l4/leg_r4 joint not found");
    }

    YAML::Node tiny_cmd_clip = cfg["env"]["tiny_cmd_clip"];
    if (tiny_cmd_clip) {
      tiny_cmd_clip_.enabled = tiny_cmd_clip["enabled"].as<bool>(false);
      if (tiny_cmd_clip_.enabled) {
        const auto cmd_x_clip =
            tiny_cmd_clip["cmd_x_clip"].as<std::vector<double>>();
        const auto cmd_abs_y_clip =
            tiny_cmd_clip["cmd_abs_y_clip"].as<std::vector<double>>();

        if (cmd_x_clip.size() != 2 || cmd_abs_y_clip.size() != 2) {
          RL_LOG_FAILURE(
              "Invalid env.tiny_cmd_clip in %s: cmd_x_clip and cmd_abs_y_clip "
              "must each have 2 values",
              config_path.c_str());
          return false;
        }

        tiny_cmd_clip_.cmd_x_pos_min = cmd_x_clip[0];
        tiny_cmd_clip_.cmd_x_pos_max = cmd_x_clip[1];
        tiny_cmd_clip_.cmd_abs_y_min = cmd_abs_y_clip[0];
        tiny_cmd_clip_.cmd_abs_y_max = cmd_abs_y_clip[1];

        tiny_cmd_clip_.cmd_x_enabled =
            tiny_cmd_clip_.cmd_x_pos_max > tiny_cmd_clip_.cmd_x_pos_min;
        tiny_cmd_clip_.cmd_y_enabled =
            tiny_cmd_clip_.cmd_abs_y_max > tiny_cmd_clip_.cmd_abs_y_min;
      }
      RL_LOGI("TinyCmdClip: %s (x=%s, y=%s)",
              tiny_cmd_clip_.enabled ? "enabled" : "disabled",
              tiny_cmd_clip_.cmd_x_enabled ? "on" : "off",
              tiny_cmd_clip_.cmd_y_enabled ? "on" : "off");
    }

    auto loadLowSpeedKickStart = [&](const YAML::Node& node,
                                     LowSpeedKickStartConfig& cfg,
                                     const char* name) {
      if (!node) {
        return;
      }
      cfg.enabled = node["enabled"].as<bool>(cfg.enabled);
      cfg.kick_velocity = node["kick_velocity"].as<double>(cfg.kick_velocity);
      cfg.duration_steps = node["duration_steps"].as<int>(cfg.duration_steps);
      cfg.rest_cmd_threshold =
          node["rest_cmd_threshold"].as<double>(cfg.rest_cmd_threshold);
      cfg.trigger_velocity =
          node["trigger_velocity"].as<double>(cfg.trigger_velocity);
      cfg.trigger_tolerance =
          node["trigger_tolerance"].as<double>(cfg.trigger_tolerance);
      cfg.lateral_threshold =
          node["lateral_threshold"].as<double>(cfg.lateral_threshold);
      cfg.yaw_threshold = node["yaw_threshold"].as<double>(cfg.yaw_threshold);
      cfg.duration_steps = std::max(cfg.duration_steps, 1);
      cfg.trigger_tolerance = std::max(cfg.trigger_tolerance, 0.0);
      if (cfg.enabled) {
        RL_LOGI(
            "%s: enabled (kick_vel=%.2f, steps=%d, trigger=%.2f±%.2f, "
            "rest=%.2f)",
            name, cfg.kick_velocity, cfg.duration_steps, cfg.trigger_velocity,
            cfg.trigger_tolerance, cfg.rest_cmd_threshold);
      }
    };
    auto loadLowSpeedYawKickStart = [&](const YAML::Node& node,
                                        LowSpeedYawKickStartConfig& cfg,
                                        const char* name) {
      if (!node) {
        return;
      }
      cfg.enabled = node["enabled"].as<bool>(cfg.enabled);
      cfg.kick_angular_velocity =
          node["kick_angular_velocity"].as<double>(cfg.kick_angular_velocity);
      cfg.duration_steps = node["duration_steps"].as<int>(cfg.duration_steps);
      cfg.rest_cmd_threshold =
          node["rest_cmd_threshold"].as<double>(cfg.rest_cmd_threshold);
      cfg.trigger_angular_velocity =
          node["trigger_angular_velocity"].as<double>(
              cfg.trigger_angular_velocity);
      cfg.trigger_tolerance =
          node["trigger_tolerance"].as<double>(cfg.trigger_tolerance);
      cfg.forward_threshold =
          node["forward_threshold"].as<double>(cfg.forward_threshold);
      cfg.lateral_threshold =
          node["lateral_threshold"].as<double>(cfg.lateral_threshold);
      cfg.enable_reverse_kick_guard =
          node["enable_reverse_kick_guard"].as<bool>(
              cfg.enable_reverse_kick_guard);
      cfg.duration_steps = std::max(cfg.duration_steps, 1);
      cfg.trigger_tolerance = std::max(cfg.trigger_tolerance, 0.0);
      cfg.kick_angular_velocity = std::max(cfg.kick_angular_velocity, 0.0);
      if (cfg.enabled || cfg.enable_reverse_kick_guard) {
        RL_LOGI(
            "%s: kick=%s (angz=%.2f, steps=%d, trigger=%.2f±%.2f, rest=%.2f), "
            "reverse_guard=%s (history=%d steps)",
            name, cfg.enabled ? "enabled" : "disabled", cfg.kick_angular_velocity,
            cfg.duration_steps, cfg.trigger_angular_velocity,
            cfg.trigger_tolerance, cfg.rest_cmd_threshold,
            cfg.enable_reverse_kick_guard ? "enabled" : "disabled",
            kReverseYawKickGuardHistorySteps_);
      }
    };
    loadLowSpeedKickStart(cfg["env"]["low_speed_kick_start"],
                          low_speed_kick_start_, "LowSpeedKickStart");
    loadLowSpeedYawKickStart(cfg["env"]["low_speed_yaw_kick_start"],
                             low_speed_yaw_kick_start_, "LowSpeedYawKickStart");

    YAML::Node mixed_motion_limits = cfg["env"]["mixedMotionLimits"];
    if (mixed_motion_limits) {
      mixed_motion_limits_.enable_mixed_mode =
          mixed_motion_limits["enableMixedMode"].as<bool>(false);
      mixed_motion_limits_.angular_vel_threshold =
          mixed_motion_limits["angularVelThreshold"].as<double>(0.05);
      mixed_motion_limits_.max_linear_vel_with_angular =
          mixed_motion_limits["maxLinearVelWithAngular"].as<double>(0.85);
      mixed_motion_limits_.linear_vel_threshold =
          mixed_motion_limits["linearVelThreshold"].as<double>(0.05);
      mixed_motion_limits_.max_angular_vel_with_linear =
          mixed_motion_limits["maxAngularVelWithLinear"].as<double>(0.7);
      RL_LOGI(
          "MixedMotionLimits: %s (max_lin_with_ang=%.2f, max_ang_with_lin=%.2f)",
          mixed_motion_limits_.enable_mixed_mode ? "enabled" : "disabled",
          mixed_motion_limits_.max_linear_vel_with_angular,
          mixed_motion_limits_.max_angular_vel_with_linear);
    }

    YAML::Node roll_comp_cfg = cfg["env"]["roll_compensation_closed_loop"];
    if (cfg["env"]["enable_roll_compensation_closed_loop"]) {
      roll_compensation_closed_loop_.enabled =
          cfg["env"]["enable_roll_compensation_closed_loop"].as<bool>();
    }
    if (roll_comp_cfg) {
      roll_compensation_closed_loop_.cmd_x_min =
          roll_comp_cfg["cmd_x_min"].as<double>(
              roll_compensation_closed_loop_.cmd_x_min);
      roll_compensation_closed_loop_.cmd_x_max =
          roll_comp_cfg["cmd_x_max"].as<double>(
              roll_compensation_closed_loop_.cmd_x_max);
      roll_compensation_closed_loop_.abs_cmd_y_max =
          roll_comp_cfg["abs_cmd_y_max"].as<double>(
              roll_compensation_closed_loop_.abs_cmd_y_max);
      roll_compensation_closed_loop_.abs_cmd_ang_z_max =
          roll_comp_cfg["abs_cmd_ang_z_max"].as<double>(
              roll_compensation_closed_loop_.abs_cmd_ang_z_max);
      roll_compensation_closed_loop_.filter_time_constant_sec =
          roll_comp_cfg["filter_time_constant_sec"].as<double>(
              roll_compensation_closed_loop_.filter_time_constant_sec);
      roll_compensation_closed_loop_.first_frame_max_abs_roll_deg =
          roll_comp_cfg["first_frame_max_abs_roll_deg"].as<double>(
              roll_compensation_closed_loop_.first_frame_max_abs_roll_deg);
      roll_compensation_closed_loop_.kp =
          roll_comp_cfg["kp"].as<double>(roll_compensation_closed_loop_.kp);
      roll_compensation_closed_loop_.ki =
          roll_comp_cfg["ki"].as<double>(roll_compensation_closed_loop_.ki);
      roll_compensation_closed_loop_.max_compensation_deg =
          roll_comp_cfg["max_compensation_deg"].as<double>(
              roll_compensation_closed_loop_.max_compensation_deg);
      roll_compensation_closed_loop_.unable_compensation_arm_controller =
          roll_comp_cfg["unable_compensation_arm_controller"].as<bool>(
              roll_compensation_closed_loop_.unable_compensation_arm_controller);
    }
    roll_compensation_closed_loop_.cmd_x_min =
        std::max(roll_compensation_closed_loop_.cmd_x_min, 0.0);
    roll_compensation_closed_loop_.cmd_x_max = std::max(
        roll_compensation_closed_loop_.cmd_x_max,
        roll_compensation_closed_loop_.cmd_x_min + 1e-3);
    roll_compensation_closed_loop_.filter_time_constant_sec =
        std::max(roll_compensation_closed_loop_.filter_time_constant_sec, 1e-3);
    roll_compensation_closed_loop_.ki =
        std::max(roll_compensation_closed_loop_.ki, 0.0);
    roll_compensation_closed_loop_.max_compensation_deg =
        std::max(roll_compensation_closed_loop_.max_compensation_deg, 0.0);
    roll_compensation_closed_loop_.first_frame_max_abs_roll_deg =
        std::max(roll_compensation_closed_loop_.first_frame_max_abs_roll_deg,
                 0.0);
    if (roll_compensation_closed_loop_.enabled) {
      RL_LOGI(
          "Roll compensation closed loop: enabled (cmd_x=[%.2f, %.2f], kp=%.2f, "
          "ki=%.2f, max=%.2f deg, first_frame_max_abs_roll=%.2f deg, "
          "roll_zero=frozen first frame, disable_on_external_arm=%s)",
          roll_compensation_closed_loop_.cmd_x_min,
          roll_compensation_closed_loop_.cmd_x_max,
          roll_compensation_closed_loop_.kp, roll_compensation_closed_loop_.ki,
          roll_compensation_closed_loop_.max_compensation_deg,
          roll_compensation_closed_loop_.first_frame_max_abs_roll_deg,
          roll_compensation_closed_loop_.unable_compensation_arm_controller
              ? "true"
              : "false");
    }
    YAML::Node lateral_yaw_comp_cfg =
        cfg["env"]["lateral_yaw_compensation_closed_loop"];
    if (cfg["env"]["enable_lateral_yaw_compensation_closed_loop"]) {
      lateral_yaw_compensation_closed_loop_.enabled =
          cfg["env"]["enable_lateral_yaw_compensation_closed_loop"].as<bool>();
    }
    if (lateral_yaw_comp_cfg) {
      lateral_yaw_compensation_closed_loop_.kp =
          lateral_yaw_comp_cfg["kp"].as<double>(
              lateral_yaw_compensation_closed_loop_.kp);
      lateral_yaw_compensation_closed_loop_.kd =
          lateral_yaw_comp_cfg["kd"].as<double>(
              lateral_yaw_compensation_closed_loop_.kd);
      lateral_yaw_compensation_closed_loop_.max_action =
          lateral_yaw_comp_cfg["max_action"].as<double>(
              lateral_yaw_compensation_closed_loop_.max_action);
    }
    lateral_yaw_compensation_closed_loop_.kp =
        std::max(lateral_yaw_compensation_closed_loop_.kp, 0.0);
    lateral_yaw_compensation_closed_loop_.kd =
        std::max(lateral_yaw_compensation_closed_loop_.kd, 0.0);
    lateral_yaw_compensation_closed_loop_.max_action =
        std::max(lateral_yaw_compensation_closed_loop_.max_action, 0.0);
    if (lateral_yaw_compensation_closed_loop_.enabled) {
      RL_LOGI(
          "Lateral yaw compensation closed loop: enabled (kp=%.2f, kd=%.2f, "
          "max_action=%.3f)",
          lateral_yaw_compensation_closed_loop_.kp,
          lateral_yaw_compensation_closed_loop_.kd,
          lateral_yaw_compensation_closed_loop_.max_action);
    }

    YAML::Node off_cmdx_after_lateral_cfg = cfg["env"]["off_cmdx_after_lateral"];
    if (off_cmdx_after_lateral_cfg) {
      off_cmdx_after_lateral_.enabled =
          off_cmdx_after_lateral_cfg["enabled"].as<bool>(
              off_cmdx_after_lateral_.enabled);
      off_cmdx_after_lateral_.cmd_y_threshold =
          off_cmdx_after_lateral_cfg["cmd_y_threshold"].as<double>(
              off_cmdx_after_lateral_.cmd_y_threshold);
      off_cmdx_after_lateral_.history_frames =
          off_cmdx_after_lateral_cfg["history_frames"].as<int>(
              off_cmdx_after_lateral_.history_frames);
    }
    off_cmdx_after_lateral_.history_frames =
        std::max(off_cmdx_after_lateral_.history_frames, 1);
    off_cmdx_after_lateral_.cmd_y_threshold =
        std::max(off_cmdx_after_lateral_.cmd_y_threshold, 0.0);
    if (off_cmdx_after_lateral_.enabled) {
      RL_LOGI(
          "Off cmd_x after lateral: enabled (threshold=%.2f, history_frames=%d)",
          off_cmdx_after_lateral_.cmd_y_threshold,
          off_cmdx_after_lateral_.history_frames);
    }

    return true;
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("Config error in %s\n  %s", config_path.c_str(), e.what());
    return false;
  }
}

bool GenericRLController::loadPolicy(const std::string& policy_path) {
  // 使用工厂模式创建推理模型
  model_ = ModelFactory::create(inference_engine_);

  if (!model_) {
    RL_LOG_FAILURE("Failed to create inference model: %s (supported: openvino, onnxruntime)",
                   inference_engine_.c_str());
    return false;
  }

  if (!model_->load(policy_path)) {
    RL_LOG_FAILURE("Failed to load policy model: %s", policy_path.c_str());
    return false;
  }

  RL_LOG_SUCCESS("Policy model loaded: %s (engine: %s)",
                 policy_path.c_str(),
                 ModelFactory::typeToString(model_->getModelType()).c_str());
  return true;
}

// ============================================================================
// 观测 → 推理 → 控制命令
// ============================================================================

/// 构建观测向量：计算各 obs term → 更新历史缓冲 → 展平到 observations_
void GenericRLController::computeObservation() {
  updateVelocityCommandCache();

  // 更新历史缓冲区：弹出最老帧，压入当前帧
  if (obs_stack_order_ == StackOrder::kIsaaclab) {
    // isaaclab 模式：按 term 分组，每个 term 独立维护时间序列
    // obs_stacks_[term_index][time_index]
    for (size_t i = 0; i < obs_terms_.size(); ++i) {
      obs_stacks_[i].pop_front();
      array_t term = getObsTerm(obs_terms_[i].name);
      term *= obs_terms_[i].scale;
      term = term.min(obs_terms_[i].clip.upper).max(obs_terms_[i].clip.lower);
      obs_stacks_[i].push_back(term);
    }
  } else {
    // classic 模式：按时间分组，每个时刻包含所有 term
    // obs_stacks_[time_index][term_index]
    obs_stacks_.pop_front();
    std::deque<array_t> single_obs;
    for (size_t i = 0; i < obs_terms_.size(); ++i) {
      array_t term = getObsTerm(obs_terms_[i].name);
      // classic 模式只 clip，不 scale（与 rl_mimic_controller 一致）
      term = term.min(obs_terms_[i].clip.upper).max(obs_terms_[i].clip.lower);
      single_obs.push_back(term);
    }
    obs_stacks_.push_back(single_obs);
  }

  // 展平 2D deque → 1D observations_ 向量
  int offset = 0;
  for (size_t i = 0; i < obs_stacks_.size(); ++i) {
    for (size_t j = 0; j < obs_stacks_[i].size(); ++j) {
      observations_.segment(offset, obs_stacks_[i][j].size()) = obs_stacks_[i][j];
      offset += obs_stacks_[i][j].size();
    }
  }

  // 发布观测调试数据
  if (logger_) {
    // 逐项发布当前帧的每个观测项
    for (size_t i = 0; i < obs_terms_.size(); ++i) {
      array_t term = getObsTerm(obs_terms_[i].name);
      std::vector<double> v(term.data(), term.data() + term.size());
      logger_->publishVector("/rl_controller/obs/" + obs_terms_[i].name, v);
    }
    // 发布拼接后的单帧观测向量
    std::vector<double> single_obs_vec(observations_.data(),
                                       observations_.data() + observations_.size());
    logger_->publishVector("/rl_controller/observations", single_obs_vec);

    // 发布 policy 的速度指令
    const array_t vel_cmd = getVelocityCommands();
    std::vector<double> vel_cmd_vec(vel_cmd.data(), vel_cmd.data() + vel_cmd.size());
    logger_->publishVector("/rl_controller/velocity_cmd_to_policy", vel_cmd_vec);
  }
}

/// 执行策略推理：observations_ → model_ → actions_
void GenericRLController::computeActions() {
  array_t inferred_actions;
  if (inferActions(observations_, inferred_actions)) {
    updateActionCache(inferred_actions);
  }
}

/// 将 actions_ 转换为电机控制命令
void GenericRLController::updateRobotCmd(RobotCmd& cmd) {
  if (cmd.q.size() != static_cast<size_t>(motor_count_)) {
    cmd.resize(motor_count_);
  }

  // 计算目标关节位置：q_target = direction * (base_pos + action * scale)
  array_t q_target;
  const array_t local_actions = getActionsSnapshot();
  MotionTrajectory* loader = getCurrentMotion();
  if (loader) {
    if (motion_residual_action_) {
      const array_t& base_pos = loader->isLoaded()
          ? loader->getJointPos()
          : default_joint_pos_;

      q_target = joint_direction_ * (base_pos + local_actions * joint_action_scale_);

      if (!motion_playing_) {
        for (int i = 0; i < policy_joint_count_; ++i) {
          if (joint_names_[i].find("zarm") != std::string::npos) {
            q_target[i] = joint_direction_[i] * base_pos[i];
          }
        }
      }
    } else {
      q_target = joint_direction_ * (default_joint_pos_ + local_actions * joint_action_scale_);
    }
  } else {
    q_target = joint_direction_ * (default_joint_pos_ + local_actions * joint_action_scale_);
  }

  // 倒地起身起播后的前段：CSV 与策略正常推进，但从固定 READY 姿态平滑过渡。
  if (fall_stand_entry_blend_active_) {
    if (motion_prep_target_q_.size() < static_cast<size_t>(motor_count_)) {
      RL_LOG_WARNING("Fall-stand entry blend cancelled: READY target is unavailable");
      fall_stand_entry_blend_active_ = false;
    } else {
      const double u = static_cast<double>(fall_stand_entry_blend_frame_) /
                       static_cast<double>(kFallStandEntryBlendFrames - 1);
      const double u2 = u * u;
      const double blend = 10.0 * u2 * u - 15.0 * u2 * u2 + 6.0 * u2 * u2 * u;
      for (int i = 0; i < policy_joint_count_; ++i) {
        const int motor_id = policy_joint_ids_[i];
        if (motor_id >= 0 && motor_id < motor_count_) {
          const double ready_target =
              joint_direction_[i] * motion_prep_target_q_[static_cast<size_t>(motor_id)];
          q_target[i] = (1.0 - blend) * ready_target + blend * q_target[i];
        }
      }

      ++fall_stand_entry_blend_frame_;
      if (fall_stand_entry_blend_frame_ >= kFallStandEntryBlendFrames) {
        fall_stand_entry_blend_active_ = false;
        RL_LOG_INFO("Fall-stand entry blend complete; CSV/policy targets are fully active");
      }
    }
  }

  applySquatJointAngleLimits(q_target);

  last_policy_q_target_ = q_target;

  // 非策略控制的关节：保持当前位置
  if (!held_non_policy_pos_initialized_ &&
      current_state_.q.size() == static_cast<size_t>(motor_count_)) {
    held_non_policy_pos_ = current_state_.q;
    if (head_default_pos_.size() == 2 && motor_count_ >= 2) {
      held_non_policy_pos_[motor_count_ - 2] = head_default_pos_[0];
      held_non_policy_pos_[motor_count_ - 1] = head_default_pos_[1];
    }
    held_non_policy_pos_initialized_ = true;
  }
  for (int i = 0; i < motor_count_; ++i) {
    // 这些关节（例如头部）不在策略输出中。必须显式保持当前状态，
    // 不能依赖 RobotCmd 默认初始化为零，否则一进入策略控制头部会被
    // 发送到 q=0，并表现为缓慢下坠。
    if (i < static_cast<int>(held_non_policy_pos_.size())) {
      cmd.q[i] = held_non_policy_pos_[i];
    } else if (i < static_cast<int>(current_state_.q.size())) {
      cmd.q[i] = current_state_.q[i];
    }
    if (i < static_cast<int>(cmd.v.size())) cmd.v[i] = 0.0;
    if (i < static_cast<int>(cmd.tau.size())) cmd.tau[i] = 0.0;
    cmd.modes[i] = 2;
    cmd.kp[i] = 100.0;
    cmd.kd[i] = 10.0;
  }

  // 策略控制的关节：根据控制模式（CSP/CSV/CST）设置命令
  for (int i = 0; i < policy_joint_count_; ++i) {
    int motor_id = policy_joint_ids_[i];
    double policy_q = current_state_.q[motor_id];
    double policy_v = current_state_.v[motor_id];

    if (joint_control_mode_[i] == 2) {
      // CSP: 软件PD计算力矩，通过前馈力矩发送，位置设为当前值
      cmd.q[motor_id] = policy_q;
      cmd.tau[motor_id] = joint_kp_[i] * (q_target[i] - policy_q);
      // // CSP: 直接位置控制
      // cmd.q[motor_id] = q_target[i];
      // cmd.tau[motor_id] = 0.0;
    } else if (joint_control_mode_[i] == 1) {
      // CSV: 位置误差 → 力矩
      cmd.q[motor_id] = policy_q;
      cmd.tau[motor_id] = joint_kp_[i] * (q_target[i] - policy_q);
    } else {
      // CST: PD 控制 → 力矩
      cmd.q[motor_id] = policy_q;
      cmd.tau[motor_id] = joint_kp_[i] * (q_target[i] - policy_q) +
                          joint_kd_[i] * (-policy_v);
    }

    // 力矩限幅
    double tau_limit = joint_torque_limit_[i];
    cmd.tau[motor_id] = std::clamp(cmd.tau[motor_id], -tau_limit, tau_limit);

    cmd.v[motor_id] = 0.0;
    cmd.modes[motor_id] = (joint_control_mode_[i] == 0) ? 0 : 2;
    if (joint_control_mode_[i] == 0) {
      ////////////////////////////////////////////////
      // mode 0 (CST): 软件已计算完整PD力矩，硬件层不需要再做PD反馈
      // 硬件控制律: final = tau + kp*(q_cmd-q) + kd*(v_cmd-v)
      // 设 kp=kd=0 避免双重阻尼
      /////////////////////////////////////////////////
      cmd.kp[motor_id] = 0.;
      cmd.kd[motor_id] = 0.;
    } else {
      cmd.kp[motor_id] = joint_kp_[i];
      cmd.kd[motor_id] = joint_kd_[i];
    }
  }

  if (name_.find("depth") != std::string::npos &&
      diagnostic_cmd_log_count_ < 8 &&
      local_actions.size() == policy_joint_count_ &&
      local_actions.abs().maxCoeff() > 1e-6) {
    std::ostringstream cmd_text;
    cmd_text.setf(std::ios::fixed);
    cmd_text.precision(5);
    for (int i = 0; i < std::min(policy_joint_count_, 13); ++i) {
      const int motor_id = policy_joint_ids_[i];
      if (i != 0) cmd_text << " | ";
      cmd_text << "p" << i << "->m" << motor_id
               << " a=" << local_actions[i]
               << " s=" << joint_action_scale_[i]
               << " q=" << q_target[i]
               << " dq=" << (q_target[i] - joint_direction_[i] * default_joint_pos_[i])
               << " tau=" << cmd.tau[motor_id];
    }
    RL_LOGI("Depth command diagnostic #%d: %s", diagnostic_cmd_log_count_,
            cmd_text.str().c_str());
    ++diagnostic_cmd_log_count_;
  }

  cmd.timestamp = leju::common::GetUnixTimestampS();

  // 发布调试数据
  // q_target is diagnostic only. Do not serialize it on every 1 ms depth tick.
  const bool depth_diag_tick =
      name_.find("depth") != std::string::npos && (step_count_ % 20 != 0);
  if (logger_ && !depth_diag_tick) {
    std::vector<double> qt(q_target.data(), q_target.data() + q_target.size());
    logger_->publishVector("/rl_controller/q_target", qt);
    if (loader) {
      logger_->publishValue("/rl_controller/motion_frame",
                            static_cast<double>(loader->getCurrentFrame()));
    }
  }
}

// ============================================================================
// 手臂控制指令（发布 arm_mode 话题）
// ============================================================================

void GenericRLController::updateArmCommand(RobotCmd& cmd) {
  // 倒地插值/保持阶段：禁止手臂控制器改写，确保锁在 CSV 首帧目标
  if (motion_prep_state_ != MotionPrepState::kIdle) {
    return;
  }

  ControllerBase::updateArmCommand(cmd);

  // v17 amp_hand 专用：仅下蹲姿态(kAuto+cmd_stance=1)用 CSP 锁定 default_arm_pos。
  // VR(kExternal)/KeepPose/其他机型/行走态保持基类原有 q/v/tau 覆盖，不改控制模式。
  if (enable_amp_arm_enhance_ && getCmdStanceMode() == 1 && arm_controller_ &&
      arm_controller_->getMode() == ArmControlMode::kAuto) {
    for (int i = 0; i < static_cast<int>(arm_joint_names_.size()); ++i) {
      const int policy_idx = findPolicyJointIndex(arm_joint_names_[i]);
      if (policy_idx < 0) {
        continue;
      }
      const int motor_id = arm_joint_ids_[i];
      cmd.modes[motor_id] = 2;
      cmd.kp[motor_id] = joint_kp_[policy_idx];
      cmd.kd[motor_id] = joint_kd_[policy_idx];
    }
  }

  if (logger_ && arm_controller_) {
    logger_->publishValue("/rl_controller/arm_mode",
                          static_cast<double>(arm_controller_->getMode()));
  }
}

// ============================================================================
// 腰部控制指令（发布 waist_mode 话题）
// ============================================================================

void GenericRLController::updateWaistCommand(RobotCmd& cmd) {
  // 倒地插值/保持阶段：禁止腰部控制器改写，持续锁在终态目标
  if (motion_prep_state_ != MotionPrepState::kIdle) {
    return;
  }

  // 调用基类实现
  ControllerBase::updateWaistCommand(cmd);

  //更新策略计算的参考关节位置
  updateBlendReferenceCmd(cmd);

  // 发布腰部控制器模式
  if (logger_ && waist_controller_) {
    logger_->publishValue("/rl_controller/waist_mode",
                          static_cast<double>(waist_controller_->getMode()));
  }
}


void GenericRLController::updateBlendReferenceCmd(const RobotCmd& final_cmd) {
  blend_reference_cmd_ = final_cmd;
  if (!blend_reference_cmd_.isValid() ||
      blend_reference_cmd_.q.size() != current_state_.q.size()) {
    blend_reference_cmd_ = RobotCmd();
    blend_torque_limits_.resize(0);
    blend_recompute_mask_.resize(0);
    return;
  }

  const int motor_count = static_cast<int>(blend_reference_cmd_.q.size());
  blend_torque_limits_.resize(motor_count);
  blend_torque_limits_.setZero();
  blend_recompute_mask_.resize(motor_count);
  blend_recompute_mask_.setZero();

  // 是否被外部控制器接管：与 ControllerBase::updateArm/WaistCommand 的早返回条件一致
  const bool arm_taken_over   = (arm_controller_   != nullptr) && !arm_joint_names_.empty();
  const bool waist_taken_over = (waist_controller_ != nullptr) && !waist_joint_names_.empty();

  for (int i = 0; i < policy_joint_count_; ++i) {
    const int motor_id = policy_joint_ids_[i];
    if (motor_id < 0 || motor_id >= motor_count) {
      continue;
    }

    // 显式判定该关节是否会被 arm/waist 控制器在 updateRobotCmd() 之后改写：
    //  - 手臂关节 + arm_controller 启用 → 已被外部接管，mask=0（沿用 final_cmd 的字段）
    //  - 腰部关节 + waist_controller 启用 → 同上
    //  - 其余 policy 关节（腿、以及 controller 未启用时的手臂/腰）→ 纯 RL 控制，mask=1
    const bool is_arm =
        std::find(arm_joint_ids_.begin(), arm_joint_ids_.end(), motor_id) != arm_joint_ids_.end();
    const bool is_waist =
        std::find(waist_joint_ids_.begin(), waist_joint_ids_.end(), motor_id) != waist_joint_ids_.end();
    const bool overridden_externally =
        (is_arm && arm_taken_over) || (is_waist && waist_taken_over);
    if (overridden_externally) {
      continue;
    }

    blend_reference_cmd_.q[motor_id] = last_policy_q_target_[i];
    blend_reference_cmd_.v[motor_id] = 0.0;
    blend_reference_cmd_.kp[motor_id] = joint_kp_[i];
    blend_reference_cmd_.kd[motor_id] = joint_kd_[i];
    blend_reference_cmd_.modes[motor_id] = (joint_control_mode_[i] == 0) ? 0 : 2;
    blend_torque_limits_[motor_id] = joint_torque_limit_[i];
    blend_recompute_mask_[motor_id] = 1;
  }
}

// ============================================================================
// 关节映射
// ============================================================================

/// 将策略关节名称映射到 SDK 电机索引
bool GenericRLController::buildJointMapping() {
  auto& robot = GlobalRobot::getInstance();
  motor_count_ = robot.getMotorNumber();
  motor_names_ = robot.getMotorNames();
  policy_joint_ids_.clear();
  for (const auto& joint_name : joint_names_) {
    int joint_id = -1;
    for (int j = 0; j < motor_count_; ++j) {
      if (joint_name == motor_names_[j]) {
        joint_id = j;
        break;
      }
    }
    if (joint_id == -1) {
      RL_LOG_FAILURE("Joint not found: %s", joint_name.c_str());
      return false;
    }
    policy_joint_ids_.push_back(joint_id);
  }

  policy_joint_count_ = static_cast<int>(policy_joint_ids_.size());
  RL_LOG_INFO("Joint mapping built: %d joints", policy_joint_count_);
  if (name_.find("depth") != std::string::npos) {
    for (int i = 0; i < policy_joint_count_; ++i) {
      RL_LOGI("Depth policy map[%d] %s -> motor[%d] direction=%.1f",
              i, joint_names_[i].c_str(), policy_joint_ids_[i],
              joint_direction_[i]);
    }
  }
  return true;
}

// ============================================================================
// Motion 轨迹加载
// ============================================================================

/// 从配置目录加载所有 motion 轨迹文件
void GenericRLController::loadMotionTrajectories(const std::string& config_dir) {
  for (const auto& [name, path] : motion_paths_) {
    std::string motion_full_path;
    try {
      motion_full_path = UriPathResolver::resolve(path, config_dir);
    } catch (const UriResolveError& e) {
      RL_LOG_WARNING("Failed to resolve motion path '%s': %s", path.c_str(), e.what());
      continue;
    }
    RL_LOG_INFO("Loading motion '%s' from: %s", name.c_str(), motion_full_path.c_str());

    auto motion_traj = std::make_unique<MotionTrajectory>();
    if (!motion_traj->load(motion_full_path)) {
      RL_LOG_WARNING("Failed to load motion '%s': %s", name.c_str(), motion_full_path.c_str());
      continue;
    }
    RL_LOG_SUCCESS("Motion '%s' loaded: %d frames", name.c_str(), motion_traj->getNumFrames());
    motions_[name] = std::move(motion_traj);
  }

  // 默认使用第一个 motion；双策略则默认 supine 轨迹
  if (dual_fall_stand_ && !fall_stand_motion_base_.empty()) {
    current_motion_name_ = fall_stand_motion_base_ + "_supine";
    fall_stand_model_type_ = FallStandModelType::kSupine;
    RL_LOG_INFO("Default dual fall-stand motion: %s", current_motion_name_.c_str());
  } else if (!motions_.empty()) {
    current_motion_name_ = motions_.begin()->first;
    RL_LOG_INFO("Default motion: %s", current_motion_name_.c_str());
  }
}

MotionTrajectory* GenericRLController::getCurrentMotion() const {
  if (current_motion_name_.empty()) {
    return nullptr;
  }
  auto it = motions_.find(current_motion_name_);
  if (it == motions_.end()) {
    return nullptr;
  }
  return it->second.get();
}

// ============================================================================
// 观测项计算
// ============================================================================

/// 获取策略关节位置（相对默认位置的偏移，已乘方向系数）
/// 公式：direction * q - default（与 kuavo-RL humanoidController 一致）
/// 注意：不能用 direction * (q - default)，否则腰部等需要翻转的关节会差 2*default
array_t GenericRLController::getPolicyJointPos() const {
  array_t policy_q(policy_joint_count_);
  for (int i = 0; i < policy_joint_count_; ++i) {
    policy_q[i] = current_state_.q[policy_joint_ids_[i]];
  }
  // return joint_direction_ * (policy_q - default_joint_pos_);  // old impl
  array_t result = joint_direction_ * policy_q - default_joint_pos_;
  // use_virtual_arm_obs：外部手臂接管时手臂关节观测置零（含下蹲）
  if (enable_amp_arm_enhance_ && isExternalArmControlActive() &&
      !isVirtualArmObsBlockedByLowCmd() && arm_policy_start_idx_ >= 0) {
    for (size_t i = 0; i < arm_joint_names_.size(); ++i) {
      const int idx = arm_policy_start_idx_ + static_cast<int>(i);
      if (idx >= 0 && idx < policy_joint_count_) {
        result[idx] = 0.0;
      }
    }
  }
  return result;
}

/// 获取策略关节速度（已乘方向系数）
array_t GenericRLController::getPolicyJointVel() const {
  array_t policy_v(policy_joint_count_);
  for (int i = 0; i < policy_joint_count_; ++i) {
    policy_v[i] = current_state_.v[policy_joint_ids_[i]];
  }
  array_t result = joint_direction_ * policy_v;
  if (enable_amp_arm_enhance_ && isExternalArmControlActive() &&
      !isVirtualArmObsBlockedByLowCmd() && arm_policy_start_idx_ >= 0) {
    for (size_t i = 0; i < arm_joint_names_.size(); ++i) {
      const int idx = arm_policy_start_idx_ + static_cast<int>(i);
      if (idx >= 0 && idx < policy_joint_count_) {
        result[idx] = 0.0;
      }
    }
  }
  return result;
}

bool GenericRLController::isValidPolicyMotorIdx(int policy_idx) const {
  return policy_idx >= 0 &&
         policy_idx < static_cast<int>(policy_joint_ids_.size());
}

/// 获取缩放后的原始速度命令 [lin_vel_x, lin_vel_y, ang_vel_z]
array_t GenericRLController::getRawVelocityCommands(
    const VelocityCommand& vel_cmd, int cmd_stance_mode) const {
  array_t cmd(3);
  if (cmd_stance_mode == 1) {
    // 姿态模式仅 angular_z 表示下蹲高度；linear_x/y 通道不使用
    cmd[0] = 0.0;
    cmd[1] = 0.0;
    cmd[2] = vel_cmd.angular_z * velocity_scale_angular_z_standing_;
    return cmd;
  }

  if (vel_cmd.linear_x < 0.0) {
    if (velocity_scale_forward_direct_) {
      cmd[0] = vel_cmd.linear_x;
    } else {
      cmd[0] = vel_cmd.linear_x * velocity_scale_linear_x_ *
               velocity_scale_linear_x_negative_;
    }
  } else if (velocity_scale_forward_direct_) {
    cmd[0] = vel_cmd.linear_x;
  } else {
    cmd[0] = vel_cmd.linear_x * velocity_scale_linear_x_;
  }
  cmd[1] = vel_cmd.linear_y * velocity_scale_linear_y_;
  cmd[2] = vel_cmd.angular_z * velocity_scale_angular_z_;
  return cmd;
}

array_t GenericRLController::getRawVelocityCommands() const {
  VelocityCommand vel_cmd;
  int cmd_stance_mode = 0;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    vel_cmd = velocity_cmd_;
    cmd_stance_mode = cmd_stance_mode_;
  }
  return getRawVelocityCommands(vel_cmd, cmd_stance_mode);
}

void GenericRLController::applyStanceHeightStandUpSmoothing(
    double& stance_height_cmd) {
  if (max_standup_change_ <= 0.0) {
    return;
  }

  if (getCmdStanceMode() != 1) {
    smoothed_stance_height_cmd_ = 0.0;
    return;
  }

  const double prev_smoothed = smoothed_stance_height_cmd_;
  const double raw_height = stance_height_cmd;
  const double diff = raw_height - smoothed_stance_height_cmd_;

  bool at_squat_joint_limit = false;
  if (max_squat_knee_angle_ > 0.0 &&
      isValidPolicyMotorIdx(leg_l4_policy_idx_) &&
      isValidPolicyMotorIdx(leg_r4_policy_idx_)) {
    const double knee_l =
        current_state_.q[policy_joint_ids_[leg_l4_policy_idx_]];
    const double knee_r =
        current_state_.q[policy_joint_ids_[leg_r4_policy_idx_]];
    at_squat_joint_limit = knee_l >= max_squat_knee_angle_ ||
                           knee_r >= max_squat_knee_angle_;
    if (!at_squat_joint_limit &&
        last_policy_q_target_.size() ==
            static_cast<size_t>(policy_joint_count_)) {
      at_squat_joint_limit =
          last_policy_q_target_[leg_l4_policy_idx_] >= max_squat_knee_angle_ ||
          last_policy_q_target_[leg_r4_policy_idx_] >= max_squat_knee_angle_;
    }
  }
  const double effective_squat_leg1_angle_max = getEffectiveSquatLeg1AngleMax();
  if (!at_squat_joint_limit && effective_squat_leg1_angle_max > 0.0 &&
      isValidPolicyMotorIdx(leg_l1_policy_idx_) &&
      isValidPolicyMotorIdx(leg_r1_policy_idx_)) {
    const double leg_l1 =
        current_state_.q[policy_joint_ids_[leg_l1_policy_idx_]];
    const double leg_r1 =
        current_state_.q[policy_joint_ids_[leg_r1_policy_idx_]];
    at_squat_joint_limit =
        leg_l1 <= -effective_squat_leg1_angle_max ||
        leg_r1 >= effective_squat_leg1_angle_max;
    if (!at_squat_joint_limit &&
        last_policy_q_target_.size() ==
            static_cast<size_t>(policy_joint_count_)) {
      at_squat_joint_limit =
          last_policy_q_target_[leg_l1_policy_idx_] <=
              -effective_squat_leg1_angle_max ||
          last_policy_q_target_[leg_r1_policy_idx_] >=
              effective_squat_leg1_angle_max;
    }
  }

  if (at_squat_joint_limit && raw_height < prev_smoothed - 1e-9) {
    // 膝角/髋 pitch 已达上限：拒绝更深的下蹲高度命令
    smoothed_stance_height_cmd_ = prev_smoothed;
  } else if (diff > 1e-9) {
    smoothed_stance_height_cmd_ += std::min(diff, max_standup_change_);
  } else {
    smoothed_stance_height_cmd_ = raw_height;
  }

  if (!at_squat_joint_limit && max_stance_squat_depth_ > 0.0 &&
      max_squat_knee_angle_ <= 0.0 && max_squat_leg1_angle_ <= 0.0) {
    smoothed_stance_height_cmd_ =
        std::max(smoothed_stance_height_cmd_, -max_stance_squat_depth_);
  }
  stance_height_cmd = smoothed_stance_height_cmd_;
}

double GenericRLController::getEffectiveSquatLeg1AngleMax() const {
  if (max_squat_leg1_angle_ <= 0.0) {
    return 0.0;
  }
  if (enable_amp_arm_enhance_ && isExternalArmControlActive()) {
    return std::max(0.0, max_squat_leg1_angle_ -
                             kSquatLeg1ExternalArmReductionRad_);
  }
  return max_squat_leg1_angle_;
}

void GenericRLController::applySquatJointAngleLimits(array_t& q_target) const {
  if (getCmdStanceMode() != 1) {
    return;
  }

  if (max_squat_knee_angle_ > 0.0) {
    for (const int policy_idx : {leg_l4_policy_idx_, leg_r4_policy_idx_}) {
      if (!isValidPolicyMotorIdx(policy_idx) ||
          policy_idx < 0 ||
          policy_idx >= static_cast<int>(q_target.size())) {
        continue;
      }
      if (q_target[policy_idx] > max_squat_knee_angle_) {
        q_target[policy_idx] = max_squat_knee_angle_;
      }
    }
  }

  const double effective_squat_leg1_angle_max = getEffectiveSquatLeg1AngleMax();
  if (effective_squat_leg1_angle_max > 0.0) {
    if (isValidPolicyMotorIdx(leg_l1_policy_idx_) &&
        leg_l1_policy_idx_ >= 0 &&
        leg_l1_policy_idx_ < static_cast<int>(q_target.size()) &&
        q_target[leg_l1_policy_idx_] < -effective_squat_leg1_angle_max) {
      q_target[leg_l1_policy_idx_] = -effective_squat_leg1_angle_max;
    }
    if (isValidPolicyMotorIdx(leg_r1_policy_idx_) &&
        leg_r1_policy_idx_ >= 0 &&
        leg_r1_policy_idx_ < static_cast<int>(q_target.size()) &&
        q_target[leg_r1_policy_idx_] > effective_squat_leg1_angle_max) {
      q_target[leg_r1_policy_idx_] = effective_squat_leg1_angle_max;
    }
  }
}

double GenericRLController::smoothLinearXCommand(double target) {
  constexpr double kEpsilon = 1e-9;
  const double previous = smoothed_raw_cmd_x_;
  const bool in_negative_cmd_x = target < -kEpsilon || previous < -kEpsilon;
  const double hold_speed = velocity_smoothing_.cmd_x_decel_hold_speed;
  const bool decelerating =
      std::abs(target) < std::abs(previous) - kEpsilon;

  // 非减速时（加速或速度不变），清除两阶段停车状态，避免残留计时。
  if (!decelerating) {
    cmd_x_decel_in_hold_ = false;
    cmd_x_decel_hold_timer_ = 0.0;
  }

  // 两阶段停车：hold_speed>0 时启用；操作者回中（|target|<=hold_speed）且正在减速时生效。
  // 先快速降到保持速度，保持 hold_time 后缓慢减速到 target(≈0)。前后方向均适用。
  if (hold_speed > 0.0 && decelerating && std::abs(target) <= hold_speed) {
    smoothed_raw_cmd_x_ = applyCmdXTwoPhaseDecel(previous, target);
    return smoothed_raw_cmd_x_;
  }

  if (in_negative_cmd_x) {
    // 后退不使用前进 EMA；加/减速分别单步限幅。
    smoothed_raw_cmd_x_ = target;
    const bool accelerating =
        std::abs(target) > std::abs(previous) + kEpsilon;
    double limit = -1.0;
    if (accelerating) {
      limit = velocity_smoothing_.max_velocity_change_neg_accel_cmd_x;
    } else if (decelerating) {
      limit = velocity_smoothing_.max_velocity_change_neg_cmd_x;
    }
    const double diff = smoothed_raw_cmd_x_ - previous;
    if (limit > 0.0 && std::abs(diff) > limit) {
      smoothed_raw_cmd_x_ = previous + std::copysign(limit, diff);
    }
    return smoothed_raw_cmd_x_;
  }

  if (!velocity_smoothing_.cmd_x_smooth_enabled) {
    smoothed_raw_cmd_x_ = target;
    return smoothed_raw_cmd_x_;
  }

  // 对齐闭源前进逻辑：按固定 policy_dt 计算分段 EMA。
  const double speed_for_tau = std::max(previous, target);
  const double tau =
      speed_for_tau > velocity_smoothing_.cmd_x_decel_ema_tau_threshold
          ? velocity_smoothing_.cmd_x_decel_ema_tau_high
          : velocity_smoothing_.cmd_x_decel_ema_tau_low;
  const double alpha =
      tau > 0.0 ? 1.0 - std::exp(-policy_dt_ / tau) : 1.0;
  smoothed_raw_cmd_x_ = previous + (target - previous) * alpha;

  const bool accelerating = std::abs(target) > std::abs(previous) + kEpsilon;
  double limit = -1.0;
  if (accelerating) {
    limit = velocity_smoothing_.max_velocity_change_cmd_x;
  } else if (decelerating) {
    limit = velocity_smoothing_.max_velocity_change_decel_cmd_x > 0.0
                ? velocity_smoothing_.max_velocity_change_decel_cmd_x
                : velocity_smoothing_.max_velocity_change_cmd_x;
  }

  const double diff = smoothed_raw_cmd_x_ - previous;
  if (limit > 0.0 && std::abs(diff) > limit) {
    smoothed_raw_cmd_x_ = previous + std::copysign(limit, diff);
  }
  return smoothed_raw_cmd_x_;
}

double GenericRLController::applyCmdXTwoPhaseDecel(double previous, double target) {
  constexpr double kEpsilon = 1e-9;
  const double hold_speed = velocity_smoothing_.cmd_x_decel_hold_speed;
  const double hold_time = velocity_smoothing_.cmd_x_decel_hold_time;
  const double sign = previous < 0.0 ? -1.0 : 1.0;
  const double prev_mag = std::abs(previous);

  // 快速段复用本方向加速斜率；缓慢段复用本方向原有减速斜率（后退 0.008 / 前进 0.05）
  const double fast_limit = previous < 0.0
      ? velocity_smoothing_.max_velocity_change_neg_accel_cmd_x
      : velocity_smoothing_.max_velocity_change_cmd_x;
  double slow_limit = previous < 0.0
      ? velocity_smoothing_.max_velocity_change_neg_cmd_x
      : (velocity_smoothing_.max_velocity_change_decel_cmd_x > 0.0
             ? velocity_smoothing_.max_velocity_change_decel_cmd_x
             : velocity_smoothing_.max_velocity_change_cmd_x);
  if (slow_limit <= 0.0) slow_limit = fast_limit;
  if (fast_limit <= 0.0) {
    return previous + std::copysign(slow_limit, target - previous);
  }

  // 阶段1：快速降到保持速度
  if (prev_mag > hold_speed + kEpsilon) {
    double next = previous - sign * fast_limit;
    if (std::abs(next) < hold_speed) {
      next = sign * hold_speed;  // 钳到保持速度，进入保持阶段
      cmd_x_decel_in_hold_ = true;
      cmd_x_decel_hold_timer_ = 0.0;
    } else {
      cmd_x_decel_in_hold_ = false;
    }
    return next;
  }

  // 阶段2：在保持速度上停留 hold_time
  if (cmd_x_decel_in_hold_ && cmd_x_decel_hold_timer_ < hold_time) {
    cmd_x_decel_hold_timer_ += policy_dt_;
    return sign * hold_speed;
  }

  // 阶段3：从保持速度缓慢减速到 target(≈0)
  cmd_x_decel_in_hold_ = false;
  const double diff = target - previous;
  if (std::abs(diff) <= slow_limit) return target;
  return previous + std::copysign(slow_limit, diff);
}

void GenericRLController::updateVelocityCommandCache() {
  VelocityCommand vel_cmd;
  int cmd_stance_mode = 0;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    vel_cmd = velocity_cmd_;
    cmd_stance_mode = cmd_stance_mode_;
  }

  // M1/M2 组合键动作播放期间（走 TactPlayer）：清零所有速度状态（包括 EMA 平滑状态），
  // 确保 filtered_velocity_cmd_ 立即归零，避免 EMA 历史值泄漏到策略观测中
  if (block_velocity_in_motion_) {
    clearVelocityFilterState();
    return;
  }

  double limited_linear_x = vel_cmd.linear_x;
  double limited_linear_y = vel_cmd.linear_y;
  double limited_angular_z = vel_cmd.angular_z;
  if (cmd_stance_mode == 0) {
    limited_linear_x = smoothLinearXCommand(limited_linear_x);
  } else {
    // 姿态模式下 linear_x 不是行走速度，清除行走平滑状态。
    smoothed_raw_cmd_x_ = 0.0;
  }

  if (enable_amp_arm_enhance_ && cmd_stance_mode == 0 &&
      max_velocity_change_cmd_y_ > 0.0) {
    const double y_diff = limited_linear_y - smoothed_raw_cmd_y_;
    const double y_change = std::abs(y_diff);
    if (y_change > max_velocity_change_cmd_y_) {
      limited_linear_y =
          smoothed_raw_cmd_y_ +
          (y_diff > 0.0 ? 1.0 : -1.0) * max_velocity_change_cmd_y_;
    }
    smoothed_raw_cmd_y_ = limited_linear_y;
  } else if (cmd_stance_mode == 0) {
    smoothed_raw_cmd_y_ = limited_linear_y;
  } else {
    smoothed_raw_cmd_y_ = 0.0;
  }

  if (cmd_stance_mode == 0 && max_velocity_change_cmd_angz_ > 0.0) {
    constexpr double kEpsilon = 1e-9;
    const double previous = smoothed_raw_cmd_angz_;
    const bool decelerating =
        std::abs(limited_angular_z) < std::abs(previous) - kEpsilon;
    if (decelerating) {
      const double diff = limited_angular_z - previous;
      if (std::abs(diff) > max_velocity_change_cmd_angz_) {
        limited_angular_z =
            previous + std::copysign(max_velocity_change_cmd_angz_, diff);
      }
    }
    smoothed_raw_cmd_angz_ = limited_angular_z;
  } else if (cmd_stance_mode == 0) {
    smoothed_raw_cmd_angz_ = limited_angular_z;
  } else {
    smoothed_raw_cmd_angz_ = 0.0;
  }

  if (enable_amp_arm_enhance_ && cmd_stance_mode == 0 &&
      external_arm_linear_x_boost_ > 0.0 && limited_linear_x > 0.0 &&
      isExternalArmControlActive()) {
    const double max_forward =
        velocity_scale_linear_x_up_ + external_arm_linear_x_boost_;
    limited_linear_x =
        std::min(limited_linear_x + external_arm_linear_x_boost_, max_forward);
  }

  filtered_velocity_cmd_ = getRawVelocityCommands(vel_cmd, cmd_stance_mode);
  if (cmd_stance_mode == 0) {
    if (limited_linear_x < 0.0) {
      if (velocity_scale_forward_direct_) {
        filtered_velocity_cmd_[0] = limited_linear_x;
      } else {
        filtered_velocity_cmd_[0] = limited_linear_x * velocity_scale_linear_x_ *
                                    velocity_scale_linear_x_negative_;
      }
    } else if (velocity_scale_forward_direct_) {
      filtered_velocity_cmd_[0] = limited_linear_x;
    } else {
      filtered_velocity_cmd_[0] = limited_linear_x * velocity_scale_linear_x_;
    }
    filtered_velocity_cmd_[1] = limited_linear_y * velocity_scale_linear_y_;
    filtered_velocity_cmd_[2] = limited_angular_z * velocity_scale_angular_z_;
  }
  if (cmd_stance_mode == 1) {
    double height_cmd =
        vel_cmd.angular_z * velocity_scale_angular_z_standing_;
    // 死区/回中时目标高度为 0，跳过缓停滤波，避免拖住 smoothed 导致守备迟迟不解除
    if (std::abs(vel_cmd.angular_z) <= 1e-9) {
      angular_z_stop_filter_.reset(0.0);
      height_cmd = 0.0;
    } else {
      height_cmd = angular_z_stop_filter_.update(height_cmd, policy_dt_);
    }
    filtered_velocity_cmd_[2] = height_cmd;
    applyStanceHeightStandUpSmoothing(filtered_velocity_cmd_[2]);
  } else {
    // cmd_stance=0 时 angular_z 是转身角速度；不做缓停滤波，避免松开摇杆后转身缓慢归零。
    angular_z_stop_filter_.reset(filtered_velocity_cmd_[2]);
    smoothed_stance_height_cmd_ = 0.0;
  }
  // enable_off_cmdy_by_cmdx：前后向/侧移互斥，避免斜向走
  if (enable_amp_arm_enhance_) {
    const double abs_cmd_y_before_mutex = std::abs(filtered_velocity_cmd_[1]);
    if (std::abs(filtered_velocity_cmd_[0]) > kRollCompensationCmdXThreshold_) {
      filtered_velocity_cmd_[1] = 0.0;
    }
    if (std::abs(filtered_velocity_cmd_[1]) > kOffCmdxByCmdYThreshold_) {
      filtered_velocity_cmd_[0] = 0.0;
    }
    // off_cmdangz_by_cmdy：行走时转身/侧移互斥
    if (cmd_stance_mode == 0) {
      if (std::abs(filtered_velocity_cmd_[2]) > kOffCmdAngzByCmdYThreshold_) {
        filtered_velocity_cmd_[1] = 0.0;
      }
      if (std::abs(filtered_velocity_cmd_[1]) > kOffCmdAngzByCmdYThreshold_) {
        filtered_velocity_cmd_[2] = 0.0;
      }

      // 近 history_frames 帧内出现过 abs(cmd_y)>threshold，则屏蔽 cmd_x
      if (off_cmdx_after_lateral_.enabled) {
        const bool lateral_active =
            abs_cmd_y_before_mutex > off_cmdx_after_lateral_.cmd_y_threshold;
        lateral_cmd_y_active_history_.push_back(lateral_active);
        while (static_cast<int>(lateral_cmd_y_active_history_.size()) >
               off_cmdx_after_lateral_.history_frames) {
          lateral_cmd_y_active_history_.pop_front();
        }
        for (const bool was_lateral : lateral_cmd_y_active_history_) {
          if (was_lateral) {
            filtered_velocity_cmd_[0] = 0.0;
            break;
          }
        }
      }
    } else if (off_cmdx_after_lateral_.enabled) {
      lateral_cmd_y_active_history_.clear();
    }
  }
  if (mixed_motion_limits_.enable_mixed_mode) {
    applyMixedMotionLimits(filtered_velocity_cmd_);
  }
  if (tiny_cmd_clip_.enabled) {
    applyTinyCmdClip(filtered_velocity_cmd_);
  }

  const double raw_cmd_x_before_kick = filtered_velocity_cmd_[0];
  const double raw_cmd_angz_before_kick = filtered_velocity_cmd_[2];
  if (enable_amp_arm_enhance_) {
    applyReverseYawKickGuard(filtered_velocity_cmd_);
    applyLowSpeedKickStart(filtered_velocity_cmd_);
    applyLowSpeedYawKickStart(filtered_velocity_cmd_);
  }
  prev_raw_cmd_vel_line_x_ = raw_cmd_x_before_kick;
  prev_raw_cmd_vel_angular_z_ = raw_cmd_angz_before_kick;

  applySquatPostureDefense();
  updateStandUpRisingState();
}

void GenericRLController::updateStandUpRisingState() {
  stand_up_rising_active_ = false;

  if (!enable_amp_arm_enhance_ || !enable_standup_enhance_v17_) {
    if (getCmdStanceMode() != 1) {
      prev_stand_up_knee_rad_ = 0.0;
      stand_up_knee_prev_initialized_ = false;
    }
    return;
  }

  if (getCmdStanceMode() != 1) {
    prev_stand_up_knee_rad_ = 0.0;
    stand_up_knee_prev_initialized_ = false;
    return;
  }

  if (!isValidPolicyMotorIdx(leg_l4_policy_idx_) ||
      !isValidPolicyMotorIdx(leg_r4_policy_idx_)) {
    return;
  }

  const double knee_l =
      current_state_.q[policy_joint_ids_[leg_l4_policy_idx_]];
  const double knee_r =
      current_state_.q[policy_joint_ids_[leg_r4_policy_idx_]];
  const double knee = 0.5 * (knee_l + knee_r);

  if (stand_up_knee_prev_initialized_) {
    const double knee_diff = prev_stand_up_knee_rad_ - knee;
    stand_up_rising_active_ =
        (prev_stand_up_knee_rad_ <= kStandUpPitchFullBiasKneeEnd_) &&
        knee_diff > kStandUpKneeDecreasingEpsilon_;
  }
  prev_stand_up_knee_rad_ = knee;
  stand_up_knee_prev_initialized_ = true;
}

void GenericRLController::applySquatPostureDefense() {
  if (!enable_amp_arm_enhance_ || !squat_posture_defense_.enabled ||
      !isValidPolicyMotorIdx(leg_l4_policy_idx_) ||
      !isValidPolicyMotorIdx(leg_r4_policy_idx_) ||
      filtered_velocity_cmd_.size() < 3) {
    return;
  }

  if (getCmdStanceMode() != 1) {
    squat_posture_deep_seen_ = false;
    return;
  }

  const double knee_l =
      current_state_.q[policy_joint_ids_[leg_l4_policy_idx_]];
  const double knee_r =
      current_state_.q[policy_joint_ids_[leg_r4_policy_idx_]];
  const double threshold = squat_posture_defense_.height_threshold;
  const bool knees_deep = knee_l > threshold && knee_r > threshold;
  const bool knees_stood = knee_l < threshold && knee_r < threshold;

  if (knees_deep) {
    squat_posture_deep_seen_ = true;
    filtered_velocity_cmd_[0] = 0.0;
    filtered_velocity_cmd_[1] = 0.0;
  } else if (squat_posture_deep_seen_ && knees_stood) {
    squat_posture_deep_seen_ = false;
    smoothed_stance_height_cmd_ = 0.0;
    smoothed_raw_cmd_x_ = 0.0;
    angular_z_stop_filter_.reset(0.0);
    RL_LOGW(
        "Squat posture defense released: knees stood up (l=%.3f, r=%.3f).",
        knee_l, knee_r);
  }
}

/// 获取当前观测周期的速度命令 [lin_vel_x, lin_vel_y, ang_vel_z]
array_t GenericRLController::getVelocityCommands() const {
  return filtered_velocity_cmd_;
}

bool GenericRLController::isDeepSquatGuardActive() const {
  if (!enable_amp_arm_enhance_ || !squat_posture_defense_.enabled ||
      getCmdStanceMode() != 1 ||
      !isValidPolicyMotorIdx(leg_l4_policy_idx_) ||
      !isValidPolicyMotorIdx(leg_r4_policy_idx_)) {
    return false;
  }

  const double knee_l =
      current_state_.q[policy_joint_ids_[leg_l4_policy_idx_]];
  const double knee_r =
      current_state_.q[policy_joint_ids_[leg_r4_policy_idx_]];
  return knee_l > squat_posture_defense_.height_threshold &&
         knee_r > squat_posture_defense_.height_threshold;
}

/// 获取 IMU 角速度 [gx, gy, gz]
array_t GenericRLController::getBaseAngVel() const {
  array_t result(3);
  result << current_imu_.gyro[0], current_imu_.gyro[1], current_imu_.gyro[2];
  return result;
}

bool GenericRLController::isExternalArmControlActive() const {
  if (!enable_amp_arm_enhance_ || !enable_arm_controller_ || !arm_controller_) {
    return false;
  }
  return arm_controller_->getMode() != ArmControlMode::kAuto;
}

bool GenericRLController::isLateralMoveCommand() const {
  if (getCmdStanceMode() != 0) {
    return false;
  }
  const double cmd_x = filtered_velocity_cmd_[0];
  const double cmd_y = filtered_velocity_cmd_[1];
  const double cmd_angular_z = filtered_velocity_cmd_[2];
  return std::abs(cmd_x) < 0.2 && std::abs(cmd_angular_z) < 0.2 &&
         std::abs(cmd_y) > 0.1;
}

bool GenericRLController::isVirtualArmObsBlockedByLowCmd() const {
  if (getCmdStanceMode() != 0) {
    return false;
  }
  if (std::abs(filtered_velocity_cmd_[0]) < 0.05 &&
      std::abs(filtered_velocity_cmd_[1]) < 0.05) {
    return false;
  }
  return std::abs(filtered_velocity_cmd_[0]) < 0.3 &&
         std::abs(filtered_velocity_cmd_[1]) < 0.15;
}

void GenericRLController::applyAmpV17ActionPostProcess(array_t& actions) const {
  if (!enable_amp_arm_enhance_ || policy_joint_count_ != 21 ||
      actions.size() != static_cast<size_t>(policy_joint_count_)) {
    return;
  }

  // lateral_elbow_fix：纯侧移时缩放对应侧髋/肘 action
  const double cmd_x = filtered_velocity_cmd_[0];
  const double cmd_y = filtered_velocity_cmd_[1];
  const double cmd_angular_z = filtered_velocity_cmd_[2];
  if (isLateralMoveCommand()) {
    // Positive cmd_y is left lateral, negative cmd_y is right lateral.
    if (cmd_y > 0.0) {
      actions[1] *= 0.6;
      actions[2] *= 1.5;
      actions[13] *= 1.2;
      actions[13] += 0.05;
      actions[17] *= 1.2;
      actions[16] *= kLateralElbowFixScale_;
    } else {
      actions[7] *= 0.6;
      actions[8] *= 1.5;
      actions[13] *= 1.2;
      actions[17] *= 1.2;
      actions[17] += 0.05;
      actions[20] *= kLateralElbowFixScale_;
    }

    if (lateral_yaw_compensation_closed_loop_.enabled) {
      Eigen::Quaterniond quat(current_imu_.quat[0], current_imu_.quat[1],
                              current_imu_.quat[2], current_imu_.quat[3]);
      if (quat.squaredNorm() > 1e-9) {
        quat.normalize();
        const double yaw_rad = std::atan2(
            2.0 * (quat.w() * quat.z() + quat.x() * quat.y()),
            1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
        if (!lateral_yaw_compensation_initialized_) {
          lateral_yaw_target_rad_ = yaw_rad;
          lateral_yaw_compensation_initialized_ = true;
        }

        const double yaw_error_rad = std::remainder(
            lateral_yaw_target_rad_ - yaw_rad, 2.0 * M_PI);
        const double max_action =
            lateral_yaw_compensation_closed_loop_.max_action;
        const double action_compensation = std::clamp(
            lateral_yaw_compensation_closed_loop_.kp * yaw_error_rad -
                lateral_yaw_compensation_closed_loop_.kd *
                    current_imu_.gyro[2],
            -max_action, max_action);
        actions[0] -= action_compensation;
      }
    }
  } else {
    lateral_yaw_compensation_initialized_ = false;
  }

  // 原地旋转：leg_l1 / leg_l4 / leg_r1 / leg_r4 action 偏置
  if (enable_turn_in_place_enhance_v17_ && getCmdStanceMode() == 0 &&
      std::abs(cmd_x) <= kInPlaceYawAbsCmdXMax_ &&
      std::abs(cmd_y) <= kInPlaceYawAbsCmdYMax_ &&
      std::abs(cmd_angular_z) >= kInPlaceYawAbsAngZMin_ &&
      isValidPolicyMotorIdx(leg_l1_policy_idx_) &&
      isValidPolicyMotorIdx(leg_l4_policy_idx_) &&
      isValidPolicyMotorIdx(leg_r1_policy_idx_) &&
      isValidPolicyMotorIdx(leg_r4_policy_idx_)) {
    actions[leg_l1_policy_idx_] += kInPlaceYawLegL1ActionBias_;
    actions[leg_l4_policy_idx_] -= kInPlaceYawLegL1ActionBias_;
    actions[leg_r1_policy_idx_] += kInPlaceYawLegR1ActionBias_;
    actions[leg_r4_policy_idx_] += kInPlaceYawLegR1ActionBias_;
  }

  // enable_elbow_scale：肘关节目标角 (default + action*scale) 不得超过 -0.2
  for (const int idx : kElbowActionIndices_) {
    if (idx < 0 || idx >= static_cast<int>(actions.size()) ||
        idx >= default_joint_pos_.size() || idx >= joint_action_scale_.size()) {
      continue;
    }
    const double scale = joint_action_scale_[idx];
    if (scale <= 0.0) {
      continue;
    }
    const double max_action =
        (kElbowTargetUpperBound_ - default_joint_pos_[idx]) / scale;
    if (actions[idx] > max_action) {
      actions[idx] = max_action;
    }
  }
}

void GenericRLController::applyBackArmEnhancePostProcess(array_t& actions) const {
  if (!enable_back_enhance_v17_ || policy_joint_count_ != 21 ||
      actions.size() != static_cast<size_t>(policy_joint_count_) ||
      getCmdStanceMode() != 0) {
    return;
  }

  const double cmd_x = filtered_velocity_cmd_[0];
  if (cmd_x >= kBackArmEnhanceCmdXThreshold_) {
    return;
  }

  const double default_action_scale =
      policy_joint_count_ > 1 ? joint_action_scale_[1] : 0.25;
  const double arm1_scale_ratio =
      default_action_scale > 0.0 ? kBackArmEnhanceScale_ / default_action_scale
                                 : 1.0;

  // kuavo_v17: leg_l4=4, leg_r4=10, zarm_l1/l2/l4=13/14/16, zarm_r1/r2/r4=17/18/20
  actions[13] = (0.15 + actions[10]) * arm1_scale_ratio;
  actions[14] *= 0.5;
  actions[16] += 1.0 - 1.0 * actions[4];
  actions[16] *= arm1_scale_ratio;
  actions[17] = (0.15 + actions[4]) * arm1_scale_ratio;
  actions[18] *= 0.5;
  actions[20] += 1.0 - 1.0 * actions[10];
  actions[20] *= arm1_scale_ratio;
  actions[16] = std::min(0.8, actions[16]);
  actions[20] = std::min(0.8, actions[20]);
}

void GenericRLController::applyStandUpEnhancePostProcess(array_t& actions) const {
  if (!enable_amp_arm_enhance_ || !enable_standup_enhance_v17_ ||
      !stand_up_rising_active_ || getCmdStanceMode() != 1 ||
      policy_joint_count_ != 21 ||
      actions.size() != static_cast<size_t>(policy_joint_count_) ||
      !isValidPolicyMotorIdx(leg_l4_policy_idx_) ||
      !isValidPolicyMotorIdx(leg_r4_policy_idx_) ||
      !isValidPolicyMotorIdx(leg_l1_policy_idx_) ||
      !isValidPolicyMotorIdx(leg_r1_policy_idx_)) {
    return;
  }

  const double knee_l =
      current_state_.q[policy_joint_ids_[leg_l4_policy_idx_]];
  const double knee_r =
      current_state_.q[policy_joint_ids_[leg_r4_policy_idx_]];
  const double knee = 0.5 * (knee_l + knee_r);
  if (knee < kStandUpKneeNoEnhanceRad_ || knee > kStandUpKneeFullEnhanceRad_) {
    return;
  }

  actions[leg_l1_policy_idx_] += kStandUpLeg1ActionBias_;
  actions[leg_r1_policy_idx_] -= kStandUpLeg1ActionBias_;
  actions[leg_l4_policy_idx_] += kStandUpKneeActionBias_;
  actions[leg_r4_policy_idx_] += kStandUpKneeActionBias_;
}

double GenericRLController::applyTinyCmdxClip(double cmd_x) const {
  if (!tiny_cmd_clip_.cmd_x_enabled) {
    return cmd_x;
  }
  if (cmd_x >= tiny_cmd_clip_.cmd_x_pos_min &&
      cmd_x < tiny_cmd_clip_.cmd_x_pos_max) {
    return tiny_cmd_clip_.cmd_x_pos_max;
  }
  return cmd_x;
}

double GenericRLController::applyTinyCmdYClip(double cmd_y) const {
  if (!tiny_cmd_clip_.cmd_y_enabled) {
    return cmd_y;
  }
  const double abs_cmd_y = std::abs(cmd_y);
  if (abs_cmd_y >= tiny_cmd_clip_.cmd_abs_y_min &&
      abs_cmd_y < tiny_cmd_clip_.cmd_abs_y_max) {
    return cmd_y >= 0.0 ? tiny_cmd_clip_.cmd_abs_y_max
                        : -tiny_cmd_clip_.cmd_abs_y_max;
  }
  return cmd_y;
}

void GenericRLController::applyMixedMotionLimits(array_t& velocity_cmd) const {
  if (!mixed_motion_limits_.enable_mixed_mode || velocity_cmd.size() < 3) {
    return;
  }

  const double angular_z_mag = std::abs(velocity_cmd[2]);
  const double linear_xy_mag =
      std::sqrt(velocity_cmd[0] * velocity_cmd[0] +
                velocity_cmd[1] * velocity_cmd[1]);

  if (angular_z_mag > mixed_motion_limits_.angular_vel_threshold &&
      linear_xy_mag > mixed_motion_limits_.max_linear_vel_with_angular) {
    const double scale_factor =
        mixed_motion_limits_.max_linear_vel_with_angular / linear_xy_mag;
    velocity_cmd[0] *= scale_factor;
    velocity_cmd[1] *= scale_factor;
  }

  if (linear_xy_mag > mixed_motion_limits_.linear_vel_threshold &&
      angular_z_mag > mixed_motion_limits_.max_angular_vel_with_linear) {
    const double scale_factor =
        mixed_motion_limits_.max_angular_vel_with_linear / angular_z_mag;
    velocity_cmd[2] *= scale_factor;
  }
}

void GenericRLController::applyTinyCmdClip(array_t& velocity_cmd) const {
  if (!tiny_cmd_clip_.enabled || velocity_cmd.size() < 3) {
    return;
  }

  velocity_cmd[0] = applyTinyCmdxClip(velocity_cmd[0]);
  velocity_cmd[1] = applyTinyCmdYClip(velocity_cmd[1]);
}

void GenericRLController::applyLowSpeedKickStart(array_t& velocity_cmd) {
  if (!low_speed_kick_start_.enabled || velocity_cmd.size() < 3) {
    return;
  }

  const auto cancel_kick = [this]() { low_speed_kick_remaining_steps_ = 0; };

  static constexpr double kKickAccelEpsilon = 1e-4;
  const double cmd_x = velocity_cmd[0];
  const double abs_cmd_x = std::abs(cmd_x);
  const double abs_prev_cmd_x = std::abs(prev_raw_cmd_vel_line_x_);
  if (abs_cmd_x + kKickAccelEpsilon < abs_prev_cmd_x) {
    cancel_kick();
    return;
  }

  if (getCmdStanceMode() != 0) {
    cancel_kick();
    return;
  }

  const bool in_kick_trigger_range =
      std::abs(cmd_x - low_speed_kick_start_.trigger_velocity) <=
          low_speed_kick_start_.trigger_tolerance &&
      std::abs(velocity_cmd[1]) < low_speed_kick_start_.lateral_threshold &&
      std::abs(velocity_cmd[2]) < low_speed_kick_start_.yaw_threshold;

  if (!in_kick_trigger_range) {
    cancel_kick();
    return;
  }

  if (low_speed_kick_remaining_steps_ > 0) {
    velocity_cmd[0] = low_speed_kick_start_.kick_velocity;
    --low_speed_kick_remaining_steps_;
    return;
  }

  if (abs_prev_cmd_x < low_speed_kick_start_.rest_cmd_threshold &&
      abs_cmd_x > abs_prev_cmd_x + kKickAccelEpsilon) {
    velocity_cmd[0] = low_speed_kick_start_.kick_velocity;
    low_speed_kick_remaining_steps_ =
        low_speed_kick_start_.duration_steps - 1;
  }
}

void GenericRLController::applyReverseYawKickGuard(array_t& velocity_cmd) {
  if (!low_speed_yaw_kick_start_.enable_reverse_kick_guard ||
      velocity_cmd.size() < 3) {
    return;
  }

  if (getCmdStanceMode() != 0) {
    yaw_cmd_angz_history_.clear();
    return;
  }

  static constexpr double kSignEpsilon = 1e-9;
  const double current_angz = velocity_cmd[2];

  if (static_cast<int>(yaw_cmd_angz_history_.size()) >=
      kReverseYawKickGuardHistorySteps_) {
    const double past_angz = yaw_cmd_angz_history_.front();
    if (std::abs(past_angz) > kSignEpsilon &&
        std::abs(current_angz) > kSignEpsilon &&
        past_angz * current_angz < -kSignEpsilon) {
      velocity_cmd[2] = 0.0;
      low_speed_yaw_kick_remaining_steps_ = 0;
    }
  }

  yaw_cmd_angz_history_.push_back(current_angz);
  while (static_cast<int>(yaw_cmd_angz_history_.size()) >
         kReverseYawKickGuardHistorySteps_) {
    yaw_cmd_angz_history_.pop_front();
  }
}

void GenericRLController::applyLowSpeedYawKickStart(array_t& velocity_cmd) {
  if (!low_speed_yaw_kick_start_.enabled || velocity_cmd.size() < 3) {
    return;
  }

  const auto cancel_kick = [this]() { low_speed_yaw_kick_remaining_steps_ = 0; };

  static constexpr double kKickAccelEpsilon = 1e-4;
  const double cmd_angz = velocity_cmd[2];
  const double abs_angz = std::abs(cmd_angz);
  const double abs_prev_angz = std::abs(prev_raw_cmd_vel_angular_z_);
  if (abs_angz + kKickAccelEpsilon < abs_prev_angz) {
    cancel_kick();
    return;
  }

  if (getCmdStanceMode() != 0) {
    cancel_kick();
    return;
  }

  const bool in_kick_trigger_range =
      std::abs(abs_angz - low_speed_yaw_kick_start_.trigger_angular_velocity) <=
          low_speed_yaw_kick_start_.trigger_tolerance &&
      std::abs(velocity_cmd[0]) <
          low_speed_yaw_kick_start_.forward_threshold &&
      std::abs(velocity_cmd[1]) <
          low_speed_yaw_kick_start_.lateral_threshold;

  if (!in_kick_trigger_range) {
    cancel_kick();
    return;
  }

  if (low_speed_yaw_kick_remaining_steps_ > 0) {
    velocity_cmd[2] =
        low_speed_yaw_kick_sign_ * low_speed_yaw_kick_start_.kick_angular_velocity;
    --low_speed_yaw_kick_remaining_steps_;
    return;
  }

  if (abs_prev_angz < low_speed_yaw_kick_start_.rest_cmd_threshold &&
      abs_angz > abs_prev_angz + kKickAccelEpsilon) {
    low_speed_yaw_kick_sign_ = cmd_angz >= 0.0 ? 1.0 : -1.0;
    velocity_cmd[2] =
        low_speed_yaw_kick_sign_ * low_speed_yaw_kick_start_.kick_angular_velocity;
    low_speed_yaw_kick_remaining_steps_ =
        low_speed_yaw_kick_start_.duration_steps - 1;
  }
}

/// 获取投影重力向量：将 [0,0,-1] 从世界坐标系旋转到体坐标系
array_t GenericRLController::getProjectedGravity() const {
  Eigen::Vector3d g_hat(0., 0., -1.);
  Eigen::Quaterniond quat(current_imu_.quat[0], current_imu_.quat[1],
                          current_imu_.quat[2], current_imu_.quat[3]);
  Eigen::Vector3d projected_gravity = quat.inverse() * g_hat;

  const double cmd_x = filtered_velocity_cmd_[0];
  const double cmd_angular_z = filtered_velocity_cmd_[2];
  const double pitch_cmd_x = getRawVelocityCommands()[0];

  // use_virtual_arm_obs：外部手臂接管时对 projected_gravity 做 pitch 补偿（含下蹲）
  // （|cmd_x|&|cmd_y|<0.2 时跳过，仅行走态 cmd_stance=0）
  if (enable_amp_arm_enhance_ && isExternalArmControlActive() &&
      !isVirtualArmObsBlockedByLowCmd()) {
    const double virtual_arm_obs_pitch_scale =
        pitch_cmd_x >= -0.12 ? pitch_cmd_x : -0.1;
    double compensation_pitch_deg =
        virtual_arm_obs_pitch_scale >= -0.005
            ? kVirtualArmObsPitchBaseDeg_ +
                  kVirtualArmObsPitchCompensationDeg_ * virtual_arm_obs_pitch_scale
            : kVirtualArmObsPitchBaseDegNeg_ +
                  kVirtualArmObsPitchCompensationDegNeg_ * virtual_arm_obs_pitch_scale;

    // 平滑 cmd_x 减速时削弱补偿（补偿量仍按 pitch_cmd_x 计算），覆盖整个 EMA 减速过程
    constexpr double kCmdEpsilon = 1e-9;
    if (cmd_x >= -0.005 &&
        prev_virtual_arm_obs_filtered_cmd_x_ > kCmdEpsilon &&
        cmd_x < prev_virtual_arm_obs_filtered_cmd_x_ - kCmdEpsilon) {
      compensation_pitch_deg =
          std::max(-0.2, compensation_pitch_deg - kVirtualArmObsPitchDecelReductionDeg_);
    }

    // 手臂后摆(zarm_l1+zarm_r1)越大，越削弱 projected_gravity 后仰补偿
    if (arm_policy_start_idx_ >= 0 && arm_joint_names_.size() > 4 &&
        policy_joint_count_ > arm_policy_start_idx_ + 4) {
      const int zarm_l1_idx = arm_policy_start_idx_;
      const int zarm_r1_idx = arm_policy_start_idx_ + 4;
      const double zarm_l1_q =
          joint_direction_[zarm_l1_idx] *
          current_state_.q[policy_joint_ids_[zarm_l1_idx]];
      const double zarm_r1_q =
          joint_direction_[zarm_r1_idx] *
          current_state_.q[policy_joint_ids_[zarm_r1_idx]];
      const double zarm_l1_back =
          std::max(0.0, zarm_l1_q - default_joint_pos_[zarm_l1_idx]);
      const double zarm_r1_back =
          std::max(0.0, zarm_r1_q - default_joint_pos_[zarm_r1_idx]);
      const double arm1_back_sum = std::clamp(
          zarm_l1_back + zarm_r1_back, 0.0, kVirtualArmObsArm1BackSumMaxRad_);
      compensation_pitch_deg -=
          kVirtualArmObsArm1BackPitchReductionMaxDeg_ *
          std::min(arm1_back_sum, kVirtualArmObsArm1BackSumPitchReductionFullRad_) /
          kVirtualArmObsArm1BackSumPitchReductionFullRad_;

      const double zarm_l1_back_vel = std::max(
          0.0, joint_direction_[zarm_l1_idx] *
                   current_state_.v[policy_joint_ids_[zarm_l1_idx]]);
      const double zarm_r1_back_vel = std::max(
          0.0, joint_direction_[zarm_r1_idx] *
                   current_state_.v[policy_joint_ids_[zarm_r1_idx]]);
      const double arm1_back_vel_sum = zarm_l1_back_vel + zarm_r1_back_vel;
      compensation_pitch_deg -=
          kVirtualArmObsArm1BackVelPitchReductionMaxDeg_ *
          std::min(arm1_back_vel_sum,
                   kVirtualArmObsArm1BackVelSumPitchReductionFullRadPerSec_) /
          kVirtualArmObsArm1BackVelSumPitchReductionFullRadPerSec_;

      if (arm_joint_names_.size() > 7 &&
          policy_joint_count_ > arm_policy_start_idx_ + 7) {
        const int zarm_l4_idx = arm_policy_start_idx_ + 3;
        const int zarm_r4_idx = arm_policy_start_idx_ + 7;
        const double l_1 = zarm_l1_q;
        const double l_4 =
            joint_direction_[zarm_l4_idx] *
            current_state_.q[policy_joint_ids_[zarm_l4_idx]];
        const double r_1 = zarm_r1_q;
        const double r_4 =
            joint_direction_[zarm_r4_idx] *
            current_state_.q[policy_joint_ids_[zarm_r4_idx]];
        const double arm_forward_length = std::max(
            0.0,
            std::max(-std::sin(l_1) - std::sin(r_1),
                     -std::sin(l_1 + l_4) - std::sin(l_1) - std::sin(r_1 + r_4) -
                         std::sin(r_1)));
        compensation_pitch_deg +=
            kVirtualArmObsPitchArmForwardScale_ * arm_forward_length;
      }
    }

    const double compensation_pitch_rad = compensation_pitch_deg * M_PI / 180.0;
    projected_gravity =
        Eigen::AngleAxisd(-compensation_pitch_rad, Eigen::Vector3d::UnitY()) *
        projected_gravity;
  }

  prev_virtual_arm_obs_filtered_cmd_x_ = cmd_x;

  // enable_roll_compensation_closed_loop：直行时对 projected_gravity 做 roll 观测闭环补偿
  double total_roll_compensation_deg = 0.0;
  const bool is_walking_mode = getCmdStanceMode() == 0;
  if (enable_amp_arm_enhance_ && roll_compensation_closed_loop_.enabled) {
    const Eigen::Matrix3d sensor_rotation = quat.toRotationMatrix();
    const double measured_roll_rad =
        std::atan2(sensor_rotation(2, 1), sensor_rotation(2, 2));
    const double observation_dt = policy_dt_;

    if (!roll_compensation_closed_loop_initialized_) {
      roll_compensation_filtered_roll_rad_ = measured_roll_rad;
      const double abs_measured_roll_rad = std::abs(measured_roll_rad);
      const double first_frame_max_abs_roll_rad =
          roll_compensation_closed_loop_.first_frame_max_abs_roll_deg * M_PI /
          180.0;
      const double first_frame_half_max_abs_roll_rad =
          first_frame_max_abs_roll_rad * 0.5;
      if (first_frame_max_abs_roll_rad <= 1e-9) {
        roll_compensation_target_roll_rad_ = measured_roll_rad;
      } else if (abs_measured_roll_rad > first_frame_max_abs_roll_rad) {
        roll_compensation_target_roll_rad_ = 0.0;
      } else if (abs_measured_roll_rad > first_frame_half_max_abs_roll_rad) {
        roll_compensation_target_roll_rad_ = measured_roll_rad * 0.5;
      } else {
        roll_compensation_target_roll_rad_ = measured_roll_rad;
      }
      roll_compensation_integral_rad_sec_ = 0.0;
      roll_compensation_closed_loop_initialized_ = true;
      RL_LOGI("Roll zero frozen: target_roll=%.4f deg (|roll|=%.4f deg, limit=%.2f deg)",
              roll_compensation_target_roll_rad_ * 180.0 / M_PI,
              std::abs(measured_roll_rad) * 180.0 / M_PI,
              roll_compensation_closed_loop_.first_frame_max_abs_roll_deg);
    }

    const double filter_alpha = std::exp(
        -observation_dt /
        roll_compensation_closed_loop_.filter_time_constant_sec);
    roll_compensation_filtered_roll_rad_ =
        filter_alpha * roll_compensation_filtered_roll_rad_ +
        (1.0 - filter_alpha) * measured_roll_rad;

    const bool external_arm_blocks_closed_loop =
        roll_compensation_closed_loop_.unable_compensation_arm_controller &&
        isExternalArmControlActive();
    if (external_arm_blocks_closed_loop) {
      // 外部手臂(tact/VR)：禁用闭环输出；积分清零、filter 继续跟踪，target 保持 tact 前
      roll_compensation_integral_rad_sec_ = 0.0;
    }

    const bool closed_loop_active =
        !external_arm_blocks_closed_loop && is_walking_mode &&
        cmd_x > roll_compensation_closed_loop_.cmd_x_min &&
        cmd_x < roll_compensation_closed_loop_.cmd_x_max &&
        std::abs(filtered_velocity_cmd_[1]) <
            roll_compensation_closed_loop_.abs_cmd_y_max &&
        std::abs(cmd_angular_z) <
            roll_compensation_closed_loop_.abs_cmd_ang_z_max;
    if (closed_loop_active) {
      const double roll_error_rad = roll_compensation_target_roll_rad_ -
                                    roll_compensation_filtered_roll_rad_;
      const double max_compensation_rad =
          roll_compensation_closed_loop_.max_compensation_deg * M_PI / 180.0;
      if (roll_compensation_closed_loop_.ki > 1e-9) {
        const double integral_limit =
            max_compensation_rad / roll_compensation_closed_loop_.ki;
        roll_compensation_integral_rad_sec_ = std::clamp(
            roll_compensation_integral_rad_sec_ +
                roll_error_rad * observation_dt,
            -integral_limit, integral_limit);
      } else {
        roll_compensation_integral_rad_sec_ = 0.0;
      }

      const double closed_loop_compensation_rad = std::clamp(
          roll_compensation_closed_loop_.kp * roll_error_rad +
              roll_compensation_closed_loop_.ki *
                  roll_compensation_integral_rad_sec_,
          -max_compensation_rad, max_compensation_rad);
      total_roll_compensation_deg +=
          closed_loop_compensation_rad * 180.0 / M_PI;
    }
  }

  // 走弧线时对 projected_gravity 做 roll 开环补偿（与直行闭环互补）
  if (enable_amp_arm_enhance_ && is_walking_mode &&
      velocity_cmd_.linear_x > kTurnRollCompensationCmdXMin_ &&
      std::abs(velocity_cmd_.angular_z) > kTurnRollCompensationAbsAngZMin_) {
    double turn_roll_compensation_deg =
        kTurnRollCompensationDeg_ * velocity_cmd_.angular_z;
    if (isExternalArmControlActive()) {
      turn_roll_compensation_deg *= kTurnRollCompensationExternalArmScale_;
    }
    total_roll_compensation_deg += turn_roll_compensation_deg;
  }

  if (std::abs(total_roll_compensation_deg) > 1e-6) {
    const double compensation_roll_rad =
        total_roll_compensation_deg * M_PI / 180.0;
    projected_gravity =
        Eigen::AngleAxisd(compensation_roll_rad, Eigen::Vector3d::UnitX()) *
        projected_gravity;
  }

  // 走弧线 pitch 开环补偿：cmd_x>0.3、|cmd_angz|>0.3；external arm 时 ×2
  if (enable_amp_arm_enhance_ && is_walking_mode &&
      cmd_x > kTurnPitchCompensationCmdXMin_ &&
      std::abs(cmd_angular_z) > kTurnPitchCompensationAbsAngZMin_) {
    double turn_pitch_compensation_deg =
        kTurnPitchCompensationDeg_ * std::abs(velocity_cmd_.angular_z);
    if (isExternalArmControlActive()) {
      turn_pitch_compensation_deg *= kTurnPitchCompensationExternalArmScale_;
    }
    const double compensation_pitch_rad =
        turn_pitch_compensation_deg * M_PI / 180.0;
    projected_gravity =
        Eigen::AngleAxisd(compensation_pitch_rad, Eigen::Vector3d::UnitY()) *
        projected_gravity;
  }

  if (enable_back_enhance_v17_ && getCmdStanceMode() == 0 &&
      cmd_x >= kBackArmEnhanceGravityCmdXMin_ &&
      cmd_x <= kBackArmEnhanceGravityCmdXMax_) {
    const double compensation_pitch_rad =
        kBackArmEnhanceGravityPitchDeg_ * M_PI / 180.0;
    projected_gravity =
        Eigen::AngleAxisd(compensation_pitch_rad, Eigen::Vector3d::UnitY()) *
        projected_gravity;
  }

  if (enable_back_enhance_v17_ && getCmdStanceMode() == 0 &&
      cmd_x >=  0.02 &&
      cmd_x <= 0.15) {
    const double compensation_pitch_rad = isExternalArmControlActive()
        ? 0.35 * M_PI / 180.0
        : 0.1 * M_PI / 180.0;
    projected_gravity =
        Eigen::AngleAxisd(compensation_pitch_rad, Eigen::Vector3d::UnitY()) *
        projected_gravity;
  }

  if (enable_amp_arm_enhance_ && enable_standup_enhance_v17_ &&
      stand_up_rising_active_ &&
      isValidPolicyMotorIdx(leg_l4_policy_idx_) &&
      isValidPolicyMotorIdx(leg_r4_policy_idx_)) {
    const double knee_l =
        current_state_.q[policy_joint_ids_[leg_l4_policy_idx_]];
    const double knee_r =
        current_state_.q[policy_joint_ids_[leg_r4_policy_idx_]];
    const double knee = 0.5 * (knee_l + knee_r);
    double pitch_weight = 0.0;
    if (knee <= kStandUpPitchFullBiasKneeEnd_ &&
        knee >= kStandUpPitchFadeKneeStart_) {
      pitch_weight = 1.0;
    } else if (knee > kStandUpPitchFullBiasKneeEnd_) {
      pitch_weight = 0.0;
    } else if (knee > 0.0) {
      pitch_weight =
          std::clamp(knee / kStandUpPitchFadeKneeStart_, 0.0, 1.0);
    }
    const double pitch_deg = kStandUpGravityPitchBiasDeg_ * pitch_weight;
    if (std::abs(pitch_deg) > 1e-6) {
      projected_gravity =
          Eigen::AngleAxisd(pitch_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
          projected_gravity;
    }
  }

  return projected_gravity.array();
}

/// 获取 motion 目标姿态相对当前基座的旋转矩阵前两列（6 维）
array_t GenericRLController::getMotionAnchorOriB() const {
  array_t result(6);
  result.setZero();

  auto* motion = getCurrentMotion();
  if (!motion) {
    return result;
  }

  Eigen::Quaterniond target_quat = motion->getBodyQuat();
  Eigen::Quaterniond base_quat(current_imu_.quat[0], current_imu_.quat[1],
                                current_imu_.quat[2], current_imu_.quat[3]);

  // R = (world_yaw^-1 * base)^-1 * target
  auto dummy_world_quat = Eigen::AngleAxisd(dummy_world_yaw_, Eigen::Vector3d{0., 0., 1.});
  auto R_BaseTarget = ((dummy_world_quat.inverse() * base_quat).inverse() * target_quat).matrix();

  result << R_BaseTarget(0, 0), R_BaseTarget(0, 1),
            R_BaseTarget(1, 0), R_BaseTarget(1, 1),
            R_BaseTarget(2, 0), R_BaseTarget(2, 1);

  return result;
}

/// 根据名称分发计算对应的观测项
array_t GenericRLController::getObsTerm(const std::string& name) const {
  if (name == "base_ang_vel") {
    return getBaseAngVel();
  } else if (name == "projected_gravity") {
    return getProjectedGravity();
  } else if (name == "joint_pos") {
    return getPolicyJointPos();
  } else if (name == "joint_vel") {
    return getPolicyJointVel();
  } else if (name == "actions") {
    return getActionsSnapshot();
  } else if (name == "velocity_commands") {
    return getVelocityCommands();
  } else if (name == "motion_command") {
    // motion 参考轨迹的 joint_pos + joint_vel（2N 维）
    auto* motion = getCurrentMotion();
    if (!motion) {
      return array_t::Zero(policy_joint_count_ * 2);
    }
    return motion->getCurrentCommand();
  } else if (name == "motion_target_height") {
    // motion 参考轨迹的目标体高（body_pos[z]）
    auto* motion = getCurrentMotion();
    if (!motion) {
      return array_t::Zero(1);
    }
    return array_t::Constant(1, motion->getBodyPos()[2]);
  } else if (name == "motion_anchor_ori_b") {
    return getMotionAnchorOriB();
  } else if (name == "cmd_stance") {
    if (enable_amp_arm_enhance_) {
      // v17：手动 cmd_stance_mode，0=行走/站立，1=下蹲
      return array_t::Constant(1, getCmdStanceValue());
    }
    // v46/v52：基于滤波后速度指令的帧防抖，与原版 lejulab 一致
    const bool is_stop =
        filtered_velocity_cmd_.size() >= 3 &&
        std::abs(filtered_velocity_cmd_[0]) < 0.01 &&
        std::abs(filtered_velocity_cmd_[1]) < 0.01 &&
        std::abs(filtered_velocity_cmd_[2]) < 0.01;
    if (is_stop) {
      stance_debounce_counter_++;
    } else {
      stance_debounce_counter_ = 0;
    }
    const double stance =
        (stance_debounce_counter_ >= stance_debounce_threshold_) ? 1.0 : 0.0;
    return array_t::Constant(1, stance);
  }

  RL_LOG_WARNING("Unknown obs term: %s", name.c_str());
  return array_t();
}

void GenericRLController::startInferenceThread() {
  stopInferenceThread();
  inference_stop_requested_ = false;
  inference_thread_running_ = true;
  inference_thread_ = std::thread(&GenericRLController::inferenceThreadLoop, this);
}

void GenericRLController::stopInferenceThread() {
  {
    std::lock_guard<std::mutex> lock(inference_mutex_);
    inference_stop_requested_ = true;
    has_pending_observation_ = false;
  }
  inference_cv_.notify_all();
  if (inference_thread_.joinable()) {
    inference_thread_.join();
  }
  inference_thread_running_ = false;
}

void GenericRLController::submitObservationForInference(const array_t& observation) {
  last_observation_submit_time_sec_.store(common::GetSteadyTimestampNs() * 1e-9);
  {
    std::lock_guard<std::mutex> lock(inference_mutex_);
    pending_observation_ = observation;
    has_pending_observation_ = true;
  }
  inference_cv_.notify_one();
}

array_t GenericRLController::getActionsSnapshot() const {
  std::lock_guard<std::mutex> lock(action_mutex_);
  return actions_;
}

void GenericRLController::updateActionCache(const array_t& new_actions) {
  array_t processed = new_actions;
  if (processed.size() == static_cast<size_t>(policy_joint_count_)) {
    if (enable_amp_arm_enhance_) {
      applyAmpV17ActionPostProcess(processed);
    }
    if (enable_back_enhance_v17_) {
      applyBackArmEnhancePostProcess(processed);
    }
    if (enable_standup_enhance_v17_) {
      applyStandUpEnhancePostProcess(processed);
    }
  }
  std::lock_guard<std::mutex> lock(action_mutex_);
  last_actions_ = actions_;
  actions_ = std::move(processed);
}

void GenericRLController::clearInferenceTimestamps() {
  last_observation_submit_time_sec_.store(0.0);
  last_inference_finish_time_sec_.store(0.0);
  last_inference_frequency_sample_time_sec_.store(0.0);
}

void GenericRLController::publishInferenceFrequency(double finish_time_sec) {
  const double last_finish_time =
      last_inference_frequency_sample_time_sec_.exchange(finish_time_sec);
  const double elapsed_sec = finish_time_sec - last_finish_time;
  if (!logger_ || last_finish_time <= 0.0 || elapsed_sec <= 0.0) {
    return;
  }

  logger_->publishValue("/" + name_ + "/policy_inference_frequency_hz",
                        1.0 / elapsed_sec);
}

bool GenericRLController::inferActions(const array_t& observation, array_t& inferred_actions) {
  InferenceModel* active_model = nullptr;
  if (dual_fall_stand_) {
    std::lock_guard<std::mutex> lock(inference_mutex_);
    active_model = (fall_stand_model_type_ == FallStandModelType::kProne)
                       ? model_prone_.get()
                       : model_supine_.get();
  } else {
    active_model = model_.get();
  }

  if (!active_model || !active_model->isLoaded()) {
    return false;
  }

  std::vector<float> obs_vec(observation.size());
  for (int i = 0; i < observation.size(); ++i) {
    obs_vec[i] = static_cast<float>(observation[i]);
  }

  std::vector<float> action_vec = active_model->forward(obs_vec);
  if (action_vec.size() != static_cast<size_t>(policy_joint_count_)) {
    RL_LOG_WARNING("Action size mismatch: got %zu, expected %d",
                   action_vec.size(), policy_joint_count_);
    return false;
  }

  inferred_actions.resize(policy_joint_count_);
  for (int i = 0; i < policy_joint_count_; ++i) {
    inferred_actions[i] = static_cast<double>(action_vec[i]);
  }
  return true;
}

void GenericRLController::inferenceThreadLoop() {
  // 推理线程早于 main 的主线程绑核，需要单独绑定 RK3588 大核。
  if (leju::cpu::bindCurrentThreadToBigCores()) {
    RL_LOGI("Inference thread bound to big cores %d-%d, current CPU=%d",
            leju::cpu::kRk3588BigCoreFirst, leju::cpu::kRk3588BigCoreLast,
            leju::cpu::getCurrentCpu());
  } else {
    RL_LOGW("Failed to bind inference thread to big cores");
  }

  while (true) {
    array_t observation;
    {
      std::unique_lock<std::mutex> lock(inference_mutex_);
      inference_cv_.wait(lock, [this] {
        return inference_stop_requested_ || has_pending_observation_;
      });

      if (inference_stop_requested_) {
        break;
      }

      observation = pending_observation_;
      has_pending_observation_ = false;
    }

    array_t inferred_actions;
    if (inferActions(observation, inferred_actions)) {
      updateActionCache(inferred_actions);
      const double finish_time_sec = common::GetSteadyTimestampNs() * 1e-9;
      last_inference_finish_time_sec_.store(finish_time_sec);
      publishInferenceFrequency(finish_time_sec);
    }
  }
}

/// 返回观测项维度
int GenericRLController::getObsTermShape(const std::string& name) const {
  if (name == "base_ang_vel" || name == "projected_gravity" || name == "velocity_commands") {
    return 3;
  } else if (name == "joint_pos" || name == "joint_vel" || name == "actions") {
    return policy_joint_count_;
  } else if (name == "motion_command") {
    return policy_joint_count_ * 2;
  } else if (name == "motion_target_height") {
    return 1;
  } else if (name == "motion_anchor_ori_b") {
    return 6;
  } else if (name == "cmd_stance") {
    return 1;
  }
  RL_LOG_WARNING("Unknown obs term shape: %s", name.c_str());
  return 0;
}

// ============================================================================
// 观测历史管理
// ============================================================================

/// 将 obs_stacks_ 中所有元素清零（保持结构不变）
void GenericRLController::resetObsHistory() {
  for (auto& stack : obs_stacks_) {
    for (auto& term : stack) {
      term.setZero();
    }
  }
  observations_.setZero();
}

/// 根据 obs_terms_ 和 history_length 初始化 obs_stacks_ 的 2D deque 结构
void GenericRLController::initObsHistory() {
  // 计算单帧观测维度
  int single_obs_size = 0;
  for (const auto& term : obs_terms_) {
    single_obs_size += getObsTermShape(term.name);
  }
  int obs_size = single_obs_size * obs_history_length_;

  obs_stacks_.clear();
  if (obs_stack_order_ == StackOrder::kIsaaclab) {
    // isaaclab: obs_stacks_[term_index][time_index]
    obs_stacks_.resize(obs_terms_.size());
    for (size_t i = 0; i < obs_terms_.size(); ++i) {
      int shape = getObsTermShape(obs_terms_[i].name);
      obs_stacks_[i].resize(obs_history_length_);
      for (int j = 0; j < obs_history_length_; ++j) {
        obs_stacks_[i][j] = array_t::Zero(shape);
      }
    }
  } else {
    // classic: obs_stacks_[time_index][term_index]
    obs_stacks_.resize(obs_history_length_);
    for (int i = 0; i < obs_history_length_; ++i) {
      obs_stacks_[i].resize(obs_terms_.size());
      for (size_t j = 0; j < obs_terms_.size(); ++j) {
        int shape = getObsTermShape(obs_terms_[j].name);
        obs_stacks_[i][j] = array_t::Zero(shape);
      }
    }
  }

  observations_.resize(obs_size);
  observations_.setZero();
}

// ============================================================================
// IMU / 世界坐标系初始化
// ============================================================================

/// 根据首帧 IMU 和 motion 目标姿态计算世界坐标系 yaw 偏移
void GenericRLController::initializeDummyWorldYaw() {
  dummy_world_yaw_ = 0.0;

  auto* motion = getCurrentMotion();
  if (!motion) {
    RL_LOG_WARNING("No motion loader available, using dummy_world_yaw=0");
    return;
  }

  Eigen::Quaterniond target_quat = motion->getBodyQuat();
  Eigen::Quaterniond base_quat(current_imu_.quat[0], current_imu_.quat[1],
                               current_imu_.quat[2], current_imu_.quat[3]);

  // 提取 yaw 分量（与 rl_mimic_controller 一致）
  Eigen::AngleAxisd R_BaseTarget(target_quat * base_quat.inverse());
  if (R_BaseTarget.axis()(2) > 0.0) {
    dummy_world_yaw_ = -R_BaseTarget.angle();
  } else {
    dummy_world_yaw_ = R_BaseTarget.angle();
  }

  // 旋转轴 z 分量过小说明存在 pitch/roll 偏差
  if (std::abs(R_BaseTarget.axis()(2)) < 0.7) {
    RL_LOG_WARNING("Detected initial base pitch or roll deviation");
  }

  RL_LOG_INFO("Initialized dummy_world_yaw: %.3f rad", dummy_world_yaw_);
}

// 注册控制器类型
REGISTER_CONTROLLER("GenericRLController", GenericRLController, generic_rl);

}  // namespace leju
