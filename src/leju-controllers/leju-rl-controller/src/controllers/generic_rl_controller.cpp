#include "leju-rl-controller/controllers/generic_rl_controller.h"

#include <Eigen/Geometry>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

#include "leju-rl-controller/inference/openvino_model.h"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-utils/time_utils.hpp"

namespace leju {

// ============================================================================
// 构造 / 析构
// ============================================================================

GenericRLController::GenericRLController(const RobotVersion& version,
                                         const std::string& name)
    : robot_version_(version) {
  name_ = name;
}

GenericRLController::~GenericRLController() = default;

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

    std::filesystem::path config_dir = std::filesystem::path(config_path_).parent_path();

    // 3. 加载所有 motion 轨迹文件
    loadMotionTrajectories(config_dir.string());

    // 4. 初始化观测历史缓冲区（obs_stacks_ 和 observations_）
    initObsHistory();

    // 5. 加载 ONNX 策略模型
    std::string policy_full_path = (config_dir / policy_path_).string();
    if (!loadPolicy(policy_full_path)) {
      RL_LOG_FAILURE("Failed to load policy: %s", policy_full_path.c_str());
      return false;
    }

    // 6. 初始化动作向量
    actions_.resize(policy_joint_count_);
    actions_.setZero();
    last_actions_.resize(policy_joint_count_);
    last_actions_.setZero();

    // 7. 创建 TopicLogger
    logger_ = TopicLogger::create();

    // 8. 计算 decimation（控制循环每 decimation_ 步执行一次策略推理）
    decimation_ = static_cast<int>(std::round(policy_dt_ / loop_dt_));

    state_ = ControllerState::kPaused;
    RL_LOG_SUCCESS("GenericRLController initialized");
    return true;
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("Exception during initialization: %s", e.what());
    return false;
  }
}


bool GenericRLController::update(double time, const RobotState& state,
                                  const ImuData& imu, RobotCmd& cmd) {
  if (state_ != ControllerState::kRunning) {
    return false;
  }

  // 缓存当前传感器数据，供观测计算使用
  current_state_ = state;
  current_imu_ = imu;

  // 首次 update 时初始化 dummy_world_yaw_（与 rl_mimic_controller 时序一致）
  MotionTrajectory* loader = getCurrentMotion();
  if (step_count_ == 0 && loader && loader->isLoaded()) {
    RL_LOG_INFO("\033[33m[step 0] initializeDummyWorldYaw at startup\033[0m");
    initializeDummyWorldYaw();
  }

  // 按 decimation 降频执行策略推理
  if (step_count_ % decimation_ == 0) {
    computeObservation();
    computeActions();
  }

  // 每个控制周期都更新电机命令
  updateRobotCmd(cmd);

  // 推进 motion 帧
  if (step_count_ % decimation_ == 0) {
    if (loader && motion_playing_ && loader->hasNext()) {
      loader->next();
    }
  }

  step_count_++;
  return true;
}

void GenericRLController::reset() {
  resetObsHistory();
  actions_.setZero();
  last_actions_.setZero();
  step_count_ = 0;
  motion_playing_ = false;
  dummy_world_yaw_ = 0.0;
  velocity_cmd_.setZero();

  MotionTrajectory* loader = getCurrentMotion();
  if (loader) {
    loader->reset();
    RL_LOG_INFO("Available motions: %zu. Press `guide` to start '%s'",
                motions_.size(), current_motion_name_.c_str());
  }
}

// ============================================================================
// Motion 播放控制
// ============================================================================

bool GenericRLController::startMotion() {
  MotionTrajectory* loader = getCurrentMotion();
  if (!loader) {
    return false;
  }

  // 如果已经在播放或播放完毕，重置到起始帧
  if (motion_playing_ || !loader->hasNext()) {
    loader->reset();
    RL_LOG_INFO("Motion '%s' reset to beginning", current_motion_name_.c_str());
  }

  // 在开始播放时重新初始化 dummy_world_yaw_（与 humanoidController 一致）
  RL_LOG_INFO("\033[32m[startMotion] initializeDummyWorldYaw before motion play\033[0m");
  initializeDummyWorldYaw();
  motion_playing_ = true;
  RL_LOG_INFO("\033[32mMotion '%s' playback started (dummy_world_yaw: %.3f)\033[0m",
              current_motion_name_.c_str(), dummy_world_yaw_);
  return true;
}

bool GenericRLController::startMotion(const std::string& name) {
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
  // 在开始播放时重新初始化 dummy_world_yaw_（与 humanoidController 一致）
  RL_LOG_INFO("\033[32m[startMotion(%s)] initializeDummyWorldYaw before motion play\033[0m", name.c_str());
  initializeDummyWorldYaw();
  motion_playing_ = true;
  RL_LOG_INFO("\033[32mMotion '%s' playback started (dummy_world_yaw: %.3f)\033[0m",
              name.c_str(), dummy_world_yaw_);
  return true;
}

bool GenericRLController::stopMotion() {
  if (motion_playing_) {
    motion_playing_ = false;
    MotionTrajectory* loader = getCurrentMotion();
    if (loader) {
      loader->reset();
    }
    RL_LOG_INFO("Motion playback stopped");
    return true;
  }
  return false;
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
// 手柄输入
// ============================================================================

void GenericRLController::onJoyInput(const JoyData& joy, const JoyData::Buttons& prev) {
  // guide 键触发 motion 播放
  if (joy.buttons.guide && !prev.guide) {
    startMotion();
    return;
  }

  // 摇杆映射为速度命令
  VelocityCommand vel;
  vel.linear_x  = -joy.axes.left_y;
  vel.linear_y  = -joy.axes.left_x;
  vel.angular_z = -joy.axes.right_x;
  setVelocityCommand(vel);
}

// ============================================================================
// 关节初始化
// ============================================================================

/// 线性插值过渡到默认关节位置
void GenericRLController::moveToDefaultPos(const RobotState& current_state, double elapse) {
  auto& robot = GlobalRobot::getInstance();

  if (current_state.q.empty()) {
    RL_LOG_FAILURE("Invalid robot state for moveToDefaultPos");
    return;
  }

  // 构建目标位置（全电机）
  // 有 motion 时使用首帧 joint_pos，使机器人在播放前就站在 motion 起始姿态
  std::vector<double> joint_current_pos = current_state.q;
  std::vector<double> joint_target_pos = current_state.q;
  MotionTrajectory* motion = getCurrentMotion();
  array_t target_joint_pos = (motion && motion->isLoaded())
      ? motion->getJointPos()   // motion 首帧关节位置
      : default_joint_pos_;

  // 仅设置策略控制的关节目标（应用方向系数）
  for (int i = 0; i < policy_joint_count_; ++i) {
    int motor_idx = policy_joint_ids_[i];
    if (motor_idx >= 0 && motor_idx < static_cast<int>(joint_target_pos.size())) {
      joint_target_pos[motor_idx] = joint_direction_[i] * target_joint_pos[i];
    }
  }

  RL_LOG_INFO("Moving to default position over %.1f seconds...", elapse);

  // 线性插值
  int motor_count = static_cast<int>(current_state.q.size());
  for (double t = 0.; t < elapse; t += loop_dt_) {
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
    //////////////////////////////////////////////
    // DEBUG: ROBAN 系列: EC电机 CSP PDO 的 kp/kd 参数
    // 从 kuavo.json 复制
    if (IS_ROBAN_LEGGED(robot_version_)) {
      constexpr double ec_motor_kp[] = {200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200};
      constexpr double ec_motor_kd[] = {5000, 5000, 5000, 5000, 10000, 10000, 5000, 5000, 5000, 5000, 10000, 10000, 5000};
      for (int i = 0; i < 13; i++) {
        cmd.kp[i] = ec_motor_kp[i];
        cmd.kd[i] = ec_motor_kd[i];
      }
    }
    /////////////////////////////////////////////////
    robot.publishRobotCmd(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(loop_dt_ * 1000)));
  }

  RL_LOG_INFO("Default position reached");
}

// ============================================================================
// 配置加载
// ============================================================================

bool GenericRLController::loadConfig(const std::string& config_path) {
  try {
    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node cfg = root["HumanoidRobotCfg"];

    // --- 控制时序 ---
    loop_dt_ = cfg["loop_dt"].as<double>();
    policy_dt_ = cfg["env"]["policy_dt"].as<double>();
    policy_path_ = cfg["policy_path"].as<std::string>();

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

    joint_direction_ = Eigen::Map<const Eigen::ArrayXd>(direction_vec.data(), direction_vec.size());
    default_joint_pos_ = Eigen::Map<const Eigen::ArrayXd>(default_pos_vec.data(), default_pos_vec.size());
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

      if (motion["motions"]) {
        for (const auto& item : motion["motions"]) {
          std::string motion_name = item["name"].as<std::string>();
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
    if (robot["residual_action"]) {
      motion_residual_action_ = robot["residual_action"].as<bool>();
    }

    // --- 速度命令限幅（可选） ---
    if (cfg["env"]["command_range"]) {
      YAML::Node cmd = cfg["env"]["command_range"];
      if (cmd["lin_vel_x"]) {
        auto range = cmd["lin_vel_x"].as<std::vector<double>>();
        cmd_lin_vel_x_ = {range[0], range[1]};
      }
      if (cmd["lin_vel_y"]) {
        auto range = cmd["lin_vel_y"].as<std::vector<double>>();
        cmd_lin_vel_y_ = {range[0], range[1]};
      }
      if (cmd["ang_vel_z"]) {
        auto range = cmd["ang_vel_z"].as<std::vector<double>>();
        cmd_ang_vel_z_ = {range[0], range[1]};
      }
    }

    return true;
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("Failed to load config: %s", e.what());
    return false;
  }
}

bool GenericRLController::loadPolicy(const std::string& policy_path) {
  model_ = std::make_unique<OpenVINOModel>();
  if (!model_->load(policy_path)) {
    RL_LOG_FAILURE("Failed to load policy model: %s", policy_path.c_str());
    return false;
  }
  RL_LOG_SUCCESS("Policy model loaded: %s", policy_path.c_str());
  return true;
}

// ============================================================================
// 观测 → 推理 → 控制命令
// ============================================================================

/// 构建观测向量：计算各 obs term → 更新历史缓冲 → 展平到 observations_
void GenericRLController::computeObservation() {
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
      logger_->publishVector("/" + name_ + "/obs/" + obs_terms_[i].name, v);
    }
    // 发布拼接后的单帧观测向量
    std::vector<double> single_obs_vec(observations_.data(),
                                       observations_.data() + observations_.size());
    logger_->publishVector("/" + name_ + "/observations", single_obs_vec);
  }
}

/// 执行策略推理：observations_ → model_ → actions_
void GenericRLController::computeActions() {
  if (!model_ || !model_->isLoaded()) {
    return;
  }

  // double → float（OpenVINO 输入）
  std::vector<float> obs_vec(observations_.size());
  for (int i = 0; i < observations_.size(); ++i) {
    obs_vec[i] = static_cast<float>(observations_[i]);
  }

  std::vector<float> action_vec = model_->forward(obs_vec);

  // float → double（内部使用）
  last_actions_ = actions_;
  if (action_vec.size() == static_cast<size_t>(policy_joint_count_)) {
    for (int i = 0; i < policy_joint_count_; ++i) {
      actions_[i] = static_cast<double>(action_vec[i]);
    }
  } else {
    RL_LOG_WARNING("Action size mismatch: got %zu, expected %d",
                   action_vec.size(), policy_joint_count_);
  }
}

/// 将 actions_ 转换为电机控制命令
void GenericRLController::updateRobotCmd(RobotCmd& cmd) {
  if (cmd.q.size() != static_cast<size_t>(motor_count_)) {
    cmd.resize(motor_count_);
  }

  // 计算目标关节位置：q_target = direction * (base_pos + action * scale)
  array_t q_target;
  MotionTrajectory* loader = getCurrentMotion();
  if (motion_residual_action_ && motion_playing_ && loader) {
    // 残差模式：动作叠加到 motion 参考轨迹上
    array_t motion_joint_pos = loader->getJointPos();
    q_target = joint_direction_ * (motion_joint_pos + actions_ * joint_action_scale_);
  } else {
    // 常规模式：有 motion 时用首帧位置（与 moveToDefaultPos 一致），否则用默认关节位置
    const array_t& base_pos = (loader && loader->isLoaded())
        ? loader->getJointPos() : default_joint_pos_;
    q_target = joint_direction_ * (base_pos + actions_ * joint_action_scale_);
  }

  // 非策略控制的关节：保持当前位置
  for (int i = 0; i < motor_count_; ++i) {
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

  cmd.timestamp = leju::common::GetUnixTimestampS();

  // 发布调试数据
  if (logger_) {
    std::vector<double> qt(q_target.data(), q_target.data() + q_target.size());
    logger_->publishVector("/" + name_ + "/q_target", qt);
    if (loader) {
      logger_->publishValue("/" + name_ + "/motion_frame",
                            static_cast<double>(loader->getCurrentFrame()));
    }
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
  return true;
}

// ============================================================================
// Motion 轨迹加载
// ============================================================================

/// 从配置目录加载所有 motion 轨迹文件
void GenericRLController::loadMotionTrajectories(const std::string& config_dir) {
  std::filesystem::path config_path(config_dir);
  for (const auto& [name, path] : motion_paths_) {
    std::string motion_full_path = (config_path / path).string();
    RL_LOG_INFO("Loading motion '%s' from: %s", name.c_str(), motion_full_path.c_str());

    auto motion_traj = std::make_unique<MotionTrajectory>();
    if (!motion_traj->load(motion_full_path)) {
      RL_LOG_WARNING("Failed to load motion '%s': %s", name.c_str(), motion_full_path.c_str());
      continue;
    }
    RL_LOG_SUCCESS("Motion '%s' loaded: %d frames", name.c_str(), motion_traj->getNumFrames());
    motions_[name] = std::move(motion_traj);
  }

  // 默认使用第一个 motion
  if (!motions_.empty()) {
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
  return joint_direction_ * policy_q - default_joint_pos_;
}

/// 获取策略关节速度（已乘方向系数）
array_t GenericRLController::getPolicyJointVel() const {
  array_t policy_v(policy_joint_count_);
  for (int i = 0; i < policy_joint_count_; ++i) {
    policy_v[i] = current_state_.v[policy_joint_ids_[i]];
  }
  return joint_direction_ * policy_v;
}

/// 获取限幅后的速度命令 [lin_vel_x, lin_vel_y, ang_vel_z]
array_t GenericRLController::getVelocityCommands() const {
  array_t cmd(3);
  cmd[0] = cmd_lin_vel_x_.clamp(velocity_cmd_.linear_x);
  cmd[1] = cmd_lin_vel_y_.clamp(velocity_cmd_.linear_y);
  cmd[2] = cmd_ang_vel_z_.clamp(velocity_cmd_.angular_z);
  return cmd;
}

/// 获取 IMU 角速度 [gx, gy, gz]
array_t GenericRLController::getBaseAngVel() const {
  array_t result(3);
  result << current_imu_.gyro[0], current_imu_.gyro[1], current_imu_.gyro[2];
  return result;
}

/// 获取投影重力向量：将 [0,0,-1] 从世界坐标系旋转到体坐标系
array_t GenericRLController::getProjectedGravity() const {
  Eigen::Vector3d g_hat(0., 0., -1.);
  Eigen::Quaterniond quat(current_imu_.quat[0], current_imu_.quat[1],
                          current_imu_.quat[2], current_imu_.quat[3]);
  return (quat.inverse() * g_hat).array();
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
    return actions_;
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
  }

  RL_LOG_WARNING("Unknown obs term: %s", name.c_str());
  return array_t();
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

}  // namespace leju
