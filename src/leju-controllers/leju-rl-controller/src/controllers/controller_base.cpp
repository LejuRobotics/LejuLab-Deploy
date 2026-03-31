#include "leju-rl-controller/controllers/controller_base.h"
#include "leju-rl-controller/rl_log.h"

#include <algorithm>
#include <cmath>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace leju {

// ============================================================================
// 生命周期管理
// ============================================================================

void ControllerBase::stop() {
  state_ = ControllerState::kStopped;
  RL_LOGI("Controller '%s' stopped", name_.c_str());
}

void ControllerBase::reset() {
  step_count_ = 0;
  // 基类空实现，子类按需 override
}

void ControllerBase::pause() {
  if (state_ == ControllerState::kRunning) {
    state_ = ControllerState::kPaused;
    RL_LOGI("Controller '%s' paused", name_.c_str());
  } else {
    RL_LOGW("Cannot pause controller '%s': current state=%d", name_.c_str(), static_cast<int>(state_));
  }
}

void ControllerBase::resume() {
  if (state_ != ControllerState::kStopped) {
    state_ = ControllerState::kRunning;
    reset();
    RL_LOGI("Controller '%s' resumed, state reset", name_.c_str());
  } else {
    RL_LOGW("Cannot resume controller '%s': current state is STOPPED", name_.c_str());
  }
}

// ============================================================================
// update() 模板方法
// ============================================================================

bool ControllerBase::update(double time, const RobotState& state, const ImuData& imu, RobotCmd& cmd) {
  // 1. 状态检查
  if (state_ != ControllerState::kRunning) {
    return false;
  }

  // 2. 缓存传感器数据
  current_state_ = state;
  current_imu_ = imu;

  // 3. 调用子类实现的具体更新逻辑
  bool success = updateImpl(time, state, imu, cmd);
  if (!success) {
    return false;
  }

  // 4. 更新手臂控制指令
  updateArmCommand(cmd);

  // 5. 更新腰部控制指令
  updateWaistCommand(cmd);

  // 6. 步数递增
  step_count_++;

  return true;
}

// ============================================================================
// 部位控制器更新
// ============================================================================

void ControllerBase::updateArmCommand(RobotCmd& cmd) {
  if (!arm_controller_ || arm_joint_names_.size() <= 0) {
    return;
  }

  // 计算站立/行走状态：1=站立, 0=行走
  double cmd_stance = (std::abs(velocity_cmd_.linear_x) < 0.01 &&
                       std::abs(velocity_cmd_.linear_y) < 0.01 &&
                       std::abs(velocity_cmd_.angular_z) < 0.01) ? 1.0 : 0.0;

  // 提取当前手臂位置和速度
  Eigen::VectorXd current_arm_pos(arm_joint_names_.size());
  Eigen::VectorXd current_arm_vel(arm_joint_names_.size());
  for (int i = 0; i < arm_joint_names_.size(); ++i) {
    int motor_id = arm_joint_ids_[i];
    int policy_idx = arm_policy_start_idx_ + i;
    // 应用方向系数转换到策略空间
    current_arm_pos[i] = joint_direction_[policy_idx] * current_state_.q[motor_id];
    current_arm_vel[i] = joint_direction_[policy_idx] * current_state_.v[motor_id];
  }

  Eigen::VectorXd desire_arm_q, desire_arm_v;
  bool override_arm = arm_controller_->update(cmd_stance, current_arm_pos, current_arm_vel,
                                               &desire_arm_q, &desire_arm_v);
  if (override_arm) {
    // 用部位控制器输出完全覆盖 cmd 中的手臂关节
    // 必须同时覆盖 q, v, tau，否则 RL 输出的速度/力矩会导致手臂偏离
    for (int i = 0; i < arm_joint_names_.size(); ++i) {
      int motor_id = arm_joint_ids_[i];
      int policy_idx = arm_policy_start_idx_ + i;
      // 转换回电机空间
      cmd.q[motor_id] = joint_direction_[policy_idx] * desire_arm_q[i];
      cmd.v[motor_id] = joint_direction_[policy_idx] * desire_arm_v[i];
      cmd.tau[motor_id] = 0.0;  // 清除前馈力矩
    }

  }
}

void ControllerBase::updateWaistCommand(RobotCmd& cmd) {
  if (!waist_controller_ || waist_joint_names_.size() <= 0) {
    return;
  }

  // 计算站立/行走状态
  double cmd_stance = (std::abs(velocity_cmd_.linear_x) < 0.01 &&
                       std::abs(velocity_cmd_.linear_y) < 0.01 &&
                       std::abs(velocity_cmd_.angular_z) < 0.01) ? 1.0 : 0.0;

  // 提取当前腰部位置和速度
  Eigen::VectorXd current_waist_pos(waist_joint_names_.size());
  Eigen::VectorXd current_waist_vel(waist_joint_names_.size());
  for (int i = 0; i < waist_joint_names_.size(); ++i) {
    int motor_id = waist_joint_ids_[i];
    int policy_idx = waist_policy_start_idx_ + i;
    current_waist_pos[i] = joint_direction_[policy_idx] * current_state_.q[motor_id];
    current_waist_vel[i] = joint_direction_[policy_idx] * current_state_.v[motor_id];
  }

  Eigen::VectorXd desire_waist_q, desire_waist_v;
  bool override_waist = waist_controller_->update(cmd_stance, current_waist_pos, current_waist_vel,
                                                   &desire_waist_q, &desire_waist_v);
  if (override_waist) {
    // 完全覆盖 q, v, tau，并强制使用 CSP 模式
    for (int i = 0; i < waist_joint_names_.size(); ++i) {
      int motor_id = waist_joint_ids_[i];
      int policy_idx = waist_policy_start_idx_ + i;
      cmd.q[motor_id] = joint_direction_[policy_idx] * desire_waist_q[i];
      cmd.v[motor_id] = joint_direction_[policy_idx] * desire_waist_v[i];
      cmd.tau[motor_id] = 0.0;
      // 强制使用 CSP 模式，让硬件层用 kp/kd 控制位置
      cmd.modes[motor_id] = 2;  // CSP
      cmd.kp[motor_id] = joint_kp_[policy_idx];
      cmd.kd[motor_id] = joint_kd_[policy_idx];
    }
  }
}

// ============================================================================
// 配置加载
// ============================================================================

bool ControllerBase::loadConfig(const std::string& config_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("Failed to parse YAML: %s\n  %s", config_path.c_str(), e.what());
    return false;
  }

  try {
    YAML::Node cfg = root["HumanoidRobotCfg"];

    // 解析通用时序配置
    if (cfg["loop_dt"]) {
      loop_dt_ = cfg["loop_dt"].as<double>();
    }

    // 解析部位控制器配置
    if (cfg["env"] && cfg["env"]["robot"]) {
      const YAML::Node& robot_node = cfg["env"]["robot"];

      // 解析 YAML 的 joint_names（策略控制的关节列表）
      std::vector<std::string> policy_joint_names;
      if (robot_node["joint_names"]) {
        policy_joint_names = robot_node["joint_names"].as<std::vector<std::string>>();
      }

      // 校验：配置中的 joint_names 必须包含所有硬件手臂关节
      // （关节名称由 ControllerManager 通过 setPartJointNames() 传入）
      for (const auto& arm_name : arm_joint_names_) {
        auto it = std::find(policy_joint_names.begin(), policy_joint_names.end(), arm_name);
        if (it == policy_joint_names.end()) {
          RL_LOG_FAILURE("Config error: Robot arm joint '%s' not found in joint_names. "
                         "Please ensure joint_names includes all arm joints from robot hardware.",
                         arm_name.c_str());
          return false;
        }
      }

      // 校验：配置中的 joint_names 必须包含所有硬件腰部关节
      for (const auto& waist_name : waist_joint_names_) {
        auto it = std::find(policy_joint_names.begin(), policy_joint_names.end(), waist_name);
        if (it == policy_joint_names.end()) {
          RL_LOG_FAILURE("Config error: Robot waist joint '%s' not found in joint_names. "
                         "Please ensure joint_names includes all waist joints from robot hardware.",
                         waist_name.c_str());
          return false;
        }
      }

      // 解析部位控制器开关（只控制是否创建部位控制器实例）
      if (robot_node["enable_arm_controller"]) {
        enable_arm_controller_ = robot_node["enable_arm_controller"].as<bool>();
      }
      if (robot_node["enable_waist_controller"]) {
        enable_waist_controller_ = robot_node["enable_waist_controller"].as<bool>();
      }

      RL_LOGI("Arm joints loaded (%zu from robot), controller %s",
              arm_joint_names_.size(), enable_arm_controller_ ? "enabled" : "disabled");
      RL_LOGI("Waist joints loaded (%zu from robot), controller %s",
              waist_joint_names_.size(), enable_waist_controller_ ? "enabled" : "disabled");
    }

    return true;
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("Config error in %s\n  %s", config_path.c_str(), e.what());
    return false;
  }
}

void ControllerBase::buildPartJointMapping() {
  // 构建手臂关节映射
  arm_joint_ids_.clear();
  arm_policy_start_idx_ = -1;
  for (const auto& name : arm_joint_names_) {
    bool found = false;
    for (size_t j = 0; j < joint_names_.size(); ++j) {
      if (joint_names_[j] == name) {
        if (arm_policy_start_idx_ < 0) {
          arm_policy_start_idx_ = static_cast<int>(j);
        }
        arm_joint_ids_.push_back(policy_joint_ids_[j]);
        found = true;
        break;
      }
    }
    if (!found) {
      RL_LOG_WARNING("Arm joint '%s' not found in joint_names", name.c_str());
    }
  }

  // 构建腰部关节映射
  waist_joint_ids_.clear();
  waist_policy_start_idx_ = -1;
  for (const auto& name : waist_joint_names_) {
    bool found = false;
    for (size_t j = 0; j < joint_names_.size(); ++j) {
      if (joint_names_[j] == name) {
        if (waist_policy_start_idx_ < 0) {
          waist_policy_start_idx_ = static_cast<int>(j);
        }
        waist_joint_ids_.push_back(policy_joint_ids_[j]);
        found = true;
        break;
      }
    }
    if (!found) {
      RL_LOG_WARNING("Waist joint '%s' not found in joint_names", name.c_str());
    }
  }

  if (arm_joint_names_.size() > 0) {
    RL_LOG_INFO("Arm joint mapping: %d joints, policy start idx: %d",
                arm_joint_names_.size(), arm_policy_start_idx_);
  }
  if (waist_joint_names_.size() > 0) {
    RL_LOG_INFO("Waist joint mapping: %d joints, policy start idx: %d",
                waist_joint_names_.size(), waist_policy_start_idx_);
  }
}

void ControllerBase::initPartControllers() {
  // 构建关节映射
  buildPartJointMapping();

  // 初始化手臂控制器
  if (enable_arm_controller_ && arm_joint_names_.size() > 0) {
    MultiModeArmControllerConfig arm_config;
    arm_config.enabled = true;
    arm_controller_ = std::make_unique<MultiModeArmController>(arm_config);

    // 提取手臂默认姿态
    Eigen::VectorXd default_arm_pos(arm_joint_names_.size());
    for (int i = 0; i < arm_joint_names_.size(); ++i) {
      default_arm_pos[i] = default_joint_pos_[arm_policy_start_idx_ + i];
    }

    arm_controller_->init(arm_joint_names_.size(), default_arm_pos, loop_dt_);
    RL_LOG_SUCCESS("Arm controller initialized (%d joints)", arm_joint_names_.size());
  }

  // 初始化腰部控制器
  if (enable_waist_controller_ && waist_joint_names_.size() > 0) {
    WaistControllerConfig waist_config;
    waist_config.enabled = true;
    waist_controller_ = std::make_unique<WaistController>(waist_config);

    // 提取腰部默认姿态
    Eigen::VectorXd default_waist_pos(waist_joint_names_.size());
    for (int i = 0; i < waist_joint_names_.size(); ++i) {
      default_waist_pos[i] = default_joint_pos_[waist_policy_start_idx_ + i];
    }

    waist_controller_->init(waist_joint_names_.size(), default_waist_pos, loop_dt_);
    RL_LOG_SUCCESS("Waist controller initialized (%d joints)", waist_joint_names_.size());
  }
}

// ============================================================================
// 其他接口
// ============================================================================

void ControllerBase::setVelocityCommand(const VelocityCommand& cmd) {
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  velocity_cmd_ = cmd;
}

void ControllerBase::moveToDefaultPos(const RobotState& current_state, double elapse) {
  (void)current_state;
  (void)elapse;
}

std::string ControllerBase::getName() const {
  return name_;
}

ControllerState ControllerBase::getState() const {
  return state_;
}

const array_t& ControllerBase::getDefaultJointPos() const {
  return default_joint_pos_;
}

bool ControllerBase::isActive() const {
  return state_ == ControllerState::kRunning;
}

bool ControllerBase::isPaused() const {
  return state_ == ControllerState::kPaused;
}

bool ControllerBase::isInitialized() const {
  return state_ != ControllerState::kUninitialized;
}

void ControllerBase::waitNextCycle(std::chrono::steady_clock::time_point cycle_start) {
  auto cycle_duration = std::chrono::duration<double>(loop_dt_);
  auto next_cycle = cycle_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(cycle_duration);
  std::this_thread::sleep_until(next_cycle);
}

}  // namespace leju
