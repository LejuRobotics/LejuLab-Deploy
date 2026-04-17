#include "leju-rl-controller/controllers/controller_manager.h"

#include <Eigen/Dense>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "leju-rl-controller/controllers/controller_registry.h"
#include "leju-rl-controller/controllers/generic_rl_controller.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/rl_log.h"
#include "leju-rl-controller/utils/uri_path_resolver.h"
#include "lejusdk-utils/time_utils.hpp"

namespace leju {

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

  return true;
}

bool ControllerManager::addController(const std::string& name,
                                       std::unique_ptr<ControllerBase> controller) {
  // 检查是否已存在同名控制器
  if (hasController(name)) {
    RL_LOGE("Controller already exists: %s", name.c_str());
    return false;
  }

  // 添加控制器（initialize 后已处于 kPaused 待命状态）
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);
  controllers_.push_back({name, std::move(controller)});
  RL_LOGI("Added controller: %s", name.c_str());
  return true;
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

bool ControllerManager::requestSwitch(const std::string& name, double now) {
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

  // 启动过渡流程
  std::string from_name = (active_index_ >= 0) ? controllers_[active_index_].name : "(none)";

  transition_.state = SwitchState::kTransitioning;
  transition_.from_controller = from_name;
  transition_.to_controller = name;
  transition_.start_time = now;

  // 初始化手臂插值器
  transition_.arm_interpolation_initialized = false;
  transition_.arm_interpolator.reset();

  // 初始化腰部插值器
  transition_.waist_interpolation_initialized = false;
  transition_.waist_interpolator.reset();

  ControllerBase* from_controller = (active_index_ >= 0) ? controllers_[active_index_].controller.get() : nullptr;
  ControllerBase* to_controller = controllers_[target_index].controller.get();

  if (from_controller && to_controller) {
    size_t from_arm_count = from_controller->getArmJointCount();
    size_t to_arm_count = to_controller->getArmJointCount();

    // 只有当两个控制器都有手臂配置时才进行插值
    if (from_arm_count > 0 && to_arm_count > 0 && from_arm_count == to_arm_count) {
      // 获取当前手臂位置（从机器人状态）
      RobotState current_state;
      if (robot_data_.getRobotState(current_state)) {
        const auto& arm_ids = from_controller->getArmJointIds();
        Eigen::VectorXd from_arm_pos(from_arm_count);
        bool valid_state = true;

        for (size_t i = 0; i < from_arm_count; ++i) {
          int idx = arm_ids[i];
          if (idx >= 0 && idx < static_cast<int>(current_state.q.size())) {
            from_arm_pos[i] = current_state.q[idx];
          } else {
            valid_state = false;
            break;
          }
        }

        if (valid_state) {
          // 获取目标控制器的默认手臂姿态
          Eigen::VectorXd to_arm_pos = to_controller->getDefaultArmPos();

          if (to_arm_pos.size() == static_cast<int>(to_arm_count)) {
            // 设置插值器
            if (transition_.arm_interpolator.setup(from_arm_pos, to_arm_pos, transition_.duration)) {
              transition_.arm_joint_ids = arm_ids;
              transition_.arm_joint_count = static_cast<int>(from_arm_count);
              transition_.arm_interpolation_initialized = true;
              RL_LOGI("Arm interpolation initialized: %zu joints, duration=%.3fs",
                      from_arm_count, transition_.duration);
            }
          } else {
            RL_LOGW("Arm pos size mismatch: from=%zu, to=%zu, expected=%zu",
                    from_arm_pos.size(), to_arm_pos.size(), to_arm_count);
          }
        }
      }
    }

    // 腰部插值初始化
    size_t from_waist_count = from_controller->getWaistJointCount();
    size_t to_waist_count = to_controller->getWaistJointCount();

    if (from_waist_count > 0 && to_waist_count > 0 && from_waist_count == to_waist_count) {
      RobotState current_state;
      if (robot_data_.getRobotState(current_state)) {
        const auto& waist_ids = from_controller->getWaistJointIds();
        Eigen::VectorXd from_waist_pos(from_waist_count);
        bool valid_state = true;

        for (size_t i = 0; i < from_waist_count; ++i) {
          int idx = waist_ids[i];
          if (idx >= 0 && idx < static_cast<int>(current_state.q.size())) {
            from_waist_pos[i] = current_state.q[idx];
          } else {
            valid_state = false;
            break;
          }
        }

        if (valid_state) {
          Eigen::VectorXd to_waist_pos = to_controller->getDefaultWaistPos();

          if (to_waist_pos.size() == static_cast<int>(to_waist_count)) {
            if (transition_.waist_interpolator.setup(from_waist_pos, to_waist_pos, transition_.duration)) {
              transition_.waist_joint_ids = waist_ids;
              transition_.waist_joint_count = static_cast<int>(from_waist_count);
              transition_.waist_interpolation_initialized = true;
              RL_LOGI("Waist interpolation initialized: %zu joints, duration=%.3fs",
                      from_waist_count, transition_.duration);
            }
          } else {
            RL_LOGW("Waist pos size mismatch: from=%zu, to=%zu, expected=%zu",
                    from_waist_pos.size(), to_waist_pos.size(), to_waist_count);
          }
        }
      }
    }
  }

  RL_LOGI("RequestSwitch: %s -> %s (start_time=%.3f, duration=%.3f)",
          from_name.c_str(), name.c_str(), now, transition_.duration);

  return true;
}

void ControllerManager::commitSwitch() {
  // 查找目标控制器索引
  int target_index = -1;
  for (size_t i = 0; i < controllers_.size(); ++i) {
    if (controllers_[i].name == transition_.to_controller) {
      target_index = static_cast<int>(i);
      break;
    }
  }

  if (target_index < 0) {
    RL_LOGE("CommitSwitch: Target controller not found: %s",
            transition_.to_controller.c_str());
    transition_.state = SwitchState::kIdle;
    return;
  }

  std::string from_name = (active_index_ >= 0) ? controllers_[active_index_].name : "(none)";

  // 暂停当前控制器（OnExit）
  if (active_index_ >= 0) {
    RL_LOGI("Pausing current controller: %s", from_name.c_str());
    controllers_[active_index_].controller->pause();
  }

  // 记录上一个控制器索引，切换到新控制器
  last_index_ = active_index_;
  active_index_ = target_index;

  // 恢复新控制器（OnEnter）
  auto* new_controller = controllers_[active_index_].controller.get();
  RL_LOGI("Resuming new controller: %s", transition_.to_controller.c_str());
  new_controller->resume();

  // 重置过渡状态
  transition_.state = SwitchState::kIdle;

  RL_LOG_SUCCESS("CommitSwitch complete: %s -> %s", from_name.c_str(),
          transition_.to_controller.c_str());
}

bool ControllerManager::isTransitioning() const {
  return transition_.state == SwitchState::kTransitioning;
}

void ControllerManager::Start() {
  running_.store(true);

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

// SmoothStep 插值函数：s = 3x² - 2x³
static double SmoothStep(double x) {
  if (x <= 0.0) return 0.0;
  if (x >= 1.0) return 1.0;
  return x * x * (3.0 - 2.0 * x);
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

  if (active_index_ < 0) {
    return cmd;
  }

  // 从 CommandSnapshot 中提取速度指令（输入优先级已在 ControlLoop::mergeAllCmdVel() 中选择）
  auto* controller = controllers_[active_index_].controller.get();
  VelocityCommand vel_cmd;
  vel_cmd.linear_x = command.cmd_vel.linear_x;
  vel_cmd.linear_y = command.cmd_vel.linear_y;
  vel_cmd.angular_z = command.cmd_vel.angular_z;

  auto* arm_ctrl = controller->getArmController();
  if (arm_ctrl && arm_ctrl->getMode() != ArmControlMode::kAuto) {
    vel_cmd.setZero();
  }

  controller->setVelocityCommand(vel_cmd);

  // 调用当前控制器的 update
  double time = common::GetSteadyTimestampNs() * 1e-9;
  if (controller->update(time, state, imu_state, cmd)) {
    // 头部指令透传（头部是最后 2 个关节）
    {
      std::lock_guard<std::mutex> lock(head_cmd_mutex_);
      if (head_cmd_received_ && cmd.q.size() >= 2 && head_cmd_.q.size() >= 2) {
        size_t head_start = cmd.q.size() - 2;
        cmd.q[head_start] = head_cmd_.q[0];      // head_yaw
        cmd.q[head_start + 1] = head_cmd_.q[1];  // head_pitch
        if (head_cmd_.v.size() >= 2) {
          cmd.v[head_start] = head_cmd_.v[0];
          cmd.v[head_start + 1] = head_cmd_.v[1];
        }
      }
    }

    // 控制器切换期间：执行手臂插值并覆盖手臂关节
    if (transition_.state == SwitchState::kTransitioning &&
        transition_.arm_interpolation_initialized &&
        transition_elapsed >= 0.0 && transition_elapsed <= transition_.duration) {
      Eigen::VectorXd arm_pos, arm_vel;
      if (transition_.arm_interpolator.evaluate(transition_elapsed, arm_pos, arm_vel)) {
        // 确保 cmd 数组足够大
        int max_arm_idx = 0;
        for (int idx : transition_.arm_joint_ids) {
          if (idx > max_arm_idx) max_arm_idx = idx;
        }
        if (max_arm_idx < static_cast<int>(cmd.q.size()) &&
            arm_pos.size() == transition_.arm_joint_count) {
          for (int i = 0; i < transition_.arm_joint_count; ++i) {
            int idx = transition_.arm_joint_ids[i];
            cmd.q[idx] = arm_pos[i];
            if (arm_vel.size() > i) {
              cmd.v[idx] = arm_vel[i];
            }
            // 使用配置的 kp/kd，确保能跟踪到位
            if (i < static_cast<int>(transition_.arm_kp.size())) {
              cmd.kp[idx] = transition_.arm_kp[i];
            }
            if (i < static_cast<int>(transition_.arm_kd.size())) {
              cmd.kd[idx] = transition_.arm_kd[i];
            }
            cmd.modes[idx] = 2;  // CSP 模式
            cmd.tau[idx] = 0.0;  // 清零前馈力矩，避免与 PD 控制冲突
          }
        }
      }
    }

    // 控制器切换期间：执行腰部插值并覆盖腰部关节
    if (transition_.state == SwitchState::kTransitioning &&
        transition_.waist_interpolation_initialized &&
        transition_elapsed >= 0.0 && transition_elapsed <= transition_.duration) {
      Eigen::VectorXd waist_pos, waist_vel;
      if (transition_.waist_interpolator.evaluate(transition_elapsed, waist_pos, waist_vel)) {
        int max_waist_idx = 0;
        for (int idx : transition_.waist_joint_ids) {
          if (idx > max_waist_idx) max_waist_idx = idx;
        }
        if (max_waist_idx < static_cast<int>(cmd.q.size()) &&
            waist_pos.size() == transition_.waist_joint_count) {
          for (int i = 0; i < transition_.waist_joint_count; ++i) {
            int idx = transition_.waist_joint_ids[i];
            cmd.q[idx] = waist_pos[i];
            if (waist_vel.size() > i) {
              cmd.v[idx] = waist_vel[i];
            }
            if (i < static_cast<int>(transition_.waist_kp.size())) {
              cmd.kp[idx] = transition_.waist_kp[i];
            }
            if (i < static_cast<int>(transition_.waist_kd.size())) {
              cmd.kd[idx] = transition_.waist_kd[i];
            }
            cmd.modes[idx] = 2;
            cmd.tau[idx] = 0.0;
          }
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

void ControllerManager::processCommandBuffer(const runtime::CommandBuffer::Snapshot& snapshot) {
  // 所有命令处理已在 ControlLogic 中完成
  // 此方法保留供未来扩展使用
  (void)snapshot;
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
      addController(name, std::move(controller));
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

void ControllerManager::moveToDefaultPos(double elapse) {
  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  if (active_index_ < 0 || active_index_ >= static_cast<int>(controllers_.size())) {
    RL_LOGW("No active controller, cannot move to default position");
    return;
  }

  auto* controller = controllers_[active_index_].controller.get();
  const array_t& default_pos = controller->getDefaultJointPos();

  if (default_pos.size() == 0) {
    RL_LOGW("Controller has no default joint position defined");
    return;
  }

  RobotState state;
  if (!robot_data_.getRobotState(state)) {
    RL_LOGE("Failed to get robot state");
    return;
  }

  int motor_count = static_cast<int>(state.q.size());
  std::vector<double> joint_target_pos(motor_count, 0.0);
  std::vector<double> joint_current_pos = state.q;

  int copy_count = std::min(static_cast<int>(default_pos.size()), motor_count);
  for (int i = 0; i < copy_count; ++i) {
    joint_target_pos[i] = default_pos[i];
  }

  auto& robot = GlobalRobot::getInstance();
  constexpr double dt = 0.001;  // 插值步长 1ms

  for (double t = 0.; t < elapse && running_; t += dt) {
    double phase = t / elapse;

    RobotCmd cmd(motor_count);
    for (int i = 0; i < motor_count; ++i) {
      cmd.q[i] = joint_current_pos[i] + phase * (joint_target_pos[i] - joint_current_pos[i]);
      cmd.kp[i] = 100.0;
      cmd.kd[i] = 10.0;
      cmd.modes[i] = 2;
      cmd.v[i] = 0.0;
      cmd.tau[i] = 0.0;
    }

    robot.publishRobotCmd(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

// ============================================================================
// 切换插值配置加载
// ============================================================================

void ControllerManager::loadSwitchInterpolationConfig(const YAML::Node& config) {
  // 注意：所有配置必须从YAML加载，代码中不提供默认值
  // 直接加载到 transition_ 中，避免后续复制

  if (config["switch_interpolation"]) {
    const auto& switch_config = config["switch_interpolation"];

    // 加载手臂 kp
    if (switch_config["arm_kp"] && switch_config["arm_kp"].IsSequence()) {
      transition_.arm_kp.clear();
      for (const auto& val : switch_config["arm_kp"]) {
        transition_.arm_kp.push_back(val.as<double>());
      }
    }

    // 加载手臂 kd
    if (switch_config["arm_kd"] && switch_config["arm_kd"].IsSequence()) {
      transition_.arm_kd.clear();
      for (const auto& val : switch_config["arm_kd"]) {
        transition_.arm_kd.push_back(val.as<double>());
      }
    }

    // 加载腰部 kp
    if (switch_config["waist_kp"] && switch_config["waist_kp"].IsSequence()) {
      transition_.waist_kp.clear();
      for (const auto& val : switch_config["waist_kp"]) {
        transition_.waist_kp.push_back(val.as<double>());
      }
    }

    // 加载腰部 kd
    if (switch_config["waist_kd"] && switch_config["waist_kd"].IsSequence()) {
      transition_.waist_kd.clear();
      for (const auto& val : switch_config["waist_kd"]) {
        transition_.waist_kd.push_back(val.as<double>());
      }
    }

    RL_LOGI("Loaded switch_interpolation config:");
    RL_LOGI("  arm_kp: [%zu values]", transition_.arm_kp.size());
    RL_LOGI("  arm_kd: [%zu values]", transition_.arm_kd.size());
    RL_LOGI("  waist_kp: [%zu values]", transition_.waist_kp.size());
    RL_LOGI("  waist_kd: [%zu values]", transition_.waist_kd.size());
  } else {
    // 没有配置则清空（不使用默认值）
    transition_.arm_kp.clear();
    transition_.arm_kd.clear();
    transition_.waist_kp.clear();
    transition_.waist_kd.clear();

    RL_LOGW("No switch_interpolation config found, switch interpolation will use controller's kp/kd");
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

  bool success = false;
  if (!name.empty()) {
    // 尝试 dynamic_cast 到 GenericRLController 调用带 name 版本
    auto* generic_rl = dynamic_cast<GenericRLController*>(controller);
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

}  // namespace leju
