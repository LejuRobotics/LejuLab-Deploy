#include "leju-rl-controller/controllers/controller_manager.h"

#include <Eigen/Dense>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "leju-rl-controller/controllers/generic_rl_controller.h"
#include "leju-rl-controller/rl/multi_mode_arm_controller.h"
#include "leju-rl-controller/rl/waist_controller.h"
#include "leju-rl-controller/rl_log.h"
#include "leju-rl-controller/utils/uri_path_resolver.h"
#include "leju-rl-controller/velocity_manager.h"
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

  // 初始化外部接口（VR SDK）- 复用 RobotData 创建的 participant
  if (!initExternalInterface()) {
    RL_LOG_FAILURE("Failed to initialize external interface");
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

bool ControllerManager::switchController(const std::string& name) {
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

  // 暂停当前控制器
  std::string prev_name = (active_index_ >= 0) ? controllers_[active_index_].name : "(none)";
  if (active_index_ >= 0) {
    RL_LOGI("Pausing current controller: %s", prev_name.c_str());
    controllers_[active_index_].controller->pause();
  }

  // 记录上一个控制器索引，切换到新控制器
  last_index_ = active_index_;
  active_index_ = target_index;

  // 恢复新控制器（使用 resume 而不是 reset+start）
  auto* new_controller = controllers_[active_index_].controller.get();
  RL_LOGI("Resuming new controller: %s", name.c_str());
  new_controller->resume();

  RL_LOGI("Switch complete: %s -> %s", prev_name.c_str(), name.c_str());
  return true;
}

void ControllerManager::starting() {
  running_ = true;

  // Step 1: 等待所有传感器数据就绪
  if (!waitForDataReady()) {
    RL_LOGI("ControllerManager stopped during data wait");
    return;
  }

  // Step 2: 如果没有激活的控制器，激活第一个
  if (active_index_ < 0 && !controllers_.empty()) {
    active_index_ = 0;
    RL_LOGI("Activating first controller: %s", controllers_[0].name.c_str());
  }

  // Step 4: 移动机器人到默认位置
  RL_LOGI("Moving to default position...");
  if (active_index_ >= 0) {
    RobotState current_state;
    if (robot_data_.getRobotState(current_state)) {
      controllers_[active_index_].controller->moveToDefaultPos(current_state, 3.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else {
      RL_LOGW("Failed to get robot state for moveToDefaultPos");
    }
  }
  RL_LOGI("Default position reached");

  // Step 5: 阻塞等待外部触发 start
  ready_for_start_ = true;
  std::cout << std::endl;
  std::cout << "\033[1;32m------------------------------------------------\033[0m" << std::endl;
  std::cout << "\033[1;32m- Ready! Press `start` button to begin\033[0m" << std::endl;
  std::cout << "\033[1;32m------------------------------------------------\033[0m" << std::endl;
  std::cout.flush();
  {
    std::unique_lock<std::mutex> lock(start_mutex_);
    start_cv_.wait(lock, [this] { return start_triggered_.load() || !running_.load(); });
  }

  if (!running_) {
    RL_LOGI("ControllerManager stopped during start wait");
    return;
  }

  // 恢复当前控制器（从 PAUSED -> RUNNING）
  if (active_index_ >= 0) {
    controllers_[active_index_].controller->resume();
  }

  std::cout << std::endl;
  std::cout << "\033[1;32m------------------------------------------------\033[0m" << std::endl;
  std::cout << "\033[1;32m- RL policy is now running!\033[0m" << std::endl;
  std::cout << "\033[1;32m- Press `back` button to stop\033[0m" << std::endl;
  std::cout << "\033[1;32m------------------------------------------------\033[0m" << std::endl;
  std::cout.flush();

  RL_LOGI("Controller started, entering control loop");
}

void ControllerManager::triggerStart() {
  start_triggered_ = true;
  start_cv_.notify_one();
}

void ControllerManager::stop() {
  const bool was_running = running_.exchange(false);

  // 唤醒可能阻塞在 starting() 中的线程
  start_cv_.notify_one();

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

void ControllerManager::update() {
  if (!running_.load()) {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(controllers_mutex_);

  if (active_index_ < 0) {
    return;
  }

  // 获取传感器数据
  RobotState state;
  ImuData imu;
  if (!robot_data_.getRobotState(state) || !robot_data_.getImuData(imu)) {
    return;
  }
  vr::VelocityCmd cmd_vel;
  bool has_cmd_vel = false;
  {
    std::lock_guard<std::mutex> lock(velocity_cmd_mutex_);
    if (velocity_cmd_received_) {
      cmd_vel = velocity_cmd_;
      has_cmd_vel = true;
    }
  }

  double time = common::GetSteadyTimestampNs() * 1e-9;

  // 调用当前控制器的 update 并发布指令
  RobotCmd cmd;
  auto* controller = controllers_[active_index_].controller.get();
  if (has_cmd_vel) {
    VelocityCommand vel;
    vel.linear_x = cmd_vel.linear_x;
    vel.linear_y = cmd_vel.linear_y;
    vel.angular_z = cmd_vel.angular_z;
    controller->setVelocityCommand(vel);
  } else {
    controller->setVelocityCommand(VelocityCommand{});
  }

  if (controller->update(time, state, imu, cmd) && running_.load()) {
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
    GlobalRobot::getInstance().publishRobotCmd(cmd);
  }
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

void ControllerManager::dispatchJoyInput(const JoyData& joy, const JoyData::Buttons& prev) {
#define JOY_PRESSED(btn) (joy.buttons.btn && !prev.btn)

  // TODO: 需要优化 trigger 的触发，按键处理放在这里不够优雅
  ////////////////////////////////////////////////////

  if (JOY_PRESSED(start)) {
    if (!ready_for_start_) {
      RL_LOG_WARNING("Start button pressed, but data not ready yet, ignored");
      return;
    }
    if (start_triggered_.load()) {
      RL_LOG_WARNING("Start button pressed, but already triggered, ignored");
      return;
    }
    triggerStart();
    RL_LOGI("Start button pressed, starting...");
    return;
  }
  else if (JOY_PRESSED(back)) {
    RL_LOGI("Back button pressed, stopping...");
    stop();
    return;
  }

  // FIXME:控制器切换: 先不支持
  // if (JOY_PRESSED(west)) {
  //   RL_LOGI("Joy [west] pressed, switching to 'amp'");
  //   switchController("amp");
  //   return;
  // }
  // else if (JOY_PRESSED(north)) {
  //   RL_LOGI("Joy [north] pressed, switching to 'mimic'");
  //   switchController("mimic");
  //   return;
  // }

  // 控制器级：透传给当前活跃控制器
  if (auto* controller = getCurrentController()) {
    controller->onJoyInput(joy, prev);
  }
  #undef JOY_PRESSED
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

      // 根据类型创建控制器
      std::unique_ptr<ControllerBase> controller;
      if (type == "GenericRLController") {
        auto generic_ctrl = std::make_unique<GenericRLController>(
            RobotVersion::from_env(), name);
        generic_ctrl->setConfigPath(ctrl_config_path);
        generic_ctrl->setUrdfPath(urdf_path);
        controller = std::move(generic_ctrl);
      } else {
        RL_LOGW("Unknown controller type: %s", type.c_str());
        continue;
      }

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

bool ControllerManager::waitForDataReady() {
  RL_LOG_SUCCESS("Waiting for sensor data...");

  // 等待 IMU 数据和机器人状态
  RL_LOGI("Checking sensor data (IMU + robot state)...");
  int wait_count = 0;
  while (running_ && !robot_data_.isDataReady()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (++wait_count % (10*3) == 0) {  // 每秒打印一次
      RL_LOGI("Waiting for sensor data... (%d s)", wait_count / 10);
    }
  }
  if (!running_) return false;
  RL_LOG_SUCCESS("Sensor data OK");

  // 等待硬件就绪
  RL_LOGI("Waiting for hardware to be ready...");
  while (running_ && !robot_data_.isHardwareReady()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!running_) return false;
  RL_LOG_SUCCESS("Hardware is ready");

  RL_LOG_SUCCESS("All data ready");
  return true;
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
// 外部接口
// ============================================================================

bool ControllerManager::initExternalInterface() {
  // 根据机器人版本创建对应的 VR API
  RobotVersion version = RobotVersion::from_env();
  if (IS_KUAVO(version)) {
    vr_api_ = std::make_unique<vr::KuavoVRAPI>();
  } else if (IS_ROBAN(version)) {
    vr_api_ = std::make_unique<vr::RobanVRAPI>();
  } else {
    RL_LOGW("Unknown robot version, external interface not initialized");
    return false;
  }

  // 初始化 VR API
  if (!vr_api_->initialize()) {
    RL_LOGE("Failed to initialize VR API");
    vr_api_.reset();
    return false;
  }

  // 订阅轨迹指令
  vr_api_->subscribeArmJointCmd(
      [this](const vr::JointTrajectoryPoint& cmd) { onArmJointCmd(cmd); });
  vr_api_->subscribeWaistJointCmd(
      [this](const vr::JointTrajectoryPoint& cmd) { onWaistJointCmd(cmd); });
  vr_api_->subscribeHeadJointCmd(
      [this](const vr::JointTrajectoryPoint& cmd) { onHeadJointCmd(cmd); });
  vr_api_->subscribeVrVelocityCmd(
      [this](const vr::VrVelocityCmd& cmd) { onVrVelocityCmd(cmd); });
  vr_api_->subscribeVelocityCmd(
      [this](const vr::VelocityCmd& cmd) { onResolvedVelocityCmd(cmd); });

  // 注册 RPC 服务处理函数
  vr_api_->registerSwitchControllerHandler(
      [this](const std::string& name, std::string& msg) {
        return onSwitchController(name, msg);
      });
  vr_api_->registerSetArmModeHandler(
      [this](vr::ControlMode mode, std::string& msg) {
        return onSetArmMode(mode, msg);
      });
  vr_api_->registerSetWaistModeHandler(
      [this](vr::ControlMode mode, std::string& msg) {
        return onSetWaistMode(mode, msg);
      });
  vr_api_->registerGetStateHandler([this]() { return onGetState(); });

  RL_LOG_SUCCESS("External interface initialized");
  return true;
}

void ControllerManager::onArmJointCmd(const vr::JointTrajectoryPoint& cmd) {
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

void ControllerManager::onWaistJointCmd(const vr::JointTrajectoryPoint& cmd) {
  auto* controller = getCurrentController();
  if (!controller) return;

  auto* waist_ctrl = controller->getWaistController();
  if (!waist_ctrl) return;

  Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(cmd.q.data(), cmd.q.size());
  waist_ctrl->setExternalTarget(q);
}

void ControllerManager::onHeadJointCmd(const vr::JointTrajectoryPoint& cmd) {
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

void ControllerManager::onVrVelocityCmd(const vr::VrVelocityCmd& cmd) {
  if (velocity_manager_) {
    velocity_manager_->onVrCmdVel(cmd);
  }
}

void ControllerManager::onResolvedVelocityCmd(const vr::VelocityCmd& cmd) {
  std::lock_guard<std::mutex> lock(velocity_cmd_mutex_);
  velocity_cmd_ = cmd;
  velocity_cmd_received_ = true;
}

bool ControllerManager::onSwitchController(const std::string& name, std::string& message) {
  if (switchController(name)) {
    message = "Switched to controller: " + name;
    return true;
  } else {
    message = "Failed to switch to controller: " + name;
    return false;
  }
}

bool ControllerManager::onSetArmMode(vr::ControlMode mode, std::string& message) {
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

  // 转换 vr::ControlMode 到 ArmControlMode
  ArmControlMode arm_mode;
  switch (mode) {
    case vr::ControlMode::kKeepPose:
      arm_mode = ArmControlMode::kKeepPose;
      break;
    case vr::ControlMode::kAuto:
      arm_mode = ArmControlMode::kAuto;
      break;
    case vr::ControlMode::kExternal:
      arm_mode = ArmControlMode::kExternal;
      break;
    default:
      message = "Unknown mode";
      return false;
  }

  arm_ctrl->setMode(arm_mode);
  message = "Arm mode set to " + std::to_string(static_cast<int>(mode));
  RL_LOGI("Arm mode set to %d", static_cast<int>(mode));
  return true;
}

bool ControllerManager::onSetWaistMode(vr::ControlMode mode, std::string& message) {
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

  // 转换 vr::ControlMode 到 WaistControlMode
  WaistControlMode waist_mode;
  switch (mode) {
    case vr::ControlMode::kAuto:
      waist_mode = WaistControlMode::kAuto;
      break;
    case vr::ControlMode::kExternal:
      waist_mode = WaistControlMode::kExternal;
      break;
    default:
      // WaistController 只支持 kAuto 和 kExternal
      message = "Waist controller only supports Auto and External modes";
      return false;
  }

  waist_ctrl->setMode(waist_mode);
  message = "Waist mode set to " + std::to_string(static_cast<int>(mode));
  RL_LOGI("Waist mode set to %d", static_cast<int>(mode));
  return true;
}

vr::ControllerState ControllerManager::onGetState() {
  vr::ControllerState state;

  state.current_controller = getCurrentControllerName();
  state.available_controllers = getControllerNames();

  auto* controller = getCurrentController();
  if (controller) {
    // 获取手臂模式
    if (auto* arm_ctrl = controller->getArmController()) {
      switch (arm_ctrl->getMode()) {
        case ArmControlMode::kKeepPose:
          state.arm_mode = vr::ControlMode::kKeepPose;
          break;
        case ArmControlMode::kAuto:
          state.arm_mode = vr::ControlMode::kAuto;
          break;
        case ArmControlMode::kExternal:
          state.arm_mode = vr::ControlMode::kExternal;
          break;
      }
    }

    // 获取腰部模式
    if (auto* waist_ctrl = controller->getWaistController()) {
      switch (waist_ctrl->getMode()) {
        case WaistControlMode::kAuto:
          state.waist_mode = vr::ControlMode::kAuto;
          break;
        case WaistControlMode::kExternal:
          state.waist_mode = vr::ControlMode::kExternal;
          break;
      }
    }
  }

  return state;
}

}  // namespace leju
