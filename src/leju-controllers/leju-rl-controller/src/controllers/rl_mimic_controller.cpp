#include "leju-rl-controller/controllers/rl_mimic_controller.h"

#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "leju-rl-controller/inference/model_factory.h"
#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace rl_mimic {

bool RLMimicConfig::loadFromYaml(YAML::Node root_node) {
  YAML::Node humanoid_cfg = root_node["HumanoidRobotCfg"];
  loop_dt = humanoid_cfg["loop_dt"].as<double>();
  policy_path = humanoid_cfg["policy_path"].as<std::string>();
  motion_data_path = humanoid_cfg["motion_data_path"].as<std::string>();

  policy_dt = humanoid_cfg["env"]["policy_dt"].as<double>();

  YAML::Node robot = humanoid_cfg["env"]["robot"];
  residual_action = robot["residual_action"].as<bool>();
  joint_names = robot["joint_names"].as<std::vector<std::string>>();
  auto joint_direction_vec = robot["joint_direction"].as<std::vector<double>>();
  auto joint_default_pos = robot["joint_default_pos"].as<std::vector<double>>();
  auto joint_torque_limit =
      robot["joint_torque_limit"].as<std::vector<double>>();
  auto actuator_kp = robot["actuator_kp"].as<std::vector<double>>();
  auto actuator_kd = robot["actuator_kd"].as<std::vector<double>>();
  auto actuator_control_mode =
      robot["actuator_control_mode"].as<std::vector<int>>();
  auto v_action_scale = robot["action_scale"].as<std::vector<double>>();

  joint_direction = Eigen::Map<const Eigen::ArrayXd>(
      joint_direction_vec.data(), joint_direction_vec.size());
  q_default = Eigen::Map<const Eigen::ArrayXd>(joint_default_pos.data(),
                                               joint_default_pos.size());
  torque_limit = Eigen::Map<const Eigen::ArrayXd>(joint_torque_limit.data(),
                                                  joint_torque_limit.size());
  kp = Eigen::Map<const Eigen::ArrayXd>(actuator_kp.data(), actuator_kp.size());
  kd = Eigen::Map<const Eigen::ArrayXd>(actuator_kd.data(), actuator_kd.size());
  control_mode = Eigen::Map<const Eigen::ArrayXi>(actuator_control_mode.data(),
                                                  actuator_control_mode.size());
  action_scale = Eigen::Map<const Eigen::ArrayXd>(v_action_scale.data(),
                                                  v_action_scale.size());

  YAML::Node obs = humanoid_cfg["env"]["observations"];
  history_length = obs["history_length"].as<int>();
  stack_order_is_isaaclab =
      (obs["stack_order"].as<std::string>() == "isaaclab");

  // 读取推理引擎配置（可选，未指定时默认使用 OpenVINO）
  if (humanoid_cfg["inference_engine"]) {
    inference_engine = humanoid_cfg["inference_engine"].as<std::string>();
  } else {
    inference_engine = "openvino";
  }
  YAML::Node obsterms = obs["terms"];
  for (const auto& pair : obsterms) {
    std::string name = pair.first.as<std::string>();
    double scale = pair.second["scale"].as<double>();
    std::vector<double> clip_range =
        pair.second["clip"].as<std::vector<double>>();
    obs_terms.emplace_back(
        ObsTermConfig{name, scale, clip_range[0], clip_range[1]});
  }

  return true;
}

void RLMimicController::printRobotInfo() const {
  std::cout << "  Robot Version: " << robot_version_.version_name()
            << std::endl;
  std::cout << "  Motor Count: " << motor_count_ << std::endl;
  std::cout << "  Motors: ";
  for (size_t i = 0; i < motor_names_.size(); ++i) {
    if (i > 0) std::cout << ", ";
    std::cout << motor_names_[i];
  }
  std::cout << std::endl;
}

void RLMimicController::setupSubscriptions() {
  auto& robot = GlobalRobot::getInstance();
  robot.subscribeImuData(
      [this](const ImuDataConstPtr& imu_data) { this->imuCallback(imu_data); });
  robot.subscribeJoyData([this](const JoyDataConstPtr& joy_data) {
    this->joyDataCallback(joy_data);
  });

  robot.subscribeRobotState([this](const RobotStateConstPtr& robot_state) {
    this->jointStateCallback(robot_state);
  });

  robot.subscribeHardwareState([this](const StringDataConstPtr& hw_state) {
    this->hwStateCallback(hw_state);
  });
}

void RLMimicController::imuCallback(const ImuDataConstPtr& imu_data) {
  std::lock_guard<std::mutex> imu_lock(imu_data_mutex_);
  imu_data_ = *imu_data;
  imu_data_msg_cnt_++;
}

void RLMimicController::jointStateCallback(
    const RobotStateConstPtr& robot_state) {
  std::lock_guard<std::mutex> state_lock(robot_state_mutex_);
  robot_state_ = *robot_state;
  robot_state_msg_cnt_++;
}

void RLMimicController::hwStateCallback(const StringDataConstPtr& hw_state) {
  std::lock_guard<std::mutex> lock(hw_state_mutex_);
  hw_state_ = leju::String2HwState(hw_state->data);
  hw_state_msg_cnt_++;
}

void RLMimicController::joyDataCallback(const JoyDataConstPtr& joy) {
  std::lock_guard<std::mutex> joy_lock(joy_data_mutex_);
  joy_prev_ = joy_;
  joy_ = *joy;

#define CALC_PRESS(name)                                                 \
  {                                                                      \
    buttons_press_.name += joy_.buttons.name && !joy_prev_.buttons.name; \
  }
  CALC_PRESS(south)
  CALC_PRESS(east)
  CALC_PRESS(west)
  CALC_PRESS(north)
  CALC_PRESS(back)
  CALC_PRESS(guide)
  CALC_PRESS(start)
  CALC_PRESS(left_stick)
  CALC_PRESS(right_stick)
  CALC_PRESS(left_shoulder)
  CALC_PRESS(right_shoulder)
  CALC_PRESS(dpad_up)
  CALC_PRESS(dpad_down)
  CALC_PRESS(dpad_left)
  CALC_PRESS(dpad_right)
  CALC_PRESS(misc1)
#undef CALC_PRESS

  joy_data_msg_cnt_++;
}

bool RLMimicController::load_cfg(const std::string& config_file) {
  std::cout << "[INFO] [RLMimicController::load_cfg] "
            << "yaml config load from: " << config_file << "." << std::endl;
  YAML::Node cfg_node = YAML::LoadFile(config_file);
  cfg_.loadFromYaml(cfg_node);
  decimation_ = std::round(cfg_.policy_dt / cfg_.loop_dt);
  if (decimation_ != cfg_.policy_dt / cfg_.loop_dt) {
    std::cerr << "[WARN] [RLMimicController::load_cfg] "
              << "policy_dt is not multiple times of loop_dt." << std::endl;
  }
  for (int i = 0; i < cfg_.joint_names.size(); i++) {
    int joint_id = -1;
    for (int j = 0; j < motor_count_; j++) {
      if (cfg_.joint_names[i] == motor_names_[j]) {
        joint_id = j;
        break;
      }
    }
    if (joint_id == -1) {
      std::cerr << "[ERROR] [RLMimicController::initialize] "
                << "unable to find a name in motor_names_ that match name in "
                << "cfg_.joint_names: " << cfg_.joint_names[i] << std::endl;
      return false;
    }
    policy_joint_ids_.emplace_back(joint_id);
  }
  policy_joint_count_ = policy_joint_ids_.size();
  int shape_single_obs = 0;
  for (int i = 0; i < cfg_.obs_terms.size(); i++) {
    shape_single_obs += get_shape_obs_term(cfg_.obs_terms[i].name);
  }
  policy_obs_shape_ = shape_single_obs * cfg_.history_length;
  policy_obs_.resize(policy_obs_shape_);
  policy_obs_ = 0.;
  if (cfg_.stack_order_is_isaaclab) {
    obs_term_stacks_.resize(cfg_.obs_terms.size());
    for (int i = 0; i < cfg_.obs_terms.size(); i++) {
      obs_term_stacks_[i].resize(cfg_.history_length);
      for (int j = 0; j < cfg_.history_length; j++) {
        obs_term_stacks_[i][j] =
            Eigen::ArrayXd::Zero(get_shape_obs_term(cfg_.obs_terms[i].name));
      }
    }
  } else {
    obs_term_stacks_.resize(cfg_.history_length);
    for (int i = 0; i < cfg_.history_length; i++) {
      obs_term_stacks_[i].resize(cfg_.obs_terms.size());
      for (int j = 0; j < cfg_.obs_terms.size(); j++) {
        obs_term_stacks_[i][j] =
            Eigen::ArrayXd::Zero(get_shape_obs_term(cfg_.obs_terms[j].name));
      }
    }
  }
  policy_action_.resize(policy_joint_count_);
  policy_action_ = 0.;

  // std::cout << "cfg:" << std::endl;
  // std::cout << "\tloop_dt: " << cfg.loop_dt << std::endl;
  // std::cout << "\tpolicy_path: " << cfg.policy_path << std::endl;
  // std::cout << "\tjoint_names: [";
  // for (const auto& name: cfg.joint_names) {
  //   std::cout << name << ", ";
  // }
  // std::cout << "]" << std::endl;
  return true;
}

bool RLMimicController::load_policy() {
  // Extract config directory from config file path using filesystem
  std::filesystem::path config_path(config_file_path_);
  std::string policy_path =
      (config_path.parent_path() / cfg_.policy_path).string();
  std::cout << "[INFO] [RLMimicController::load_policy] "
            << "policy load from: " << policy_path << "." << std::endl;

  // 使用工厂模式创建推理模型
  inference_engine_ = cfg_.inference_engine;
  model_ = ModelFactory::create(inference_engine_);

  if (!model_) {
    std::cerr << "[ERROR] [RLMimicController::load_policy] "
              << "Failed to create inference model: " << inference_engine_
              << " (supported: openvino, onnxruntime)" << std::endl;
    return false;
  }

  if (!model_->load(policy_path)) {
    std::cerr << "[ERROR] [RLMimicController::load_policy] "
              << "Failed to load policy model: " << policy_path << std::endl;
    return false;
  }

  std::cout << "[INFO] [RLMimicController::load_policy] "
            << "Policy model loaded: " << policy_path
            << " (engine: " << ModelFactory::typeToString(model_->getModelType()) << ")"
            << std::endl;

  return true;
}

bool RLMimicController::load_motion_data() {
  std::filesystem::path config_path(config_file_path_);
  std::string motion_data_path =
      (config_path.parent_path() / cfg_.motion_data_path).string();
  std::cout << "[INFO] [RLMimicController::load_motion_data] "
            << "motion data load from: " << motion_data_path << "."
            << std::endl;
  std::ifstream file(motion_data_path);

  // body_pos_[xyz], body_quat_[wxyz], joint_pos[xx], joint_vel[xx] TODO check
  // names
  const int expected_cols = 3 + 4 + policy_joint_count_ * 2;

  std::string line;
  std::getline(file, line);

  // Auto-detect delimiter: comma or tab
  char delimiter = (line.find(',') != std::string::npos) ? ',' : '\t';

  std::stringstream ss(line);
  std::string value;
  while (std::getline(ss, value, delimiter)) {
    data_field_names_.push_back(value);
  }
  if (data_field_names_.size() != expected_cols) {
    std::cerr << "[ERROR] [RLMimicController::load_motion_data] "
              << "data_field_names_ size != expected_cols, "
              << "data_field_names_ size: " << data_field_names_.size()
              << ", expected_cols: " << expected_cols << "." << std::endl;
    return false;
  }

  std::vector<double> csv_data_flatten;
  int num_row = 0;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string value;

    int row_data_size = 0;
    while (std::getline(ss, value, delimiter)) {
      if (value.empty()) continue;
      csv_data_flatten.push_back(std::stod(value));
      row_data_size++;
    }
    if (row_data_size != expected_cols) {
      std::cerr << "[ERROR] [RLMimicController::load_motion_data] "
                << "row_size != expected_cols, "
                << "row_size: " << row_data_size
                << ", expected_cols: " << expected_cols << "." << std::endl;
      return false;
    }
    num_row++;
  }
  motion_data_ = Eigen::Map<Eigen::Array<double, -1, -1, Eigen::RowMajor>>(
      csv_data_flatten.data(), num_row, expected_cols);
  num_data_rows_ = num_row;
  return true;
}

void RLMimicController::jointMoveTo(const std::vector<double>& joint_target_pos,
                                    double elapse) {
  std::cout << "[DEBUG] [RLMimicController::jointMoveTo] "
            << "func start." << std::endl;
  std::cout << "joint_target_pos: [";
  for (int i = 0; i < joint_target_pos.size(); i++) {
    std::cout << joint_target_pos[i] << ", ";
  }
  std::cout << "]." << std::endl;
  auto& robot = GlobalRobot::getInstance();
  RobotState robot_state = getRobotState();
  std::vector<double> joint_current_pos = robot_state.q;
  std::cout << "joint_current_pos: [";
  for (int i = 0; i < joint_current_pos.size(); i++) {
    std::cout << joint_current_pos[i] << ", ";
  }
  std::cout << "]." << std::endl;

  for (double t = 0.; t < elapse; t += cfg_.loop_dt) {
    double phase = t / elapse;
    std::vector<double> joint_demand_pos(motor_count_);
    std::vector<double> joint_demand_vel(motor_count_);
    std::vector<double> joint_demand_tau(motor_count_);
    std::vector<double> kp(motor_count_);
    std::vector<double> kd(motor_count_);
    std::vector<unsigned char> mode(motor_count_);
    for (int i = 0; i < motor_count_; i++) {
      joint_demand_pos[i] =
          joint_current_pos[i] +
          phase * (joint_target_pos[i] - joint_current_pos[i]);
      kp[i] = 100.;
      kd[i] = 10.;
      mode[i] = 2;
      joint_demand_vel[i] = 0.;
      joint_demand_tau[i] = 0.;
    }

    RobotCmd cmd(motor_count_);
    cmd.q = joint_demand_pos;
    cmd.kp = kp;
    cmd.kd = kd;
    cmd.modes = mode;
    cmd.v = joint_demand_vel;
    cmd.tau = joint_demand_tau;
    robot.publishRobotCmd(cmd);
    // TODO use loop rate sleep
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(cfg_.loop_dt * 1000)));
  }
}

void RLMimicController::computeObservation() {
  if (cfg_.stack_order_is_isaaclab) {
    for (int i = 0; i < cfg_.obs_terms.size(); i++) {
      obs_term_stacks_[i].pop_front();
      auto obs_term = get_obs_term(cfg_.obs_terms[i].name);
      // std::cout << "[DEBUG] [RLMimicController::computeObservation] "
      //           << "raw obs term: " << obs_term.transpose() << std::endl;
      obs_term *= cfg_.obs_terms[i].scale;
      obs_term = obs_term.min(cfg_.obs_terms[i].ub).max(cfg_.obs_terms[i].lb);
      obs_term_stacks_[i].push_back(obs_term);
    }
  } else {
    obs_term_stacks_.pop_front();
    std::deque<Eigen::ArrayXd> single_obs;
    for (int i = 0; i < cfg_.obs_terms.size(); i++) {
      auto obs_term = get_obs_term(cfg_.obs_terms[i].name);
      obs_term = obs_term.min(cfg_.obs_terms[i].ub).max(cfg_.obs_terms[i].lb);
      single_obs.push_back(obs_term);
    }
    obs_term_stacks_.push_back(single_obs);
  }
  int obs_term_addr = 0;
  for (int i = 0; i < obs_term_stacks_.size(); i++) {
    for (int j = 0; j < obs_term_stacks_[i].size(); j++) {
      policy_obs_.segment(obs_term_addr, obs_term_stacks_[i][j].size()) =
          obs_term_stacks_[i][j];
      obs_term_addr += obs_term_stacks_[i][j].size();
    }
  }
  // 发布观测调试数据
  if (logger_) {
    for (int i = 0; i < cfg_.obs_terms.size(); i++) {
      auto term = get_obs_term(cfg_.obs_terms[i].name);
      std::vector<double> v(term.data(), term.data() + term.size());
      logger_->publishVector("/" + name_ + "/obs/" + cfg_.obs_terms[i].name, v);
    }
    std::vector<double> obs_vec(policy_obs_.data(), policy_obs_.data() + policy_obs_.size());
    logger_->publishVector("/" + name_ + "/observations", obs_vec);
  }
}

void RLMimicController::computeActions() {
  if (!model_ || !model_->isLoaded()) {
    std::cerr << "[ERROR] [RLMimicController::computeActions] "
              << "Model not loaded" << std::endl;
    return;
  }

  // double → float（模型输入）
  std::vector<float> obs_vec(policy_obs_shape_);
  for (int i = 0; i < policy_obs_shape_; ++i) {
    obs_vec[i] = static_cast<float>(policy_obs_[i]);
  }

  // 执行推理
  std::vector<float> action_vec = model_->forward(obs_vec);

  // 检查输出大小
  if (action_vec.size() != static_cast<size_t>(policy_joint_count_)) {
    std::cerr << "[ERROR] [RLMimicController::computeActions] "
              << "Action size mismatch: got " << action_vec.size()
              << ", expected " << policy_joint_count_ << std::endl;
    return;
  }

  // float → double（内部使用）
  for (int i = 0; i < policy_joint_count_; ++i) {
    policy_action_[i] = static_cast<double>(action_vec[i]);
  }
}

void RLMimicController::updateRobotCmd() {
  RobotState robot_state = getRobotState();
  Eigen::ArrayXd policy_q(policy_joint_count_);
  Eigen::ArrayXd policy_v(policy_joint_count_);
  for (int i = 0; i < policy_joint_count_; i++) {
    policy_q[i] = robot_state.q[policy_joint_ids_[i]];
    policy_v[i] = robot_state.v[policy_joint_ids_[i]];
  }
  Eigen::ArrayXd policy_q_target;
  if (cfg_.residual_action) {
    policy_q_target = cfg_.joint_direction *
                      (get_obs_motion_command().head(policy_joint_count_) +
                       policy_action_ * cfg_.action_scale);
  } else {
    policy_q_target = cfg_.joint_direction *
                      (cfg_.q_default + policy_action_ * cfg_.action_scale);
  }

  for (int i = 0; i < motor_count_; i++) {
    cmd_.modes[i] = 2;  // set default to position control
    cmd_.kp[i] = 100.;  // TODO load default value from config
    cmd_.kd[i] = 10.;
  }
  // 策略控制的关节：根据控制模式（CSP/CSV/CST）设置命令
  for (int i = 0; i < policy_joint_count_; i++) {
    int motor_id = policy_joint_ids_[i];
    cmd_.v[motor_id] = 0.;

    if (cfg_.control_mode[i] == 2) {
      // CSP: 软件PD计算力矩，通过前馈力矩发送，位置设为当前值
      cmd_.q[motor_id] = policy_q[i];
      cmd_.tau[motor_id] = cfg_.kp[i] * (policy_q_target[i] - policy_q[i]);

      // // CSP: 纯粹的位置控制
      // cmd_.q[motor_id] = policy_q_target[i];
      // cmd_.tau[motor_id] = 0.0;
    } else if (cfg_.control_mode[i] == 1) {
      // CSV: 位置误差 → 力矩
      cmd_.q[motor_id] = policy_q[i];
      cmd_.tau[motor_id] = cfg_.kp[i] * (policy_q_target[i] - policy_q[i]);
    } else {
      // CST: PD 控制 → 力矩
      cmd_.q[motor_id] = policy_q[i];
      cmd_.tau[motor_id] = cfg_.kp[i] * (policy_q_target[i] - policy_q[i]) +
                           cfg_.kd[i] * (-policy_v[i]);
    }
    cmd_.modes[motor_id] = cfg_.control_mode[i] == 0 ? 0 : 2;
    if (cfg_.control_mode[i] == 0) {
      ////////////////////////////////////////////////
      // mode 0 (CST): 软件已计算完整PD力矩，硬件层不需要再做PD反馈
      // 硬件控制律: final = tau + kp*(q_cmd-q) + kd*(v_cmd-v)
      // 设 kp=kd=0 避免双重阻尼
      /////////////////////////////////////////////////
      cmd_.kp[motor_id] = 0.;
      cmd_.kd[motor_id] = 0.;
    } else {
      cmd_.kp[motor_id] = cfg_.kp[i];
      cmd_.kd[motor_id] = cfg_.kd[i];
    }
    cmd_.timestamp = leju::common::GetUnixTimestampS();
  }
  // 发布调试数据
  if (logger_) {
    std::vector<double> qt(policy_q_target.data(), policy_q_target.data() + policy_q_target.size());
    logger_->publishVector("/" + name_ + "/q_target", qt);
    logger_->publishValue("/" + name_ + "/motion_frame",
                          static_cast<double>(policy_phase_cnt_));
  }
}

Eigen::ArrayXd RLMimicController::get_obs_term(const std::string& name) {
  // std::cout << "[DEBUG] [RLMimicController::get_obs_term] "
  //           << "name = " << name << std::endl;
  if (name == "base_ang_vel") {
    return get_obs_base_ang_vel();
  } else if (name == "projected_gravity") {
    return get_obs_projected_gravity();
  } else if (name == "joint_pos") {
    return get_obs_joint_pos();
  } else if (name == "joint_vel") {
    return get_obs_joint_vel();
  } else if (name == "actions") {
    return get_obs_actions();
  } else if (name == "motion_command") {
    return get_obs_motion_command();
  } else if (name == "motion_target_height") {
    return get_obs_motion_target_height();
  } else if (name == "motion_anchor_ori_b") {
    return get_obs_motion_anchor_ori_b();
  } else {
    std::cerr << "[ERROR] [RLMimicController::get_obs_term] "
              << "obs term not implemented: " << name << "." << std::endl;
    return Eigen::ArrayXd(0);
  }
  return Eigen::ArrayXd(0);
}

int RLMimicController::get_shape_obs_term(const std::string& name) {
  if (name == "base_ang_vel") {
    return get_shape_obs_base_ang_vel();
  } else if (name == "projected_gravity") {
    return get_shape_obs_projected_gravity();
  } else if (name == "joint_pos") {
    return get_shape_obs_joint_pos();
  } else if (name == "joint_vel") {
    return get_shape_obs_joint_vel();
  } else if (name == "actions") {
    return get_shape_obs_actions();
  } else if (name == "motion_command") {
    return get_shape_obs_motion_command();
  } else if (name == "motion_target_height") {
    return get_shape_obs_motion_target_height();
  } else if (name == "motion_anchor_ori_b") {
    return get_shape_obs_motion_anchor_ori_b();
  } else {
    std::cerr << "[ERROR] [RLMimicController::get_shape_obs_term] "
              << "obs term not implemented: " << name << "." << std::endl;
    return -1;
  }
  return -1;
}

Eigen::ArrayXd RLMimicController::get_obs_base_ang_vel() {
  ImuData imu_data = getImuData();
  Eigen::ArrayXd ans(3);
  ans << imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2];
  return ans;
}

Eigen::ArrayXd RLMimicController::get_obs_projected_gravity() {
  ImuData imu_data = getImuData();
  Eigen::ArrayXd ans(3);
  Eigen::Vector3d g_hat(0., 0., -1.);
  Eigen::Quaterniond quat(imu_data.quat[0], imu_data.quat[1], imu_data.quat[2],
                          imu_data.quat[3]);
  ans = (quat.inverse() * g_hat).array();
  return ans;
}

Eigen::ArrayXd RLMimicController::get_obs_joint_pos() {
  RobotState robot_state = getRobotState();
  Eigen::ArrayXd policy_q(policy_joint_count_);
  for (int i = 0; i < policy_joint_count_; i++) {
    policy_q(i) = robot_state.q[policy_joint_ids_[i]];
  }
  Eigen::ArrayXd ans(policy_joint_count_);
  ans = cfg_.joint_direction * (policy_q - cfg_.q_default);
  return ans;
}

Eigen::ArrayXd RLMimicController::get_obs_joint_vel() {
  RobotState robot_state = getRobotState();
  Eigen::ArrayXd policy_v(policy_joint_count_);
  for (int i = 0; i < policy_joint_count_; i++) {
    policy_v(i) = robot_state.v[policy_joint_ids_[i]];
  }
  Eigen::ArrayXd ans(policy_joint_count_);
  ans = cfg_.joint_direction * policy_v;
  return ans;
}

Eigen::ArrayXd RLMimicController::get_obs_actions() { return policy_action_; }

Eigen::ArrayXd RLMimicController::get_obs_motion_command() {
  int phase_index = std::clamp(policy_phase_cnt_, 0, num_data_rows_ - 1);
  return motion_data_.row(phase_index).segment(7, policy_joint_count_ * 2);
}

Eigen::ArrayXd RLMimicController::get_obs_motion_target_height() {
  int phase_index = std::clamp(policy_phase_cnt_, 0, num_data_rows_ - 1);
  return motion_data_.row(phase_index).segment(2, 1);
}

Eigen::ArrayXd RLMimicController::get_obs_motion_anchor_ori_b() {
  int phase_index = std::clamp(policy_phase_cnt_, 0, num_data_rows_ - 1);
  Eigen::Quaterniond target_quat(
      motion_data_(phase_index, 3), motion_data_(phase_index, 4),
      motion_data_(phase_index, 5), motion_data_(phase_index, 6));
  ImuData imu_data = getImuData();
  Eigen::Quaterniond base_quat(imu_data.quat[0], imu_data.quat[1],
                               imu_data.quat[2], imu_data.quat[3]);
  auto dummy_world_quat =
      Eigen::AngleAxisd(dummy_world_yaw_, Eigen::Vector3d{0., 0., 1.});
  auto R_BaseTarget =
      ((dummy_world_quat.inverse() * base_quat).inverse() * target_quat)
          .matrix();
  // std::cout << "target_quat: " << target_quat.coeffs().transpose() <<
  // std::endl; std::cout << "base_quat: " << base_quat.coeffs().transpose() <<
  // std::endl;

  Eigen::ArrayXd ans(6);
  ans << R_BaseTarget(0, 0), R_BaseTarget(0, 1), R_BaseTarget(1, 0),
      R_BaseTarget(1, 1), R_BaseTarget(2, 0), R_BaseTarget(2, 1);
  // std::cout << "motion_anchor_ori_b:" << ans.transpose() << std::endl;
  return ans;
}

int RLMimicController::get_shape_obs_base_ang_vel() const { return 3; }

int RLMimicController::get_shape_obs_projected_gravity() const { return 3; }

int RLMimicController::get_shape_obs_joint_pos() const {
  return policy_joint_count_;
}

int RLMimicController::get_shape_obs_joint_vel() const {
  return policy_joint_count_;
}

int RLMimicController::get_shape_obs_actions() const {
  return policy_joint_count_;
}

int RLMimicController::get_shape_obs_motion_command() const {
  return 2 * policy_joint_count_;
}

int RLMimicController::get_shape_obs_motion_target_height() const { return 1; }

int RLMimicController::get_shape_obs_motion_anchor_ori_b() const { return 6; }

RLMimicController::RLMimicController(const RobotVersion& version)
    : robot_version_(version), motor_count_(0) {}

RLMimicController::~RLMimicController() = default;

bool RLMimicController::initialize(const std::string& config_file) {
  std::cout << "[DEBUG] [RLMimicController::initialize] "
            << "func start." << std::endl;

  // Store config file path
  config_file_path_ = config_file;

  if (!GlobalRobot::init_env(robot_version_)) {
    std::cerr << "[ERROR] [RLMimicController::initialize] "
              << "GlobalRobot::init_env() failed." << std::endl;
    return false;
  }

  auto& robot = GlobalRobot::getInstance();
  motor_count_ = robot.getMotorNumber();
  motor_names_ = robot.getMotorNames();

  cmd_.resize(motor_count_);

  printRobotInfo();
  setupSubscriptions();

  load_cfg(config_file);
  load_policy();
  load_motion_data();

  logger_ = TopicLogger::create();

  std::cout << "[DEBUG] [RLMimicController::initialize] "
            << "func end." << std::endl;
  return true;
}

void RLMimicController::start() {
  std::cout << "[DEBUG] [RLMimicController::start] "
            << "func start." << std::endl;
  std::cout << "[INFO] [RLMimicController::start] "
            << "wait for data available." << std::endl;
  while (true) {
    int imu_data_msg_cnt;
    {
      std::lock_guard<std::mutex> lock(imu_data_mutex_);
      imu_data_msg_cnt = imu_data_msg_cnt_;
    }
    if (imu_data_msg_cnt > 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "imu data be patient... \n";
    std::cout.flush();
  }
  while (true) {
    int hw_state_msg_cnt;
    {
      std::lock_guard<std::mutex> lock(hw_state_mutex_);
      hw_state_msg_cnt = hw_state_msg_cnt_;
    }
    if (hw_state_msg_cnt > 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "hw state be patient... \n";
    std::cout.flush();
  }
  while (true) {
    int robot_state_msg_cnt;
    {
      std::lock_guard<std::mutex> lock(robot_state_mutex_);
      robot_state_msg_cnt = hw_state_msg_cnt_;
    }
    if (robot_state_msg_cnt > 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "robot state be patient... \n";
    std::cout.flush();
  }
  while (true) {
    int joy_data_msg_cnt;
    {
      std::lock_guard<std::mutex> lock(joy_data_mutex_);
      joy_data_msg_cnt = joy_data_msg_cnt_;
    }
    if (joy_data_msg_cnt > 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "joy data be patient... may be you forget to run joystick?\n";
    std::cout.flush();
  }
  std::cout << std::endl;
  std::cout << "[INFO] [RLMimicController::start] "
            << "data available now." << std::endl;

  while (true) {
    if (getHardwareState() == leju::HardwareState::READY_OK) break;
    std::cout << "wait for hardware state ready..." << std::endl;
  }

  std::cout
      << "[INFO] [RLMimicController::start] robot move to joint default pos"
      << std::endl;
  // TODO wait for data available, use a flag
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::vector<double> joint_target_pos(motor_count_);
  for (int i = 0; i < policy_joint_count_; i++) {
    joint_target_pos[policy_joint_ids_[i]] =
        cfg_.joint_direction[i] *
        motion_data_.row(0).segment(7, policy_joint_count_)[i];
  }
  // std::cout << "[DEBUG] [RLMimicController::start] cfg_.joint_direction: " <<
  // cfg_.joint_direction.transpose() << std::endl; std::cout << "[DEBUG]
  // [RLMimicController::start] cfg_.q_default: " << cfg_.q_default.transpose()
  // << std::endl; std::cout << "policy_joint_count_: " << policy_joint_count_
  // << std::endl;
  jointMoveTo(joint_target_pos, 3.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::cout << "[INFO] [RLMimicController::start] joint default pos reached."
            << std::endl;

  std::cout << std::endl;
  std::cout
      << "\033[1;32m------------------------------------------------\033[0m"
      << std::endl;
  std::cout
      << "\033[1;32m- press gamepad button `start` to run rl policy.  \033[0m"
      << std::endl;
  std::cout
      << "\033[1;32m------------------------------------------------\033[0m"
      << std::endl;

  while (true) {
    bool press_start = false;
    {
      std::lock_guard<std::mutex> lock(joy_data_mutex_);
      press_start = buttons_press_.start > 0;
    }
    if (press_start) {
      {
        std::lock_guard<std::mutex> lock(joy_data_mutex_);
        buttons_press_.start = 0;
      }
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  dummy_world_yaw_ = 0.;
  Eigen::Quaterniond target_quat(motion_data_(0, 3), motion_data_(0, 4),
                                 motion_data_(0, 5), motion_data_(0, 6));
  ImuData imu_data = getImuData();
  Eigen::Quaterniond base_quat(imu_data.quat[0], imu_data.quat[1],
                               imu_data.quat[2], imu_data.quat[3]);
  // Eigen::AngleAxisd R_BaseTarget =
  //     Eigen::AngleAxisd(base_quat.inverse() * target_quat);
  Eigen::AngleAxisd R_BaseTarget =
      Eigen::AngleAxisd(target_quat * base_quat.inverse());
  if (R_BaseTarget.axis()(2) > 0.) {
    dummy_world_yaw_ = -R_BaseTarget.angle();
  } else {
    dummy_world_yaw_ = R_BaseTarget.angle();
  }
  if (std::abs(R_BaseTarget.axis()(2)) < 0.7) {
    std::cout << "[WARN] [RLMimicController::start] "
              << "detact init base pitch or roll deviation." << std::endl;
  }
  std::cout << "[DEBUG] [RLMimicController::start] "
            << "func end." << std::endl;
}

void RLMimicController::mainLoop() {
  std::cout << "\n--- Starting Control Loop ---" << std::endl;
  std::cout << "Press Ctrl+C to stop" << std::endl;

  auto& robot = GlobalRobot::getInstance();
  auto start_time = std::chrono::steady_clock::now();

  int cnt = 0;
  auto t0 = std::chrono::steady_clock::now();
  while (true) {
    bool press_guide = false;
    {
      std::lock_guard<std::mutex> lock(joy_data_mutex_);
      press_guide = buttons_press_.guide > 0;
    }
    if (press_guide) {
      {
        std::lock_guard<std::mutex> lock(joy_data_mutex_);
        buttons_press_.guide = 0;
      }
      start_play_motion_ = true;
    }

    // get data
    if (cnt % decimation_ == 0) {
      // std::cout << "start update observation." << std::endl;
      computeObservation();
      // std::cout << "start update action." << std::endl;
      computeActions();
      if (start_play_motion_) {
        policy_phase_cnt_++;
      }
    }
    // std::cout << "start update robot cmd." << std::endl;
    updateRobotCmd();
    // std::cout << "end update robot cmd." << std::endl;

    // publish command;
    bool success = robot.publishRobotCmd(cmd_);
    if (!success) {
      std::cout << "Failed to publish cmd." << std::endl;
    } else {
      // std::cout << "Sent joint command:" << cmd_ << std::endl;
      ;
    }
    auto joy_data = getJoyData();
    if (joy_data.buttons.back) {
      std::cout << "Joystick button `back` pressed, exiting..." << std::endl;
      break;
    }

    cnt++;
    // TODO use loop rate sleep
    t0 += std::chrono::milliseconds(static_cast<int>(1000 * cfg_.loop_dt));
    std::this_thread::sleep_until(t0);
    // std::this_thread::sleep_for(
    //     std::chrono::milliseconds(static_cast<int>(1000 * cfg_.loop_dt)));
  }
  for (int i = 0; i < 50; i++) {
    RobotCmd cmd;
    cmd.resize(motor_count_);
    robot.publishRobotCmd(cmd);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1000 * cfg_.loop_dt)));
  }

  // Stop Robot
  robot.publishStopRobot();
  robot.publishStopRobot();
  robot.publishStopRobot();

  robot.shutdown();
  std::cout << "[DEBUG] [RLMimicController::mainLoop] func end." << std::endl;
  return;
}

}  // namespace rl_mimic
}  // namespace leju