#include "leju-rl-controller/controllers/rl_demo_controller.h"

#include <Eigen/Dense>
#include <filesystem>

namespace leju {
namespace rl_demo {

bool RLDemoConfig::loadFromYaml(YAML::Node root_node) {
  YAML::Node humanoid_cfg = root_node["HumanoidRobotCfg"];
  loop_dt = humanoid_cfg["loop_dt"].as<double>();
  policy_path = humanoid_cfg["policy_path"].as<std::string>();

  policy_dt = humanoid_cfg["env"]["policy_dt"].as<double>();

  YAML::Node robot = humanoid_cfg["env"]["robot"];
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

  joint_direction = Eigen::Map<const Eigen::ArrayXd>(joint_direction_vec.data(),
                                                     joint_direction_vec.size());
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
  YAML::Node obsterms = obs["terms"];
  for (const auto& pair : obsterms) {
    std::string name = pair.first.as<std::string>();
    double scale = pair.second["scale"].as<double>();
    std::vector<double> clip_range =
        pair.second["clip"].as<std::vector<double>>();
    obs_terms.emplace_back(
        ObsTermConfig{name, scale, clip_range[0], clip_range[1]});
  }

  YAML::Node command_range = humanoid_cfg["env"]["command_range"];
  std::vector<double> range;
  range = command_range["lin_vel_x"].as<std::vector<double>>();
  command_range_lin_vel_x_lb = range[0];
  command_range_lin_vel_x_ub = range[1];
  range = command_range["lin_vel_y"].as<std::vector<double>>();
  command_range_lin_vel_y_lb = range[0];
  command_range_lin_vel_y_ub = range[1];
  range = command_range["ang_vel_z"].as<std::vector<double>>();
  command_range_ang_vel_z_lb = range[0];
  command_range_ang_vel_z_ub = range[1];
  return true;
}

void RLDemoController::printRobotInfo() const {
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

void RLDemoController::setupSubscriptions() {
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

void RLDemoController::imuCallback(const ImuDataConstPtr& imu_data) {
  std::lock_guard<std::mutex> imu_lock(imu_data_mutex_);
  imu_data_ = *imu_data;
  imu_data_msg_cnt_++;
}

void RLDemoController::jointStateCallback(
    const RobotStateConstPtr& robot_state) {
  std::lock_guard<std::mutex> state_lock(robot_state_mutex_);
  robot_state_ = *robot_state;
  robot_state_msg_cnt_++;
}

void RLDemoController::hwStateCallback(const StringDataConstPtr& hw_state) {
  std::lock_guard<std::mutex> lock(hw_state_mutex_);
  hw_state_ = leju::String2HwState(hw_state->data);
  hw_state_msg_cnt_++;
}

void RLDemoController::joyDataCallback(const JoyDataConstPtr& joy) {
  std::lock_guard<std::mutex> joy_lock(joy_data_mutex_);
  joy_prev_ = joy_;
  joy_ = *joy;
// for (int i = 0; i < 16; i++) {
//   if (joy_.buttons[i] && (!joy_prev_.buttons[i])) {
//     buttons_press_[i]++;
//   }
// }
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

bool RLDemoController::load_cfg(const std::string& config_file) {
  YAML::Node cfg_node = YAML::LoadFile(config_file);
  cfg_.loadFromYaml(cfg_node);

  // Load cmd_stance config (optional)
  YAML::Node humanoid_cfg = cfg_node["HumanoidRobotCfg"];
  if (humanoid_cfg["cmd_stance"]) {
    CmdStanceConfig cs_cfg;
    auto cs_node = humanoid_cfg["cmd_stance"];
    if (cs_node["smart_stop"]) {
      cs_cfg.smart_stop_enabled =
          cs_node["smart_stop"]["enabled"].as<bool>(true);
      cs_cfg.torso_velocity_threshold =
          cs_node["smart_stop"]["torso_velocity_threshold"]
              .as<double>(0.05);
      cs_cfg.feet_alignment_threshold =
          cs_node["smart_stop"]["feet_alignment_threshold"]
              .as<double>(0.08);
    }
    if (cs_node["velocity_magnitude_threshold"]) {
      cs_cfg.velocity_magnitude_threshold =
          cs_node["velocity_magnitude_threshold"].as<double>(0.01);
    }
    cmd_stance_calculator_.setConfig(cs_cfg);
  }

  decimation_ = std::round(cfg_.policy_dt / cfg_.loop_dt);
  if (decimation_ != cfg_.policy_dt / cfg_.loop_dt) {
    std::cerr << "[WARN] [RLDemoController::load_cfg] "
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
      std::cerr << "[ERROR] [RLDemoController::initialize] "
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

  // Load arm_controller config (optional)
  if (humanoid_cfg["arm_controller"]) {
    ArmControllerConfig ac_cfg;
    auto ac_node = humanoid_cfg["arm_controller"];
    ac_cfg.enabled = ac_node["enabled"].as<bool>(true);
    ac_cfg.interpolation_velocity =
        ac_node["interpolation_velocity"].as<double>(1.0);
    ac_cfg.min_duration = ac_node["min_duration"].as<double>(0.2);
    ac_cfg.max_duration = ac_node["max_duration"].as<double>(2.0);
    arm_controller_.setConfig(ac_cfg);

    int leg_count = ac_node["leg_joint_count"].as<int>(12);
    int waist_count = ac_node["waist_joint_count"].as<int>(0);
    arm_start_index_ = leg_count + waist_count;
    arm_joint_count_ = policy_joint_count_ - arm_start_index_;
    if (arm_joint_count_ > 0 && arm_start_index_ < policy_joint_count_) {
      Eigen::VectorXd default_arm(arm_joint_count_);
      for (int i = 0; i < arm_joint_count_; i++) {
        default_arm[i] = cfg_.q_default[arm_start_index_ + i];
      }
      arm_controller_.init(arm_start_index_, arm_joint_count_, default_arm);
    }
  }

  // Load hardware_override_kp_kd (optional):
  // - v52 等有腰部的版本：15 个值，覆盖 左腿6+右腿6+腰1+左臂第1+右臂第1
  // - v46 等无腰部的版本：14 个值，覆盖 左腿6+右腿6+左臂第1+右臂第1
  if (humanoid_cfg["hardware_override_kp_kd"]) {
    auto kp_vec = humanoid_cfg["hardware_override_kp_kd"]["kp"].as<std::vector<double>>();
    auto kd_vec = humanoid_cfg["hardware_override_kp_kd"]["kd"].as<std::vector<double>>();
    if ((kp_vec.size() == 15u && kd_vec.size() == 15u) ||
        (kp_vec.size() == 14u && kd_vec.size() == 14u)) {
      hardware_override_kp_15_ = std::move(kp_vec);
      hardware_override_kd_15_ = std::move(kd_vec);
    }
  }

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

bool RLDemoController::load_policy() {
  // Extract config directory from config file path using filesystem
  std::filesystem::path config_path(config_file_path_);
  std::string policy_path =
      (config_path.parent_path() / cfg_.policy_path).string();

  compiled_model_ = core_.compile_model(policy_path, "CPU");
  input_port_ = compiled_model_.input();
  output_port_ = compiled_model_.output();
  if (input_port_.get_shape().size() != 2) {
    std::cerr << "[ERROR] [RLDemoController::load_policy] "
              << "input_port_ shape size != 2, "
              << "shape size: " << input_port_.get_shape().size() << "."
              << std::endl;
    return false;
  }
  if (input_port_.get_shape()[0] != 1) {
    std::cerr << "[ERROR] [RLDemoController::load_policy] "
              << "input_port_ shape[0] != 1, "
              << "shape[0]: " << input_port_.get_shape()[0] << "." << std::endl;
    return false;
  }
  if (input_port_.get_shape()[1] != policy_obs_shape_) {
    std::cerr << "[ERROR] [RLDemoController::load_policy] "
              << "input_port_ shape[1] != policy_joint_count_, "
              << "shape[1]: " << input_port_.get_shape()[1]
              << ", policy_obs_shape_: " << policy_obs_shape_ << "."
              << std::endl;
    return false;
  }
  if (output_port_.get_shape().size() != 2) {
    std::cerr << "[ERROR] [RLDemoController::load_policy] "
              << "output_port_ shape size != 2, "
              << "shape size: " << output_port_.get_shape().size() << "."
              << std::endl;
    return false;
  }
  if (output_port_.get_shape()[0] != 1) {
    std::cerr << "[ERROR] [RLDemoController::load_policy] "
              << "output_port_ shape[0] != 1, "
              << "shape[0]: " << output_port_.get_shape()[0] << "."
              << std::endl;
    return false;
  }
  if (output_port_.get_shape()[1] != policy_joint_count_) {
    std::cerr << "[ERROR] [RLDemoController::load_policy] "
              << "output_port_ shape[1] != policy_joint_count_, "
              << "shape[1]: " << output_port_.get_shape()[1]
              << ", policy_joint_count_: " << policy_joint_count_ << "."
              << std::endl;
    return false;
  }
  input_tensor_ =
      ov::Tensor(input_port_.get_element_type(), input_port_.get_shape());
  output_tensor_ =
      ov::Tensor(output_port_.get_element_type(), output_port_.get_shape());
  infer_request_ = compiled_model_.create_infer_request();
  infer_request_.set_input_tensor(input_tensor_);
  infer_request_.set_output_tensor(output_tensor_);
  return true;
}

void RLDemoController::jointMoveTo(const std::vector<double>& joint_target_pos,
                                   double elapse) {
  std::cout << "[DEBUG] [RLDemoController::jointMoveTo] "
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
    //////////////////////////////////////////////
    // DEBUG: ROBAN 系列: EC电机 CSP PDO 的 kp/kd 参数
    // 从 kuavo.json 复制
    if (IS_ROBAN_LEGGED(robot_version_)) {
      constexpr double ec_motor_kp[] = {200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200};
      constexpr double ec_motor_kd[] = {5000, 5000, 5000, 5000, 10000, 10000, 5000, 5000, 5000, 5000, 10000, 10000, 5000};
      for (int i = 0; i < 13; i++) {
        kp[i] = ec_motor_kp[i];
        kd[i] = ec_motor_kd[i];
      }
    }
    /////////////////////////////////////////////////
    RobotCmd cmd(motor_count_);
    cmd.q = joint_demand_pos;
    cmd.kp = kp;
    cmd.kd = kd;
    cmd.modes = mode;
    cmd.v = joint_demand_vel;
    cmd.tau = joint_demand_tau;
    applyHardwareKpKdOverride(cmd);
    robot.publishRobotCmd(cmd);
    // TODO use loop rate sleep
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(cfg_.loop_dt * 1000)));
  }
}

void RLDemoController::applyHardwareKpKdOverride(RobotCmd& cmd) {
  // 需要有足够长度的 kp/kd（至少覆盖到右臂第 1 个关节所在的 motor 索引 20）
  if (hardware_override_kp_15_.empty() ||
      hardware_override_kp_15_.size() != hardware_override_kd_15_.size() ||
      cmd.kp.size() < 21u || cmd.kd.size() < 21u) {
    return;
  }

  if (hardware_override_kp_15_.size() == 15u) {
    // 有腰部的版本（如 52）：
    // 15 个值对应：左腿6(0-5) + 右腿6(6-11) + 腰1(12) + 左臂第1(13) + 右臂第1(20，左臂7个之后)
    // 索引 14-19 左臂其余、21-26 右臂其余、27-28 头部：不覆盖，保持 cmd 原值（主循环中头部为 updateRobotCmd 的默认 100/10）
    const size_t kOverrideIndices15[15] = {0, 1, 2, 3, 4, 5,
                                           6, 7, 8, 9, 10, 11,
                                           12, 13, 20};
    for (size_t i = 0; i < 15u; i++) {
      size_t j = kOverrideIndices15[i];
      cmd.kp[j] = hardware_override_kp_15_[i];
      cmd.kd[j] = hardware_override_kd_15_[i];
    }
  } else if (hardware_override_kp_15_.size() == 14u) {
    // 无腰部的版本（如 46）：
    // 14 个值对应：左腿6(0-5) + 右腿6(6-11) + 左臂第1(12) + 右臂第1(19)
    // 腰部电机（索引 12）在该版本不存在，不做覆盖。
    const size_t kOverrideIndices14[14] = {0, 1, 2, 3, 4, 5,
                                           6, 7, 8, 9, 10, 11,
                                           12, 19};
    for (size_t i = 0; i < 14u; i++) {
      size_t j = kOverrideIndices14[i];
      cmd.kp[j] = hardware_override_kp_15_[i];
      cmd.kd[j] = hardware_override_kd_15_[i];
    }
  }
}

void RLDemoController::computeObservation() {
  if (cfg_.stack_order_is_isaaclab) {
    for (int i = 0; i < cfg_.obs_terms.size(); i++) {
      obs_term_stacks_[i].pop_front();
      auto obs_term = get_obs_term(cfg_.obs_terms[i].name);
      // std::cout << "[DEBUG] [RLDemoController::computeObservation] "
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

void RLDemoController::computeActions() {
  if (input_tensor_.get_element_type() == ov::element::f32) {
    for (int i = 0; i < policy_obs_shape_; i++) {
      input_tensor_.data<float>()[i] = policy_obs_[i];
    }
  } else if (input_tensor_.get_element_type() == ov::element::f64) {
    for (int i = 0; i < policy_obs_shape_; i++) {
      input_tensor_.data<double>()[i] = policy_obs_[i];
    }
  } else {
    std::cerr << "[ERROR] [RLDemoController::computeActions] "
              << "policy input cast to element type not implemented: "
              << input_tensor_.get_element_type() << "." << std::endl;
    return;
  }
  for (int i = 0; i < policy_obs_shape_; i++) {
    input_tensor_.data<float>()[i] = policy_obs_[i];
  }
  infer_request_.infer();
  // std::cout << "[DEBUG] [RLDemoController::computeActions] after infer" <<
  // std::endl;
  if (output_tensor_.get_element_type() == ov::element::f32) {
    Eigen::ArrayXf policy_action_32(policy_joint_count_);
    policy_action_32 = Eigen::Map<const Eigen::ArrayXf>(
        output_tensor_.data<float>(), policy_joint_count_);
    policy_action_ = policy_action_32.cast<double>();
  } else if (output_tensor_.get_element_type() == ov::element::f64) {
    policy_action_ = Eigen::Map<const Eigen::ArrayXd>(
        output_tensor_.data<double>(), policy_joint_count_);
  } else {
    std::cerr << "[ERROR] [RLDemoController::computeActions] "
              << "policy output from element type not implemented: "
              << output_tensor_.get_element_type() << "." << std::endl;
    return;
  }
  // std::cout << "observation: " << policy_obs_.transpose() << std::endl;
  // std::cout << "action: " << policy_action_.transpose() << std::endl;
}

void RLDemoController::updateRobotCmd() {
  RobotState robot_state = getRobotState();
  Eigen::ArrayXd policy_q(policy_joint_count_);
  Eigen::ArrayXd policy_v(policy_joint_count_);
  for (int i = 0; i < policy_joint_count_; i++) {
    policy_q[i] = robot_state.q[policy_joint_ids_[i]];
    policy_v[i] = robot_state.v[policy_joint_ids_[i]];
  }
  Eigen::ArrayXd policy_q_target =
      cfg_.joint_direction * (cfg_.q_default + policy_action_ * cfg_.action_scale);
  for (int i = 0; i < motor_count_; i++) {
    cmd_.modes[i] = 2;  // set default to position control
    cmd_.kp[i] = 100.;  // TODO load default value from config
    cmd_.kd[i] = 10.;
  }
  for (int i = 0; i < policy_joint_count_; i++) {
    int motor_id = policy_joint_ids_[i];
    cmd_.v[motor_id] = 0.;

    if (cfg_.control_mode[i] == 2) {
      // CSP: 软件PD计算力矩，通过前馈力矩发送，位置设为当前值
      cmd_.q[motor_id] = policy_q[i];
      cmd_.tau[motor_id] = cfg_.kp[i] * (policy_q_target[i] - policy_q[i]);
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

  // 手臂控制器：站立时平滑插值到默认姿态，行走时使用 RL 输出
  if (arm_joint_count_ > 0) {
    double cmd_stance = get_obs_cmd_stance()(0);
    Eigen::VectorXd current_arm_pos(arm_joint_count_);
    Eigen::VectorXd current_arm_vel(arm_joint_count_);
    for (int i = 0; i < arm_joint_count_; i++) {
      current_arm_pos[i] = policy_q[arm_start_index_ + i];
      current_arm_vel[i] = policy_v[arm_start_index_ + i];
    }
    Eigen::VectorXd desire_arm_q, desire_arm_v;
    if (arm_controller_.update(cmd_stance, current_arm_pos, current_arm_vel,
                              std::chrono::steady_clock::now(), &desire_arm_q,
                              &desire_arm_v)) {
      for (int i = 0; i < arm_joint_count_; i++) {
        int policy_idx = arm_start_index_ + i;
        int motor_idx = policy_joint_ids_[policy_idx];
        cmd_.q[motor_idx] = desire_arm_q[i];
        cmd_.v[motor_idx] = desire_arm_v[i];
        if (cfg_.control_mode[policy_idx] == 2) {
          cmd_.tau[motor_idx] = 0.;
        } else {
          cmd_.tau[motor_idx] =
              cfg_.kp[policy_idx] * (desire_arm_q[i] - policy_q[policy_idx]);
        }
      }
    }
  }
  // std::cout << "[DEBUG] [RLDemoController::updateRobotCmd] "
  //           << "policy_q: " << policy_q.transpose() << std::endl;
  // std::cout << "[DEBUG] [RLDemoController::updateRobotCmd] "
  //           << "policy_v: " << policy_v.transpose() << std::endl;
  // std::cout << "[DEBUG] [RLDemoController::updateRobotCmd] "
  //           << "policy_q_target: " << policy_q_target.transpose() <<
  //           std::endl;

  if (logger_) {
    std::vector<double> qt(policy_q_target.data(),
                           policy_q_target.data() + policy_q_target.size());
    logger_->publishVector("/" + name_ + "/q_target", qt);
  }
}

Eigen::ArrayXd RLDemoController::get_obs_term(const std::string& name) {
  // std::cout << "[DEBUG] [RLDemoController::get_obs_term] "
  //           << "name = " << name << std::endl;
  if (name == "base_ang_vel") {
    return get_obs_base_ang_vel();
  } else if (name == "projected_gravity") {
    return get_obs_projected_gravity();
  } else if (name == "velocity_commands") {
    return get_obs_velocity_commands();
  } else if (name == "cmd_stance") {
    return get_obs_cmd_stance();
  } else if (name == "joint_pos") {
    return get_obs_joint_pos();
  } else if (name == "joint_vel") {
    return get_obs_joint_vel();
  } else if (name == "actions") {
    return get_obs_actions();
  } else {
    std::cerr << "[ERROR] [RLDemoController::get_obs_term] "
              << "obs term not implemented: " << name << "." << std::endl;
    return Eigen::ArrayXd(0);
  }
  return Eigen::ArrayXd(0);
}

int RLDemoController::get_shape_obs_term(const std::string& name) {
  if (name == "base_ang_vel") {
    return get_shape_obs_base_ang_vel();
  } else if (name == "projected_gravity") {
    return get_shape_obs_projected_gravity();
  } else if (name == "velocity_commands") {
    return get_shape_obs_velocity_commands();
  } else if (name == "cmd_stance") {
    return get_shape_obs_cmd_stance();
  } else if (name == "joint_pos") {
    return get_shape_obs_joint_pos();
  } else if (name == "joint_vel") {
    return get_shape_obs_joint_vel();
  } else if (name == "actions") {
    return get_shape_obs_actions();
  } else {
    std::cerr << "[ERROR] [RLDemoController::get_shape_obs_term] "
              << "obs term not implemented: " << name << "." << std::endl;
    return -1;
  }
  return -1;
}

Eigen::ArrayXd RLDemoController::get_obs_base_ang_vel() {
  ImuData imu_data = getImuData();
  Eigen::ArrayXd ans(3);
  ans << imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2];
  return ans;
}

Eigen::ArrayXd RLDemoController::get_obs_projected_gravity() {
  ImuData imu_data = getImuData();
  Eigen::ArrayXd ans(3);
  Eigen::Vector3d g_hat(0., 0., -1.);
  Eigen::Quaterniond quat(imu_data.quat[0], imu_data.quat[1], imu_data.quat[2],
                          imu_data.quat[3]);
  ans = (quat.inverse() * g_hat).array();
  return ans;
}

Eigen::ArrayXd RLDemoController::get_obs_velocity_commands() {
  JoyData joy = getJoyData();
  Eigen::ArrayXd ans(3);
  if (joy.axes.left_y <= 0.) {
    ans(0) = -joy.axes.left_y * cfg_.command_range_lin_vel_x_ub;
  } else {
    ans(0) = joy.axes.left_y * cfg_.command_range_lin_vel_x_lb;
  }
  if (joy.axes.left_x >= 0.) {
    ans(1) = -joy.axes.left_x * cfg_.command_range_lin_vel_y_ub;
  } else {
    ans(1) = joy.axes.left_x * cfg_.command_range_lin_vel_y_lb;
  }
  if (joy.axes.right_x >= 0.) {
    ans(2) = -joy.axes.right_x * cfg_.command_range_ang_vel_z_ub;
  } else {
    ans(2) = joy.axes.right_x * cfg_.command_range_ang_vel_z_lb;
  }
  return ans;
}

Eigen::ArrayXd RLDemoController::get_obs_cmd_stance() {
  Eigen::ArrayXd velocity_commands = get_obs_velocity_commands();
  Eigen::Vector3d vel(velocity_commands(0), velocity_commands(1),
                     velocity_commands(2));
  double cmd_stance = cmd_stance_calculator_.computeSimple(vel);
  Eigen::ArrayXd ans(1);
  ans(0) = cmd_stance;
  return ans;
}

Eigen::ArrayXd RLDemoController::get_obs_joint_pos() {
  RobotState robot_state = getRobotState();
  Eigen::ArrayXd policy_q(policy_joint_count_);
  for (int i = 0; i < policy_joint_count_; i++) {
    policy_q(i) = robot_state.q[policy_joint_ids_[i]];
  }
  Eigen::ArrayXd ans(policy_joint_count_);
  ans = cfg_.joint_direction * (policy_q - cfg_.q_default);
  return ans;
}

Eigen::ArrayXd RLDemoController::get_obs_joint_vel() {
  RobotState robot_state = getRobotState();
  Eigen::ArrayXd policy_v(policy_joint_count_);
  for (int i = 0; i < policy_joint_count_; i++) {
    policy_v(i) = robot_state.v[policy_joint_ids_[i]];
  }
  Eigen::ArrayXd ans(policy_joint_count_);
  ans = cfg_.joint_direction * policy_v;
  return ans;
}

Eigen::ArrayXd RLDemoController::get_obs_actions() { return policy_action_; }

int RLDemoController::get_shape_obs_base_ang_vel() const { return 3; }

int RLDemoController::get_shape_obs_projected_gravity() const { return 3; }

int RLDemoController::get_shape_obs_velocity_commands() const { return 3; }

int RLDemoController::get_shape_obs_cmd_stance() const { return 1; }

int RLDemoController::get_shape_obs_joint_pos() const {
  return policy_joint_count_;
}

int RLDemoController::get_shape_obs_joint_vel() const {
  return policy_joint_count_;
}

int RLDemoController::get_shape_obs_actions() const {
  return policy_joint_count_;
}

RLDemoController::RLDemoController(const RobotVersion& version)
    : robot_version_(version), motor_count_(0) {}

RLDemoController::~RLDemoController() = default;

bool RLDemoController::initialize(const std::string& config_file) {
  std::cout << "[DEBUG] [RLDemoController::initialize] "
            << "func start." << std::endl;

  // Store config file path
  config_file_path_ = config_file;

  if (!GlobalRobot::init_env(robot_version_)) {
    std::cerr << "[ERROR] [RLDemoController::initialize] "
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

  logger_ = TopicLogger::create();

  std::cout << "[DEBUG] [RLDemoController::initialize] "
            << "func end." << std::endl;
  return true;
}

void RLDemoController::start() {
  std::cout << "[DEBUG] [RLDemoController::start] "
            << "func start." << std::endl;
  std::cout << "[INFO] [RLDemoController::start] "
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
  std::cout << "[INFO] [RLDemoController::start] "
            << "data available now." << std::endl;

  while (true) {
    if (getHardwareState() == leju::HardwareState::READY_OK) break;
    std::cout << "wait for hardware state ready..." << std::endl;
  }

  std::cout
      << "[INFO] [RLDemoController::start] robot move to joint default pos"
      << std::endl;
  // TODO wait for data available, use a flag
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::vector<double> joint_target_pos(motor_count_);
  for (int i = 0; i < policy_joint_count_; i++) {
    joint_target_pos[policy_joint_ids_[i]] = cfg_.joint_direction[i] * cfg_.q_default[i];
  }
  // std::cout << "[DEBUG] [RLDemoController::start] cfg_.joint_direction: " << cfg_.joint_direction.transpose() << std::endl;
  // std::cout << "[DEBUG] [RLDemoController::start] cfg_.q_default: " << cfg_.q_default.transpose() << std::endl;
  // std::cout << "policy_joint_count_: " << policy_joint_count_ << std::endl;
  jointMoveTo(joint_target_pos, 3.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::cout << "[INFO] [RLDemoController::start] joint default pos reached."
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
  std::cout << "[DEBUG] [RLDemoController::start] "
            << "func end." << std::endl;
}

void RLDemoController::mainLoop() {
  std::cout << "\n--- Starting Control Loop ---" << std::endl;
  std::cout << "Press Ctrl+C to stop" << std::endl;

  auto& robot = GlobalRobot::getInstance();
  auto start_time = std::chrono::steady_clock::now();

  int cnt = 0;
  auto t0 = std::chrono::steady_clock::now();
  while (true) {
    // get data
    if (cnt % decimation_ == 0) {
      // std::cout << "start update observation." << std::endl;
      computeObservation();
      // std::cout << "start update action." << std::endl;
      computeActions();
    }
    // std::cout << "start update robot cmd." << std::endl;
    updateRobotCmd();
    // std::cout << "end update robot cmd." << std::endl;

    // publish command;
    applyHardwareKpKdOverride(cmd_);
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
    applyHardwareKpKdOverride(cmd);
    robot.publishRobotCmd(cmd);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1000 * cfg_.loop_dt)));
  }

  // Stop Robot
  robot.publishStopRobot();
  robot.publishStopRobot();
  robot.publishStopRobot();

  robot.shutdown();
  std::cout << "[DEBUG] [RLDemoController::mainLoop] func end." << std::endl;
  return;
}

}  // namespace rl_demo
}  // namespace leju