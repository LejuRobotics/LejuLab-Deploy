#include "leju-dummy-controller/joint_monkey_controller.h"
#include <yaml-cpp/yaml.h>
#include <stdio.h>

JointMonkeyController::JointMonkeyController(const RobotVersion& version)
    : robot_version_(version), motor_count_(0) {}

JointMonkeyController::~JointMonkeyController() = default;

bool JointMonkeyController::initialize() {
  if (!GlobalRobot::init_env(robot_version_)) {
    std::cerr << "Failed to call init_env" << std::endl;
    return false;
  }

  auto& robot = GlobalRobot::getInstance();
  motor_count_ = robot.getMotorNumber();
  motor_names_ = robot.getMotorNames();

  printRobotInfo();
  setupSubscriptions();

  load_cfg();

  std::cout << "✓ Kuavo robot initialized successfully" << std::endl;
  return true;
}

void JointMonkeyController::printRobotInfo() const {
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

void JointMonkeyController::setupSubscriptions() {
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

void JointMonkeyController::imuCallback(const ImuDataConstPtr& imu_data) {
  std::lock_guard<std::mutex> imu_lock(imu_data_mutex_);
  imu_data_ = *imu_data;
  imu_data_msg_cnt_++;
}

void JointMonkeyController::jointStateCallback(
    const RobotStateConstPtr& robot_state) {
  std::lock_guard<std::mutex> state_lock(robot_state_mutex_);
  robot_state_ = *robot_state;
  robot_state_msg_cnt_++;
}

void JointMonkeyController::hwStateCallback(
    const StringDataConstPtr& hw_state) {
  std::lock_guard<std::mutex> lock(hw_state_mutex_);
  hw_state_ = leju::String2HwState(hw_state->data);
  hw_state_msg_cnt_++;
}

void JointMonkeyController::joyDataCallback(const JoyDataConstPtr& joy) {
  std::lock_guard<std::mutex> joy_lock(joy_data_mutex_);
  joy_prev_ = joy_;
  joy_ = *joy;
  // for (int i = 0; i < 16; i++) {
  //   if (joy_.buttons[i] && (!joy_prev_.buttons[i])) {
  //     buttons_press_[i]++;
  //   }
  // }
  joy_data_msg_cnt_++;
  #define CALC_PRESS(name) {buttons_press_.name = joy_.buttons.name && !joy_prev_.buttons.name;}
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
}

bool JointMonkeyController::load_cfg() {
  std::string cfg_file =
      "src/leju-controllers/leju-dummy-controller/config/46/config.yaml";
  YAML::Node cfg_node = YAML::LoadFile(cfg_file);
  joint_range_ = cfg_node["HumanoidRobotCfg"]["joint_range"]
                     .as<std::vector<std::vector<double>>>();
  if (joint_range_.size() != motor_count_) {
    std::cerr << "[ERROR] [JointMonkeyController::load_cfg] "
              << "size of joint_range_ mismatch motor_count_: "
              << "joint_range_.size()=" << joint_range_.size()
              << ", motor_count_=" << motor_count_
              << std::endl;
    return false;
  }
  return true;
}

void JointMonkeyController::start() {
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
    std::cout << "be patient... ";
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
    std::cout << "be patient... ";
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
    std::cout << "be patient... ";
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
    std::cout << "be patient... may be you forget to run joystick?\n";
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
  for (int i = 0; i < motor_count_; i++) {
    joint_target_pos[i] = 0;
  }
  jointMoveTo(joint_target_pos, 3.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::cout << "[INFO] [RLDemoController::start] joint default pos reached."
            << std::endl;
  std::cout << "press gamepad button `start` to run rl policy." << std::endl;
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

void JointMonkeyController::jointMoveTo(
    const std::vector<double>& joint_target_pos, double elapse) {
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

  for (double t = 0.; t < elapse; t += 0.001) {
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
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1)));
  }
}

void JointMonkeyController::mainLoop() {
  std::cout << "\n--- Starting Control Loop ---" << std::endl;
  std::cout << "Press Ctrl+C to stop" << std::endl;

  auto& robot = GlobalRobot::getInstance();
  auto start_time = std::chrono::steady_clock::now();

  int joint_id = 0;
  while (true) {
    std::vector<double> joint_target_pos(motor_count_);
    joint_target_pos[joint_id] = joint_range_[joint_id][0];
    char tmp[256];
    std::cout << "move joint " << joint_id << " to " << joint_target_pos[joint_id]
              << ", press enter to continue:";
    std::cout.flush();
    getchar();
    jointMoveTo(joint_target_pos, 3.0);
    joint_target_pos[joint_id] = joint_range_[joint_id][1];
    std::cout << "move joint " << joint_id << " to " << joint_target_pos[joint_id]
              << ", press enter to continue:";
    std::cout.flush();
    getchar();
    jointMoveTo(joint_target_pos, 3.0);
    joint_target_pos[joint_id] = 0.;
    std::cout << "move joint " << joint_id << " to " << joint_target_pos[joint_id]
              << ", press enter to continue:";
    std::cout.flush();
    getchar();
    jointMoveTo(joint_target_pos, 3.0);
    joint_id = (joint_id + 1) % motor_count_;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}
