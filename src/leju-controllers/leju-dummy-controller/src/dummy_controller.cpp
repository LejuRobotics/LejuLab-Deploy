#include "leju-dummy-controller/dummy_controller.h"

DummyController::DummyController(const RobotVersion& version)
    : robot_version_(version), motor_count_(0) {}

DummyController::~DummyController() = default;

bool DummyController::initialize() {

  if (!GlobalRobot::init_env(robot_version_)) {
    std::cerr << "Failed to call init_env" << std::endl;
    return false;
  }

  auto& robot = GlobalRobot::getInstance();
  motor_count_ = robot.getMotorNumber();
  motor_names_ = robot.getMotorNames();

  printRobotInfo();
  setupSubscriptions();

  std::cout << "✓ Kuavo robot initialized successfully" << std::endl;
  return true;
}

void DummyController::printRobotInfo() const {
  std::cout << "  Robot Version: "
            << robot_version_.version_name()
            << std::endl;
  std::cout << "  Motor Count: " << motor_count_ << std::endl;
  std::cout << "  Motors: ";
  for (size_t i = 0; i < motor_names_.size(); ++i) {
    if (i > 0) std::cout << ", ";
    std::cout << motor_names_[i];
  }
  std::cout << std::endl;
}

void DummyController::setupSubscriptions() {
  auto& robot = GlobalRobot::getInstance();
  robot.subscribeImuData(
      [this](const ImuDataConstPtr& imu_data) { this->imuCallback(imu_data); });
  robot.subscribeJoyData(
      [this](const JoyDataConstPtr& joy_data) { this->joyDataCallback(joy_data); });

  robot.subscribeRobotState([this](const RobotStateConstPtr& robot_state) {
    this->jointStateCallback(robot_state);
  });
}

void DummyController::imuCallback(const ImuDataConstPtr& imu_data) {
  std::lock_guard<std::mutex> imu_lock(imu_data_mutex_);
  imu_data_ = *imu_data;
  imu_data_msg_cnt_++;
}

void DummyController::jointStateCallback(
    const RobotStateConstPtr& robot_state) {
  std::lock_guard<std::mutex> state_lock(robot_state_mutex_);
  robot_state_ = *robot_state;
  robot_state_msg_cnt_++;
}

void DummyController::joyDataCallback(const JoyDataConstPtr& joy) {
  std::lock_guard<std::mutex> joy_lock(joy_data_mutex_);
  joy_ = *joy;
  joy_data_msg_cnt_++;
}

void DummyController::mainLoop() {
  std::cout << "\n--- Starting Control Loop ---" << std::endl;
  std::cout << "Press Ctrl+C to stop" << std::endl;

  auto& robot = GlobalRobot::getInstance();
  auto start_time = std::chrono::steady_clock::now();

  while (true) {
    // get data
    {
      std::lock_guard<std::mutex> lock(robot_state_mutex_);
      std::cout << "Robot State: \n" << robot_state_ << std::endl;
      std::cout << "Robot State publish rate(Hz): " << robot_state_msg_cnt_
                << "\n" << std::endl;
      robot_state_msg_cnt_ = 0;
    }
    {
      std::lock_guard<std::mutex> lock(imu_data_mutex_);
      std::cout << "Imu Data: \n" << imu_data_ << std::endl;
      std::cout << "Imu Data publish rate(Hz): " << imu_data_msg_cnt_
                << "\n" << std::endl;
      imu_data_msg_cnt_ = 0;
    }
    {
      std::lock_guard<std::mutex> lock(joy_data_mutex_);
      std::cout << "Joy Data: \n" << joy_ << std::endl;
      std::cout << "Joy Data publish rate(Hz): " << joy_data_msg_cnt_
                << "\n" << std::endl;
      joy_data_msg_cnt_ = 0;
    }

    // publish command
    RobotCmd cmd(motor_count_);
    bool success = robot.publishRobotCmd(cmd);
    if (!success) {
      std::cout << "Failed to publish cmd." << std::endl;
    } else {
      std::cout << "Sent zero joint command.\n" << std::endl;
    }

    // TODO use loop rate sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}
