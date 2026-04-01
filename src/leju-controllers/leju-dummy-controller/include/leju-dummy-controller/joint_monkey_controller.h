#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <atomic>
#include <mutex>
#include <vector>
#include <iomanip>
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/time_utils.hpp"

using namespace leju;

class JointMonkeyController {
 public:
  explicit JointMonkeyController(const RobotVersion& version = RobotVersions::KUAVO5_BASE);

  ~JointMonkeyController();

  bool initialize();
  void start();
  void mainLoop();

 private:
  void setupSubscriptions();
  void printRobotInfo() const;

  bool load_cfg();
 
  void imuCallback(const ImuDataConstPtr& imu_data);
  void jointStateCallback(const RobotStateConstPtr& robot_state);
  void hwStateCallback(const StringDataConstPtr& hw_state);
  void joyDataCallback(const JoyDataConstPtr& joy);

  void jointMoveTo(const std::vector<double>& joint_target_pos, double elapse);

  RobotState getRobotState() {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    return robot_state_;
  }

  ImuData getImuData() {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    return imu_data_;
  }

  HardwareState getHardwareState() {
    std::lock_guard<std::mutex> lock(hw_state_mutex_);
    return hw_state_;
  }

  JoyData getJoyData() {
    std::lock_guard<std::mutex> lock(joy_data_mutex_);
    return joy_;
  }

  // program init param
  RobotVersion robot_version_;

  // robot info and policy cfg
  size_t motor_count_;
  std::vector<std::string> motor_names_;

  std::vector<std::vector<double>> joint_range_;

  // data buffer
  std::mutex robot_state_mutex_;
  RobotState robot_state_;
  int robot_state_msg_cnt_ = 0; // TODO use bool var "xxx_msg_ready_" instead

  std::mutex imu_data_mutex_;
  ImuData imu_data_;
  int imu_data_msg_cnt_ = 0;

  std::mutex hw_state_mutex_;
  HardwareState hw_state_;
  int hw_state_msg_cnt_ = 0;

  std::mutex joy_data_mutex_;
  JoyData joy_;
  JoyData joy_prev_;
  JoyData::Buttons buttons_press_ = {};
  int joy_data_msg_cnt_ = 0;
  
};