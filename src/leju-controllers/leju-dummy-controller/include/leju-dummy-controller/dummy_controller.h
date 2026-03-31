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

class DummyController {
 public:
  explicit DummyController(const RobotVersion& version = RobotVersions::KUAVO5_BASE);

  ~DummyController();

  bool initialize();

  void mainLoop();

 private:
  void setupSubscriptions();
  void printRobotInfo() const; 
 
  void imuCallback(const ImuDataConstPtr& imu_data);
  void jointStateCallback(const RobotStateConstPtr& robot_state);
  void joyDataCallback(const JoyDataConstPtr& joy);

  // RobotState getRobotState() {
  //   std::lock_guard<std::mutex> lock(robot_state_mutex_);
  //   return robot_state_;
  // }

  // ImuData getImuData() {
  //   std::lock_guard<std::mutex> lock(imu_data_mutex_);
  //   return imu_data_;
  // }

  // JoyData getJoyData() {
  //   std::lock_guard<std::mutex> lock(joy_data_mutex_);
  //   return joy_;
  // }

  RobotVersion robot_version_;
  size_t motor_count_;
  std::vector<std::string> motor_names_;

  std::mutex robot_state_mutex_;
  RobotState robot_state_;
  int robot_state_msg_cnt_;

  std::mutex imu_data_mutex_;
  ImuData imu_data_;
  int imu_data_msg_cnt_;

  std::mutex joy_data_mutex_;
  JoyData joy_;
  int joy_data_msg_cnt_;
  
};