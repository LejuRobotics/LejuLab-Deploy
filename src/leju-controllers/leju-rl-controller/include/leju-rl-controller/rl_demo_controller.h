#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <openvino/openvino.hpp>
#include <Eigen/Dense>

#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/time_utils.hpp"

namespace leju {
namespace rl_demo {

using namespace leju;

struct ObsTermConfig {
  std::string name;
  double scale;
  double lb;
  double ub;
};

struct RLDemoConfig {
  double loop_dt;
  std::string policy_path;

  double policy_dt;
  std::vector<std::string> joint_names;
  // std::vector<double> joint_default_pos;
  // std::vector<double> joint_torque_limit;

  // std::vector<double> actuator_kp;
  // std::vector<double> actuator_kd;
  // std::vector<double> actuator_control_mode;

  // std::vector<double> action_scale;
  Eigen::ArrayXd q_default;
  Eigen::ArrayXd torque_limit;

  Eigen::ArrayXd kp;
  Eigen::ArrayXd kd;
  Eigen::ArrayXi control_mode;

  Eigen::ArrayXd action_scale;

  int history_length;
  bool stack_order_is_isaaclab;
  std::vector<ObsTermConfig> obs_terms;

  double command_range_lin_vel_x_lb;
  double command_range_lin_vel_x_ub;
  double command_range_lin_vel_y_lb;
  double command_range_lin_vel_y_ub;
  double command_range_ang_vel_z_lb;
  double command_range_ang_vel_z_ub;

  bool loadFromYaml(YAML::Node node);
};

class RLDemoController {
 public:
  explicit RLDemoController(
      const RobotVersion& version = RobotVersions::KUAVO5_BASE);

  ~RLDemoController();

  bool initialize(const std::string& config_file);
  void start();
  void mainLoop();

 private:
  void setupSubscriptions();
  void printRobotInfo() const;

  void imuCallback(const ImuDataConstPtr& imu_data);
  void jointStateCallback(const RobotStateConstPtr& robot_state);
  void hwStateCallback(const StringDataConstPtr& hw_state);
  void joyDataCallback(const JoyDataConstPtr& joy);

  bool load_cfg(const std::string& config_file);
  bool load_policy();

  void jointMoveTo(const std::vector<double>& joint_target_pos, double elapse);

  void computeObservation();
  void computeActions();
  void updateRobotCmd();

  Eigen::ArrayXd get_obs_term(const std::string& name);
  int get_shape_obs_term(const std::string& name);

  Eigen::ArrayXd get_obs_base_ang_vel();
  Eigen::ArrayXd get_obs_projected_gravity();
  Eigen::ArrayXd get_obs_velocity_commands();
  Eigen::ArrayXd get_obs_joint_pos();
  Eigen::ArrayXd get_obs_joint_vel();
  Eigen::ArrayXd get_obs_actions();

  int get_shape_obs_base_ang_vel() const;
  int get_shape_obs_projected_gravity() const;
  int get_shape_obs_velocity_commands() const;
  int get_shape_obs_joint_pos() const;
  int get_shape_obs_joint_vel() const;
  int get_shape_obs_actions() const;

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
  std::string config_file_path_;

  // robot info and policy cfg
  size_t motor_count_;
  std::vector<std::string> motor_names_;

  RLDemoConfig cfg_;
  int decimation_;
  // policy_joint_ids_ can be a "subset" of motor_names_, 
  // default joints will be 0 padded
  int policy_joint_count_;
  std::vector<int> policy_joint_ids_;
  int policy_obs_shape_;

  // inference resource
  ov::Core core_;
  ov::CompiledModel compiled_model_;
  ov::Output<const ov::Node> input_port_;
  ov::Output<const ov::Node> output_port_;
  ov::Tensor input_tensor_;
  ov::Tensor output_tensor_;
  ov::InferRequest infer_request_;
  

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

  Eigen::ArrayXd policy_obs_;
  std::deque<std::deque<Eigen::ArrayXd>> obs_term_stacks_;
  Eigen::ArrayXd policy_action_; // no lock because we have only 1 thread
  RobotCmd cmd_;
};

}  // namespace rl_demo
}  // namespace leju
