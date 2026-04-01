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
#include "lejusdk-lowlevel/topic_logger.h"
#include "lejusdk-utils/time_utils.hpp"

namespace leju {
namespace rl_mimic {

using namespace leju;

struct ObsTermConfig {
  std::string name;
  double scale;
  double lb;
  double ub;
};

struct RLMimicConfig {
  double loop_dt;
  std::string policy_path;
  std::string motion_data_path;

  double policy_dt;
  bool residual_action;
  std::vector<std::string> joint_names;
  
  Eigen::ArrayXd joint_direction; // 为了兼容roban的不同旋转方向临时加的，处理可能并不完美
  Eigen::ArrayXd q_default;
  Eigen::ArrayXd torque_limit;

  Eigen::ArrayXd kp;
  Eigen::ArrayXd kd;
  Eigen::ArrayXi control_mode;

  Eigen::ArrayXd action_scale;

  int history_length;
  bool stack_order_is_isaaclab;
  std::vector<ObsTermConfig> obs_terms;

  bool loadFromYaml(YAML::Node node);
};

class RLMimicController {
 public:
  explicit RLMimicController(
      const RobotVersion& version = RobotVersions::KUAVO5_BASE);

  ~RLMimicController();

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
  bool load_motion_data();

  void jointMoveTo(const std::vector<double>& joint_target_pos, double elapse);

  void computeObservation();
  void computeActions();
  void updateRobotCmd();

  Eigen::ArrayXd get_obs_term(const std::string& name);
  int get_shape_obs_term(const std::string& name);

  Eigen::ArrayXd get_obs_base_ang_vel();
  Eigen::ArrayXd get_obs_projected_gravity();
  Eigen::ArrayXd get_obs_joint_pos();
  Eigen::ArrayXd get_obs_joint_vel();
  Eigen::ArrayXd get_obs_actions();
  Eigen::ArrayXd get_obs_motion_command();
  Eigen::ArrayXd get_obs_motion_target_height();
  Eigen::ArrayXd get_obs_motion_anchor_ori_b();

  int get_shape_obs_base_ang_vel() const;
  int get_shape_obs_projected_gravity() const;
  int get_shape_obs_joint_pos() const;
  int get_shape_obs_joint_vel() const;
  int get_shape_obs_actions() const;
  int get_shape_obs_motion_command() const;
  int get_shape_obs_motion_target_height() const;
  int get_shape_obs_motion_anchor_ori_b() const;

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

  RLMimicConfig cfg_;
  std::vector<std::string> data_field_names_;
  int num_data_rows_;
  Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> motion_data_;
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

  // user defined data buffer
  double dummy_world_yaw_ = 0.;
  bool start_play_motion_ = false;
  int policy_phase_cnt_ = 0;

  Eigen::ArrayXd policy_obs_;
  std::deque<std::deque<Eigen::ArrayXd>> obs_term_stacks_;
  Eigen::ArrayXd policy_action_; // no lock because we have only 1 thread
  RobotCmd cmd_;

  // logger
  std::unique_ptr<TopicLogger> logger_;
  std::string name_ = "mimic";
};

}  // namespace rl_mimic
}  // namespace leju
