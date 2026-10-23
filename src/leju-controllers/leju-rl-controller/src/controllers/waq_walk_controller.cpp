#include "leju-rl-controller/controllers/waq_walk_controller.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "leju-rl-controller/controllers/controller_registry.h"
#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kWaqPolicyActionSize = 21;
constexpr int kWaqOutputSize = kWaqPolicyActionSize + 1;

array_t VectorToArray(const std::vector<float>& values, int count) {
  array_t out(count);
  for (int i = 0; i < count; ++i) {
    out[i] = static_cast<double>(values[i]);
  }
  return out;
}

double ClampBySignedLimit(double value, double positive_limit,
                          double negative_limit) {
  const double limit =
      value >= 0.0 ? std::abs(positive_limit) : std::abs(negative_limit);
  return std::clamp(value, -limit, limit);
}

double ScaleBySignedLimit(double value, double positive_limit,
                          double negative_limit, double nominal_positive,
                          double nominal_negative) {
  const double nominal =
      value >= 0.0 ? std::abs(nominal_positive) : std::abs(nominal_negative);
  const double target =
      value >= 0.0 ? std::abs(positive_limit) : std::abs(negative_limit);
  if (nominal <= 1e-6) {
    return ClampBySignedLimit(value, positive_limit, negative_limit);
  }
  const double normalized = std::clamp(value / nominal, -1.0, 1.0);
  return normalized * target;
}

}  // namespace

WaqWalkController::WaqWalkController(const RobotVersion& version,
                                     const std::string& name)
    : GenericRLController(version, name) {
  velocity_limits_ << 1.0, -1.0, 0.5, -0.5, 1.0, -1.0;
}

bool WaqWalkController::initialize() {
  if (!GenericRLController::initialize()) return false;

  if (policyJointCount() != kWaqPolicyActionSize) {
    RL_LOGE("WaqWalkController requires %d policy joints, got %d",
            kWaqPolicyActionSize, policyJointCount());
    return false;
  }

  RL_LOGI("WaqWalkController contract: float32[%d] -> float32[%d]",
          customObservationSize(), kWaqOutputSize);
  return true;
}

bool WaqWalkController::loadConfig(const std::string& config_path) {
  if (!GenericRLController::loadConfig(config_path)) return false;

  try {
    const auto root = YAML::LoadFile(config_path);
    const auto env = root["HumanoidRobotCfg"]["env"];
    if (env["policy_dt"]) policy_dt_sec_ = env["policy_dt"].as<double>();
    const auto waq = env["waq"];
    if (!waq) {
      RL_LOGW("WaqWalkController: HumanoidRobotCfg.env.waq not found, using defaults");
      return true;
    }

    if (waq["frame_stack"])
      frame_stack_count_ = std::max(1, waq["frame_stack"].as<int>());
    if (waq["num_single_obs"])
      single_obs_size_ = std::max(1, waq["num_single_obs"].as<int>());
    if (waq["cycle_time"]) cycle_time_ = waq["cycle_time"].as<double>();
    if (waq["cycle_time_short"])
      cycle_time_short_ = waq["cycle_time_short"].as<double>();
    if (waq["switch_ratio"]) switch_ratio_ = waq["switch_ratio"].as<double>();
    if (waq["leg_bias"]) leg_bias_ = waq["leg_bias"].as<double>();
    if (waq["stance_ratio"]) stance_ratio_ = waq["stance_ratio"].as<double>();
    if (waq["clip_actions"]) clip_actions_ = waq["clip_actions"].as<double>();

    if (waq["velocity_limits"]) {
      const auto limits = waq["velocity_limits"].as<std::vector<double>>();
      if (limits.size() == 8) {
        RL_LOGW("WaqWalkController: 8-value waq.velocity_limits is deprecated; "
                "entries [4,5] are unused. Use [x+, x-, y+, y-, yaw+, yaw-]");
        velocity_limits_ << limits[0], limits[1], limits[2], limits[3],
            limits[6], limits[7];
      } else if (limits.size() == 6) {
        velocity_limits_ = Eigen::Map<const array_t>(limits.data(), 6);
      } else {
        throw std::runtime_error(
            "waq.velocity_limits must contain 6 values "
            "[x+, x-, y+, y-, yaw+, yaw-] (or 8 legacy values)");
      }
    }

    RL_LOGI("WaqWalkController config: frame_stack=%d single_obs=%d cycle=%.3f "
            "cycle_short=%.3f clip=%.1f",
            frame_stack_count_, single_obs_size_, cycle_time_,
            cycle_time_short_, clip_actions_);
  } catch (const std::exception& e) {
    RL_LOGE("WaqWalkController config error: %s", e.what());
    return false;
  }

  return true;
}

void WaqWalkController::reset() {
  GenericRLController::reset();
  phase_ = 0.0;
  phase_initialized_ = false;
  gait_frequency_.store(1.0, std::memory_order_relaxed);
  fre_phase_.store(0.0, std::memory_order_relaxed);
  obs_stack_.assign(frame_stack_count_, array_t::Zero(single_obs_size_));
  observations_ = array_t::Zero(customObservationSize());
}

void WaqWalkController::updatePhase(const array_t& velocity_cmd,
                                    double gait_frequency) {
  if (phase_initialized_) {
    phase_ += policy_dt_sec_ * gait_frequency;
  } else {
    phase_initialized_ = true;
  }

  const bool zero_cmd =
      std::abs(velocity_cmd[0]) < 0.01 && std::abs(velocity_cmd[1]) < 0.01 &&
      std::abs(velocity_cmd[2]) < 0.01;
  if (zero_cmd) {
    phase_ = 0.0;
  } else {
    phase_ = std::fmod(phase_, 1.0);
    if (phase_ < 0.0) phase_ += 1.0;
  }
}

array_t WaqWalkController::buildSingleObservation() {
  array_t single = array_t::Zero(single_obs_size_);
  if (single_obs_size_ != 78) {
    RL_LOGW("WaqWalkController currently expects num_single_obs=78, got %d",
            single_obs_size_);
    return single;
  }

  refreshVelocityCommandCache();
  array_t velocity_cmd = filteredVelocityCommandSnapshot();
  if (velocity_cmd.size() < 3) velocity_cmd = array_t::Zero(3);

  velocity_cmd[0] = ScaleBySignedLimit(velocity_cmd[0], velocity_limits_[0],
                                       velocity_limits_[1], 1.0, -1.0);
  velocity_cmd[1] = ScaleBySignedLimit(velocity_cmd[1], velocity_limits_[2],
                                       velocity_limits_[3], 0.5, -0.5);
  velocity_cmd[2] = ScaleBySignedLimit(velocity_cmd[2], velocity_limits_[4],
                                       velocity_limits_[5], 1.0, -1.0);

  const double fre_phase = fre_phase_.load(std::memory_order_relaxed);
  const double gait_frequency =
      gait_frequency_.load(std::memory_order_relaxed);
  updatePhase(velocity_cmd, gait_frequency);

  array_t joint_pos = array_t::Zero(policyJointCount());
  array_t joint_vel = array_t::Zero(policyJointCount());
  for (int i = 0; i < policyJointCount(); ++i) {
    const int motor_id = policy_joint_ids_[i];
    if (motor_id < 0 || motor_id >= static_cast<int>(current_state_.q.size()) ||
        motor_id >= static_cast<int>(current_state_.v.size())) {
      continue;
    }
    joint_pos[i] = joint_direction_[i] * current_state_.q[motor_id] -
                   default_joint_pos_[i];
    joint_vel[i] = joint_direction_[i] * current_state_.v[motor_id];
  }

  array_t action = currentActionSnapshot();
  if (action.size() != policyJointCount()) {
    action = array_t::Zero(policyJointCount());
  }

  Eigen::Quaterniond quat(current_imu_.quat[0], current_imu_.quat[1],
                          current_imu_.quat[2], current_imu_.quat[3]);
  const Eigen::Vector3d gravity_world(0.0, 0.0, -1.0);
  const Eigen::Vector3d projected_gravity = quat.inverse() * gravity_world;

  array_t command_phase(5);
  command_phase << std::sin(2.0 * kPi * phase_),
      std::cos(2.0 * kPi * phase_), leg_bias_, stance_ratio_,
      gait_frequency;

  int offset = 0;
  single.segment(offset, 3) << current_imu_.gyro[0], current_imu_.gyro[1],
      current_imu_.gyro[2];
  offset += 3;
  single.segment(offset, 3) << projected_gravity[0], projected_gravity[1],
      projected_gravity[2];
  offset += 3;
  single.segment(offset, policyJointCount()) = joint_pos;
  offset += policyJointCount();
  single.segment(offset, policyJointCount()) = joint_vel;
  offset += policyJointCount();
  single.segment(offset, policyJointCount()) = action;
  offset += policyJointCount();
  single[offset++] = fre_phase;
  single.segment(offset, 3) = velocity_cmd.head(3);
  offset += 3;
  single.segment(offset, 5) = command_phase;

  return single;
}

void WaqWalkController::computeObservation() {
  if (obs_stack_.size() != static_cast<size_t>(frame_stack_count_)) {
    obs_stack_.assign(frame_stack_count_, array_t::Zero(single_obs_size_));
  }

  obs_stack_.push_back(buildSingleObservation());
  while (obs_stack_.size() > static_cast<size_t>(frame_stack_count_)) {
    obs_stack_.pop_front();
  }

  observations_.resize(customObservationSize());
  for (int i = 0; i < frame_stack_count_; ++i) {
    observations_.segment(i * single_obs_size_, single_obs_size_) =
        obs_stack_[i];
  }
}

bool WaqWalkController::inferActions(const array_t& observation,
                                     array_t& inferred_actions) {
  if (!model_ || !model_->isLoaded()) {
    return false;
  }

  std::vector<float> obs_vec(observation.size());
  for (int i = 0; i < observation.size(); ++i) {
    obs_vec[i] = static_cast<float>(observation[i]);
  }

  const std::vector<float> output = model_->forward(obs_vec);
  if (output.size() != kWaqOutputSize) {
    RL_LOGW("WaqWalkController action size mismatch: got %zu, expected %d",
            output.size(), kWaqOutputSize);
    return false;
  }

  inferred_actions = VectorToArray(output, kWaqPolicyActionSize);
  for (int i = 0; i < inferred_actions.size(); ++i) {
    inferred_actions[i] =
        std::clamp(inferred_actions[i], -clip_actions_, clip_actions_);
  }

  const double fre_phase = static_cast<double>(output[kWaqPolicyActionSize]);
  fre_phase_.store(fre_phase, std::memory_order_relaxed);
  gait_frequency_.store(std::tanh(fre_phase / 4.0) * 0.1 + 0.9,
                        std::memory_order_relaxed);
  return true;
}

void WaqWalkController::computeActions() {
  array_t inferred_actions;
  if (inferActions(observations_, inferred_actions)) {
    updateActionCache(inferred_actions);
  }
}

REGISTER_CONTROLLER("WaqWalkController", WaqWalkController, waq_walk);

void ForceLinkWaqWalkController() {}

}  // namespace leju
