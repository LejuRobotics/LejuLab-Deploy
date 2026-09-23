#pragma once

#include <string>

#include "leju-rl-controller/runtime/input/teleop/teleop_binding.hpp"

namespace leju {
namespace runtime {

/**
 * @brief 手柄速度配置
 *
 * 语义为摇杆轴值 [-1, 1] 到物理 cmd_vel 的缩放上限。
 */
struct AmpHandPostureAxisConfig {
  bool enabled = false;
  float axis_threshold = 0.4f;
  double walk_cmd_block_threshold = 0.1;
  double turn_exit_guard_threshold = 0.45;
  double policy_angular_z_scale = 1.0;
  double deep_squat_guard = -0.05;
  double squat_height_min = -0.3;  ///< posture 模式最大下蹲深度 [m]
  double squat_height_max = 0.01;  ///< posture 模式最大站起高度 [m]
};

/// 十字键前进速度档：档位来自 config_amp env.velocity_scale.walking
struct AmpHandCmdVelLineXGearConfig {
  bool enabled = false;
  /// 默认档 policy 上限，对应 velocity_scale.walking.linear_x
  double policy_linear_x = 0.75;
  /// 低档 / 高档 policy 上限，对应 linear_x_low / linear_x_up
  double policy_linear_x_low = 0.4;
  double policy_linear_x_up = 0.95;
  /// 后退 policy 上限，对应 linear_x_negative_scale
  double policy_linear_x_negative = 0.6;
};

/// AMP 入场后基于真实 Joy 帧确认摇杆回中，避免缓存清零造成误解锁。
struct VelocityEntryNeutralGuardConfig {
  bool enabled = false;
  int neutral_frames = 5;
};

/// BACK+START 组合键：先下蹲再退出程序（仅按下即触发，无长按/取消）。
struct QuitSquatConfig {
  bool enabled = false;
  double squat_height = -0.15;  ///< 下蹲高度 [m]，负值=下蹲；真机标定区间约 0.1~0.2
  double duration_sec = 1.5;    ///< 下蹲保持时长 [s]，之后退出；真机优先调此项
};

struct TeleopConfig {
  float stick_deadzone = 0.05f;     ///< 摇杆死区阈值
  double max_linear_x = 1.0;       ///< 最大前进速度 [m/s]
  double max_linear_y = 0.6;        ///< 最大侧向速度 [m/s]
  double max_angular_z = 0.3;       ///< 最大旋转速度 [rad/s]
  float trigger_threshold = 0.5f;   ///< LT/RT 扳机判定为「按下」的阈值 [0,1]
  AmpHandPostureAxisConfig amp_hand_posture_axis;
  AmpHandCmdVelLineXGearConfig amp_hand_cmd_vel_line_x_gear;
  VelocityEntryNeutralGuardConfig velocity_entry_neutral_guard;
  QuitSquatConfig quit_squat;
};

bool ParseAmpHandTeleopFromControllerConfig(const std::string& controller_config_path,
                                            TeleopConfig& config);

/**
 * @brief Teleop 组合键配置管理器
 *
 * 管理所有设备的组合键绑定配置和速度配置，支持从 YAML 文件加载。
 *
 * 配置示例：
 * @code{.yaml}
 * velocity_limits:
 *   stick_deadzone: 0.05
 *   linear_x: 0.55
 *   linear_y: 0.30
 *   angular_z: 0.30
 *
 * joy_bindings:
 *   - combo: [LB, X]
 *     type: controller_action
 *     controller_id: dance_controller
 *     action_id: dance_a
 *   - combo: [RB, A]
 *     type: arm_mode
 *     arm_mode: auto
 *
 * quest_bindings:
 *   - combo: [LEFT_TRIGGER, A]
 *     type: controller_action
 *     controller_id: dance_controller
 *     action_id: wave
 * @endcode
 */
class TeleopBindingConfig {
 public:
  TeleopBindingConfig() = default;
  ~TeleopBindingConfig() = default;

  // 默认拷贝和移动
  TeleopBindingConfig(const TeleopBindingConfig&) = default;
  TeleopBindingConfig& operator=(const TeleopBindingConfig&) = default;
  TeleopBindingConfig(TeleopBindingConfig&&) noexcept = default;
  TeleopBindingConfig& operator=(TeleopBindingConfig&&) noexcept = default;

  /**
   * @brief 从 YAML 文件加载配置
   * @param config_path 配置文件路径
   * @return 是否加载成功
   */
  bool loadFromFile(const std::string& config_path);

  /**
   * @brief 获取 Joy 设备绑定配置
   * @return Joy 设备绑定配置
   */
  const DeviceBindingConfig& getJoyConfig() const { return joy_config_; }

  /**
   * @brief 获取速度配置
   * @return 速度配置
   */
  const TeleopConfig& getTeleopConfig() const { return teleop_config_; }

  /**
   * @brief 获取 Quest 设备绑定配置
   * @return Quest 设备绑定配置
   */
  const DeviceBindingConfig& getQuestConfig() const { return quest_config_; }

  /**
   * @brief 检查是否已加载配置
   * @return 是否已加载
   */
  bool isLoaded() const { return loaded_; }

 private:
  TeleopConfig teleop_config_;       ///< 速度配置
  DeviceBindingConfig joy_config_;   ///< Joy 设备绑定配置
  DeviceBindingConfig quest_config_; ///< Quest 设备绑定配置
  bool loaded_ = false;
};

}  // namespace runtime
}  // namespace leju
