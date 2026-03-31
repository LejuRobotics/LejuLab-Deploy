#pragma once

#include <string>

#include "leju-rl-controller/runtime/input/teleop/teleop_binding.hpp"

namespace leju {
namespace runtime {

/**
 * @brief Teleop 组合键配置管理器
 *
 * 管理所有设备的组合键绑定配置，支持从 YAML 文件加载。
 *
 * 配置示例：
 * @code{.yaml}
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
  DeviceBindingConfig joy_config_;   ///< Joy 设备绑定配置
  DeviceBindingConfig quest_config_; ///< Quest 设备绑定配置
  bool loaded_ = false;
};

}  // namespace runtime
}  // namespace leju
