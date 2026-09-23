/**
 * @file teleop_binding_config.cpp
 * @brief Teleop 组合键配置管理实现
 */

#include "leju-rl-controller/runtime/input/teleop/teleop_binding_config.h"

#include <algorithm>

#include <magic_enum/magic_enum.hpp>
#include <yaml-cpp/yaml.h>

#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace runtime {

static bool ParseTeleopConfig(const YAML::Node& node, TeleopConfig& config) {
  if (!node) {
    RL_LOG_FAILURE("TeleopBindingConfig: 缺少必需的 velocity_limits 配置段");
    return false;
  }
  if (!node["stick_deadzone"] || !node["linear_x"] || !node["linear_y"] || !node["angular_z"]) {
    RL_LOG_FAILURE("TeleopBindingConfig: velocity_limits 必须包含 stick_deadzone, linear_x, linear_y, angular_z");
    return false;
  }

  config.stick_deadzone = node["stick_deadzone"].as<float>();
  config.max_linear_x = node["linear_x"].as<double>();
  config.max_linear_y = node["linear_y"].as<double>();
  config.max_angular_z = node["angular_z"].as<double>();
  // 可选：LT/RT 扳机按下判定阈值，缺省保留 TeleopConfig 默认值
  if (node["trigger_threshold"]) {
    config.trigger_threshold = node["trigger_threshold"].as<float>();
  }
  return true;
}

// ============================================================================
// 辅助函数：解析 ActionTrigger
// ============================================================================

static ActionTrigger parseActionTrigger(const YAML::Node& node) {
  if (!node["type"]) {
    return ActionTrigger();
  }

  std::string type_str = node["type"].as<std::string>();
  ActionType action_type = ParseActionType(type_str);

  if (action_type == ActionType::None) {
    return ActionTrigger();
  }

  // 解析 args
  std::string name_arg;
  if (node["args"] && node["args"]["name"]) {
    name_arg = node["args"]["name"].as<std::string>();
  }

  // 创建对应的 ActionTrigger
  switch (action_type) {
    case ActionType::SwitchController: {
      bool auto_start = false;
      if (node["args"] && node["args"]["auto_start_motion"]) {
        auto_start = node["args"]["auto_start_motion"].as<bool>();
      }
      return MakeSwitchControllerTrigger(name_arg, auto_start);
    }

    case ActionType::SetArmMode:
      return MakeSetArmModeTrigger(name_arg);

    case ActionType::SetWaistMode:
      return MakeSetWaistModeTrigger(name_arg);

    case ActionType::Quit:
      return MakeQuitTrigger();

    case ActionType::SetInputMask:
      // name 缺省为 toggle
      return MakeSetInputMaskTrigger(name_arg.empty() ? "toggle" : name_arg);

    case ActionType::TransportFallStand:
      // 搬运/倒地起身调度事件必须指定事件名
      if (name_arg.empty()) {
        RL_LOG_FAILURE("TransportFallStand: 缺少必需的 'name' 参数（事件名）");
        return ActionTrigger();
      }
      return MakeTransportFallStandTrigger(name_arg);

    case ActionType::MotionCommand: {
      // MotionCommand 必须指定 op 参数
      if (!node["args"] || !node["args"]["op"]) {
        RL_LOG_FAILURE("MotionCommand: 缺少必需的 'op' 参数");
        return ActionTrigger();
      }

      std::string op_str = node["args"]["op"].as<std::string>();
      auto op_result = magic_enum::enum_cast<MotionCommandArgs::Operation>(op_str);
      if (!op_result.has_value()) {
        RL_LOG_FAILURE("MotionCommand: 不支持的操作 '%s'", op_str.c_str());
        return ActionTrigger();
      }

      // music: 可选, 自定义组合键动作的配套音乐 (control_logic 音乐钩子 → audio_player_node)
      std::string music_arg;
      if (node["args"]["music"]) {
        music_arg = node["args"]["music"].as<std::string>();
      }
      // music_delay: 可选, 音乐起播延迟[s], 用于对齐舞蹈起播的就位/blend 时间 (默认 0)
      double music_delay_arg = 0.0;
      if (node["args"]["music_delay"]) {
        music_delay_arg = node["args"]["music_delay"].as<double>();
      }

      return MakeMotionCommandTrigger(op_result.value(), name_arg, music_arg, music_delay_arg);
    }

    default:
      return ActionTrigger(action_type);
  }
}

// ============================================================================
// 辅助函数：解析绑定（仅支持新格式）
// ============================================================================

static TeleopBinding ParseBinding(const YAML::Node& node) {
  TeleopBinding binding;

  // 解析 buttons
  if (node["buttons"] && node["buttons"].IsSequence()) {
    for (const auto& btn : node["buttons"]) {
      binding.combo.buttons.push_back(btn.as<std::string>());
    }
    RL_LOGD("ParseBinding: Parsed combo [%s]", ComboKeyToString(binding.combo).c_str());
  }

  // 解析 action 结构
  if (node["action"] && node["action"]["type"]) {
    binding.action = parseActionTrigger(node["action"]);
    RL_LOGD("ParseBinding: Parsed action type=%d", static_cast<int>(binding.action.type));
  }

  // 解析触发边沿：edge: press | release（缺省 press）
  if (node["edge"]) {
    std::string edge_str = node["edge"].as<std::string>();
    if (edge_str == "release") {
      binding.edge = TriggerEdge::kRelease;
    } else if (edge_str == "press") {
      binding.edge = TriggerEdge::kPress;
    } else {
      RL_LOGW("ParseBinding: 未知 edge '%s'，回退为 press", edge_str.c_str());
    }
  }

  // 解析走路门控：block_when_walking: true（缺省 false）
  if (node["block_when_walking"]) {
    binding.block_when_walking = node["block_when_walking"].as<bool>();
  }

  // 解析长按时长：hold: 2.0（秒，缺省 0 = 边沿触发；>0 时忽略 edge）
  if (node["hold"]) {
    binding.hold_duration = node["hold"].as<double>();
    if (binding.hold_duration <= 0.0) {
      RL_LOGW("ParseBinding: hold=%.2f 非法，回退为边沿触发", binding.hold_duration);
      binding.hold_duration = 0.0;
    }
  }

  return binding;
}

static DeviceBindingConfig ParseDeviceConfig(const YAML::Node& node,
                                              TeleopDeviceType device_type) {
  DeviceBindingConfig config;
  config.device_type = device_type;

  if (!node.IsSequence()) {
    RL_LOGW("ParseDeviceConfig: Node is not a sequence");
    return config;
  }

  RL_LOGD("ParseDeviceConfig: Parsing %zu bindings", node.size());
  for (const auto& binding_node : node) {
    auto binding = ParseBinding(binding_node);
    if (!binding.combo.empty() && binding.action.IsValid()) {
      RL_LOGD("ParseDeviceConfig: Adding binding for combo [%s] -> action type=%d",
              ComboKeyToString(binding.combo).c_str(),
              static_cast<int>(binding.action.type));
      config.addBinding(binding);
    } else {
      RL_LOGW("ParseDeviceConfig: Skipping invalid binding (empty combo=%d, valid action=%d)",
              binding.combo.empty(), binding.action.IsValid());
    }
  }

  return config;
}

// ============================================================================
// TeleopBindingConfig 实现
// ============================================================================

static void ParseAmpHandPostureAxisConfig(const YAML::Node& posture_axis,
                                          TeleopConfig& config) {
  if (!posture_axis) {
    return;
  }
  config.amp_hand_posture_axis.enabled =
      posture_axis["enabled"].as<bool>(false);
  config.amp_hand_posture_axis.axis_threshold =
      posture_axis["axis_threshold"].as<float>(0.4f);
  config.amp_hand_posture_axis.walk_cmd_block_threshold =
      posture_axis["walk_cmd_block_threshold"].as<double>(0.1);
  config.amp_hand_posture_axis.turn_exit_guard_threshold =
      posture_axis["turn_exit_guard_threshold"].as<double>(0.45);
  config.amp_hand_posture_axis.deep_squat_guard =
      posture_axis["deep_squat_guard"].as<double>(-0.05);
  config.amp_hand_posture_axis.squat_height_min =
      posture_axis["squat_height_min"].as<double>(-0.3);
  config.amp_hand_posture_axis.squat_height_max =
      posture_axis["squat_height_max"].as<double>(0.01);
}

static void ParseQuitSquatConfig(const YAML::Node& quit_squat,
                                 TeleopConfig& config) {
  if (!quit_squat) {
    return;
  }
  config.quit_squat.enabled = quit_squat["enabled"].as<bool>(false);
  config.quit_squat.squat_height =
      quit_squat["squat_height"].as<double>(-0.15);
  config.quit_squat.duration_sec =
      quit_squat["duration_sec"].as<double>(1.5);
}

static void ParseAmpHandCmdVelLineXGearConfig(const YAML::Node& walking_scale,
                                              TeleopConfig& config) {
  if (!walking_scale || !walking_scale["linear_x"] ||
      !walking_scale["linear_x_low"] || !walking_scale["linear_x_up"]) {
    return;
  }
  auto& gear_cfg = config.amp_hand_cmd_vel_line_x_gear;
  gear_cfg.enabled = true;
  gear_cfg.policy_linear_x = walking_scale["linear_x"].as<double>();
  gear_cfg.policy_linear_x_low = walking_scale["linear_x_low"].as<double>();
  gear_cfg.policy_linear_x_up = walking_scale["linear_x_up"].as<double>();
  if (walking_scale["linear_x_negative_scale"]) {
    gear_cfg.policy_linear_x_negative =
        walking_scale["linear_x_negative_scale"].as<double>();
  }
}

static void ParseVelocityEntryNeutralGuardConfig(const YAML::Node& node,
                                                 TeleopConfig& config) {
  if (!node) {
    return;
  }
  auto& guard = config.velocity_entry_neutral_guard;
  guard.enabled = node["enabled"].as<bool>(false);
  guard.neutral_frames =
      std::max(1, node["neutral_frames"].as<int>(guard.neutral_frames));
}

bool ParseAmpHandTeleopFromControllerConfig(
    const std::string& controller_config_path, TeleopConfig& config) {
  try {
    const YAML::Node root = YAML::LoadFile(controller_config_path);
    const YAML::Node env = root["HumanoidRobotCfg"]["env"];
    if (!env) {
      RL_LOGW("ParseAmpHandTeleop: no HumanoidRobotCfg.env in %s",
              controller_config_path.c_str());
      return false;
    }

    const YAML::Node posture_axis = env["amp_hand_posture_axis"];
    const YAML::Node neutral_guard = env["velocity_entry_neutral_guard"];
    const YAML::Node quit_squat = env["quit_squat"];
    const YAML::Node walking_scale =
        env["velocity_scale"] ? env["velocity_scale"]["walking"] : YAML::Node();
    const bool has_line_x_gear =
        walking_scale && walking_scale["linear_x_low"] &&
        walking_scale["linear_x_up"];
    if (!posture_axis && !has_line_x_gear && !neutral_guard && !quit_squat) {
      return false;
    }

    ParseAmpHandPostureAxisConfig(posture_axis, config);
    ParseVelocityEntryNeutralGuardConfig(neutral_guard, config);
    ParseQuitSquatConfig(quit_squat, config);
    if (walking_scale && walking_scale["angular_z"]) {
      config.amp_hand_posture_axis.policy_angular_z_scale =
          walking_scale["angular_z"].as<double>(1.0);
    }
    ParseAmpHandCmdVelLineXGearConfig(walking_scale, config);
    RL_LOGI("ParseAmpHandTeleop: loaded from %s "
            "(posture=%s, line_x_gear=%s, entry_neutral_guard=%s/%d, quit_squat=%s)",
            controller_config_path.c_str(),
            config.amp_hand_posture_axis.enabled ? "on" : "off",
            config.amp_hand_cmd_vel_line_x_gear.enabled ? "on" : "off",
            config.velocity_entry_neutral_guard.enabled ? "on" : "off",
            config.velocity_entry_neutral_guard.neutral_frames,
            config.quit_squat.enabled ? "on" : "off");
    return true;
  } catch (const std::exception& e) {
    RL_LOGW("ParseAmpHandTeleop: failed to load %s: %s",
            controller_config_path.c_str(), e.what());
    return false;
  }
}

bool TeleopBindingConfig::loadFromFile(const std::string& config_path) {
  // 初始化设备类型
  loaded_ = false;
  joy_config_.device_type = TeleopDeviceType::kJoy;
  quest_config_.device_type = TeleopDeviceType::kQuest;
  try {
    RL_LOGI("TeleopBindingConfig: Loading config from %s", config_path.c_str());
    YAML::Node config = YAML::LoadFile(config_path);

    if (!ParseTeleopConfig(config["velocity_limits"], teleop_config_)) {
      return false;
    }

    // 解析 joy_bindings
    if (config["joy_bindings"]) {
      RL_LOGD("TeleopBindingConfig: Found joy_bindings section");
      joy_config_ = ParseDeviceConfig(config["joy_bindings"], TeleopDeviceType::kJoy);
    } else {
      RL_LOGW("TeleopBindingConfig: 未找到 joy_bindings 配置段");
    }

    // 解析 quest_bindings
    if (config["quest_bindings"]) {
      RL_LOGD("TeleopBindingConfig: Found quest_bindings section");
      quest_config_ = ParseDeviceConfig(config["quest_bindings"], TeleopDeviceType::kQuest);
    } else {
      RL_LOGW("TeleopBindingConfig: 未找到 quest_bindings 配置段");
    }

    loaded_ = true;

    RL_LOG_SUCCESS("TeleopBindingConfig: 已加载 %zu 个 joy 绑定和 %zu 个 quest",
            joy_config_.size(), quest_config_.size());

    return true;
  } catch (const YAML::Exception& e) {
    RL_LOG_FAILURE("TeleopBindingConfig: 加载配置失败 %s", config_path.c_str());
    RL_LOG_FAILURE("  错误: %s", e.what());
    RL_LOG_FAILURE("  位置: 第 %zu 行, 第 %zu 列", e.mark.line, e.mark.column);
    RL_LOG_WARNING("  提示: 请检查文件是否存在以及 YAML 语法是否正确");
    return false;
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("TeleopBindingConfig: 加载配置时发生错误: %s", e.what());
    return false;
  }
}

}  // namespace runtime
}  // namespace leju
