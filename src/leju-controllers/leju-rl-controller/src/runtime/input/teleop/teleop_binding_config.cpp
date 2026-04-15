/**
 * @file teleop_binding_config.cpp
 * @brief Teleop 组合键配置管理实现
 */

#include "leju-rl-controller/runtime/input/teleop/teleop_binding_config.h"

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
    case ActionType::SwitchController:
      return MakeSwitchControllerTrigger(name_arg);

    case ActionType::SetArmMode:
      return MakeSetArmModeTrigger(name_arg);

    case ActionType::SetWaistMode:
      return MakeSetWaistModeTrigger(name_arg);

    case ActionType::Quit:
      return MakeQuitTrigger();

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

      return MakeMotionCommandTrigger(op_result.value(), name_arg);
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
