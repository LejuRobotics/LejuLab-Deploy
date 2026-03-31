/**
 * @file action_trigger.cpp
 * @brief ActionTrigger 模块实现
 */

#include "leju-rl-controller/runtime/input/action_trigger.h"

#include <magic_enum/magic_enum.hpp>

namespace leju {
namespace runtime {

ActionType ParseActionType(const std::string& str) {
  auto result = magic_enum::enum_cast<ActionType>(str);
  return result.value_or(ActionType::None);
}

std::string ActionTypeToString(ActionType type) {
  auto str = magic_enum::enum_name(type);
  return std::string(str);
}

std::shared_ptr<ActionArgs> CreateNamedArgs(const std::string& name) {
  return std::make_shared<NamedArgs>(name);
}

ActionTrigger MakeSwitchControllerTrigger(const std::string& controller_name) {
  return ActionTrigger(ActionType::SwitchController, CreateNamedArgs(controller_name));
}

ActionTrigger MakeSetArmModeTrigger(const std::string& mode_name) {
  return ActionTrigger(ActionType::SetArmMode, CreateNamedArgs(mode_name));
}

ActionTrigger MakeSetWaistModeTrigger(const std::string& mode_name) {
  return ActionTrigger(ActionType::SetWaistMode, CreateNamedArgs(mode_name));
}

ActionTrigger MakeQuitTrigger() {
  return ActionTrigger(ActionType::Quit);
}

ActionTrigger MakeMotionCommandTrigger(MotionCommandArgs::Operation op,
                                       const std::string& motion_name) {
  return ActionTrigger(ActionType::MotionCommand,
                       std::make_shared<MotionCommandArgs>(op, motion_name));
}

}  // namespace runtime
}  // namespace leju
