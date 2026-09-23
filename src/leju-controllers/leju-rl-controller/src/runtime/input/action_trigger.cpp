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

ActionTrigger MakeSwitchControllerTrigger(const std::string& controller_name,
                                          bool auto_start_motion) {
  return ActionTrigger(ActionType::SwitchController,
                       std::make_shared<NamedArgs>(controller_name, auto_start_motion));
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

ActionTrigger MakeQuitSquatTrigger(double squat_height, double duration_sec) {
  return ActionTrigger(ActionType::QuitSquat,
                       std::make_shared<QuitSquatArgs>(squat_height, duration_sec));
}

ActionTrigger MakeMotionCommandTrigger(MotionCommandArgs::Operation op,
                                       const std::string& motion_name,
                                       const std::string& music,
                                       double music_delay) {
  return ActionTrigger(ActionType::MotionCommand,
                       std::make_shared<MotionCommandArgs>(op, motion_name, music, music_delay));
}

ActionTrigger MakeSetInputMaskTrigger(const std::string& mode) {
  return ActionTrigger(ActionType::SetInputMask, CreateNamedArgs(mode));
}

ActionTrigger MakeTransportFallStandTrigger(const std::string& event_name) {
  return ActionTrigger(ActionType::TransportFallStand, CreateNamedArgs(event_name));
}

}  // namespace runtime
}  // namespace leju
