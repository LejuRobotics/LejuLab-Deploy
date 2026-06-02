/**
 * @file quest_vr_fsm.cpp
 * @brief Quest VR joystick FSM implementation
 *
 * 对齐 kuavo-ros-control：仅用 X+A / X+B，无 trigger 解锁、无 grip 锁定。
 */

#include "leju-vr-control/quest_vr_fsm.h"

namespace leju {
namespace vr_control {

QuestVrFSMAction QuestVrFSM::update(const QuestJoystickData& joy, int dt_ms) {
  (void)dt_ms;
  QuestVrFSMAction action;

  bool xa_now = isXAPressed(joy);
  bool xb_now = isXBPressed(joy);

  if (xb_now && !xb_prev_) {
    arm_mode_ = (arm_mode_ == 0) ? 2 : 0;
    action.request_set_arm_mode = true;
    action.arm_mode = arm_mode_;
    action.xb_pressed_event = true;
  } else if (xa_now && !xa_prev_) {
    arm_mode_ = (arm_mode_ == 1) ? 2 : 1;
    action.request_set_arm_mode = true;
    action.arm_mode = arm_mode_;
    action.xa_pressed_event = true;
  }

  xa_prev_ = xa_now;
  xb_prev_ = xb_now;

  return action;
}

}  // namespace vr_control
}  // namespace leju
