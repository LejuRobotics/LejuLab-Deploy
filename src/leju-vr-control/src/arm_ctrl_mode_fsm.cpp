#include "leju-vr-control/arm_ctrl_mode_fsm.h"

namespace leju {
namespace vr_control {

void ArmCtrlModeFSM::reset() {
  left_active_ = false;
  right_active_ = false;
}

ArmCtrlModeState ArmCtrlModeFSM::update(int armMode, bool armModeChanged) {
  ArmCtrlModeState state;
  const bool nextActive = (armMode == 2);

  if (armModeChanged) {
    if (left_active_ != nextActive) {
      state.left_changed = true;
    }
    if (right_active_ != nextActive) {
      state.right_changed = true;
    }

    if (state.left_changed || state.right_changed) {
      left_active_ = nextActive;
      right_active_ = nextActive;
    }
  }

  state.left_active = left_active_;
  state.right_active = right_active_;
  return state;
}

}  // namespace vr_control
}  // namespace leju
