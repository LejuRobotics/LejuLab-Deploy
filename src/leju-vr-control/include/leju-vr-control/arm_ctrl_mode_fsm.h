#pragma once

namespace leju {
namespace vr_control {

struct ArmCtrlModeState {
  bool left_active = false;
  bool right_active = false;
  bool left_changed = false;
  bool right_changed = false;
};

class ArmCtrlModeFSM {
 public:
  ArmCtrlModeFSM() = default;

  ArmCtrlModeState update(int armMode, bool armModeChanged);
  void reset();

 private:
  bool left_active_ = false;
  bool right_active_ = false;
};

}  // namespace vr_control
}  // namespace leju
