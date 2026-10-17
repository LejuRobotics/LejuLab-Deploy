#pragma once

#include <algorithm>

namespace leju::depth {

class DepthStanceGate {
 public:
  bool update(bool near_zero_command, int debounce_ticks) {
    if (!near_zero_command) {
      reset();
      return false;
    }
    const int threshold = std::max(1, debounce_ticks);
    zero_command_ticks_ = std::min(zero_command_ticks_ + 1, threshold);
    return zero_command_ticks_ >= threshold;
  }

  void reset() { zero_command_ticks_ = 0; }

 private:
  int zero_command_ticks_ = 0;
};

}  // namespace leju::depth
