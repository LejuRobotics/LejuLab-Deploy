#pragma once

#include <atomic>

namespace leju {

/// Cross-thread gate: PicoTeleopAdapter (RT+Y/X) → HeftRLController (policy tick).
class HeftStreamFreezeGate {
 public:
  static HeftStreamFreezeGate& instance() {
    static HeftStreamFreezeGate g;
    return g;
  }

  void requestFreeze() { freeze_req_.store(true, std::memory_order_release); }
  void requestResume() { resume_req_.store(true, std::memory_order_release); }

  bool consumeFreeze() {
    return freeze_req_.exchange(false, std::memory_order_acq_rel);
  }
  bool consumeResume() {
    return resume_req_.exchange(false, std::memory_order_acq_rel);
  }

 private:
  HeftStreamFreezeGate() = default;
  std::atomic<bool> freeze_req_{false};
  std::atomic<bool> resume_req_{false};
};

}  // namespace leju
