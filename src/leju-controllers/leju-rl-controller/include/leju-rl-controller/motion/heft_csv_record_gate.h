#pragma once

#include <atomic>

namespace leju {

/// Cross-thread gate: PicoTeleopAdapter (joy) → HeftRLController (policy tick).
class HeftCsvRecordGate {
 public:
  static HeftCsvRecordGate& instance() {
    static HeftCsvRecordGate g;
    return g;
  }

  void requestStart() { start_req_.store(true, std::memory_order_release); }
  void requestStop() { stop_req_.store(true, std::memory_order_release); }

  bool consumeStart() {
    return start_req_.exchange(false, std::memory_order_acq_rel);
  }
  bool consumeStop() {
    return stop_req_.exchange(false, std::memory_order_acq_rel);
  }

 private:
  HeftCsvRecordGate() = default;
  std::atomic<bool> start_req_{false};
  std::atomic<bool> stop_req_{false};
};

}  // namespace leju
