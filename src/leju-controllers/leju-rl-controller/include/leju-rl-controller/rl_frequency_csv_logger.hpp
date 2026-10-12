#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>
#include <string>

namespace leju {

/// RL 控制/推理频率 + 线程大核落点 CSV 记录器（默认每秒一行，不打印终端）
class RLFrequencyCsvLogger {
 public:
  void start(double control_target_hz, double inference_target_hz,
             const std::string& csv_path = "");
  void resetCounters();
  void tickControl();
  void tickInference();
  void stop();

  const std::string& csvPath() const { return csv_path_; }
  bool isActive() const { return out_.is_open(); }

 private:
  void maybeFlushRow();
  void writeRow(double session_sec, double elapsed);
  static std::string defaultCsvPath();

  std::ofstream out_;
  std::string csv_path_;
  double control_target_hz_ = 0.0;
  double inference_target_hz_ = 0.0;
  uint64_t control_count_ = 0;
  uint64_t inference_count_ = 0;
  uint64_t control_on_big_core_count_ = 0;
  uint64_t inference_on_big_core_count_ = 0;
  int control_last_cpu_ = -1;
  int inference_last_cpu_ = -1;
  bool warned_control_off_big_core_ = false;
  bool warned_inference_off_big_core_ = false;
  double record_interval_sec_ = 1.0;
  std::chrono::steady_clock::time_point session_start_;
  std::chrono::steady_clock::time_point window_start_;
};

}  // namespace leju
