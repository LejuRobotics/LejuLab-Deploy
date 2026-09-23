#include "leju-rl-controller/rl_frequency_csv_logger.hpp"

#include "leju-rl-controller/rl_log.h"
#include "lejusdk-utils/cpu_affinity.hpp"

#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace leju {

namespace {

std::string timestampForFilename() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  localtime_r(&t, &tm_buf);
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
  return oss.str();
}

double bigCoreHitPct(uint64_t on_big_core_count, uint64_t total_count) {
  if (total_count == 0) {
    return 0.0;
  }
  return 100.0 * static_cast<double>(on_big_core_count) /
         static_cast<double>(total_count);
}

}  // namespace

std::string RLFrequencyCsvLogger::defaultCsvPath() {
  if (const char* env_path = std::getenv("RL_FREQ_CSV")) {
    if (env_path[0] != '\0') {
      return env_path;
    }
  }

  const char* home = std::getenv("HOME");
  const std::filesystem::path dir =
      home ? std::filesystem::path(home) / ".ros" / "leju"
           : std::filesystem::path("/tmp");
  std::filesystem::create_directories(dir);
  return (dir / ("rl_frequency_" + timestampForFilename() + ".csv")).string();
}

void RLFrequencyCsvLogger::start(double control_target_hz, double inference_target_hz,
                                 const std::string& csv_path) {
  stop();

  control_target_hz_ = control_target_hz;
  inference_target_hz_ = inference_target_hz;
  csv_path_ = csv_path.empty() ? defaultCsvPath() : csv_path;

  const bool file_exists = std::filesystem::exists(csv_path_);
  out_.open(csv_path_, std::ios::out | std::ios::app);
  if (!out_.is_open()) {
    csv_path_.clear();
    return;
  }

  out_ << std::fixed << std::setprecision(3);
  if (!file_exists || std::filesystem::file_size(csv_path_) == 0) {
    out_ << "timestamp_sec,control_hz,control_target_hz,inference_hz,inference_target_hz,"
            "control_on_big_core_pct,inference_on_big_core_pct,control_last_cpu,"
            "inference_last_cpu\n";
  }

  session_start_ = std::chrono::steady_clock::now();
  resetCounters();
}

void RLFrequencyCsvLogger::resetCounters() {
  control_count_ = 0;
  inference_count_ = 0;
  control_on_big_core_count_ = 0;
  inference_on_big_core_count_ = 0;
  control_last_cpu_ = -1;
  inference_last_cpu_ = -1;
  warned_control_off_big_core_ = false;
  warned_inference_off_big_core_ = false;
  window_start_ = std::chrono::steady_clock::now();
}

void RLFrequencyCsvLogger::tickControl() {
  if (!out_.is_open()) {
    return;
  }

  const int cpu = cpu::getCurrentCpu();
  control_last_cpu_ = cpu;
  ++control_count_;
  if (cpu::isBigCore(cpu)) {
    ++control_on_big_core_count_;
  }
  maybeFlushRow();
}

void RLFrequencyCsvLogger::tickInference() {
  if (!out_.is_open()) {
    return;
  }

  const int cpu = cpu::getCurrentCpu();
  inference_last_cpu_ = cpu;
  ++inference_count_;
  if (cpu::isBigCore(cpu)) {
    ++inference_on_big_core_count_;
  }
}

void RLFrequencyCsvLogger::writeRow(double session_sec, double elapsed) {
  const double control_hz = static_cast<double>(control_count_) / elapsed;
  const double inference_hz = static_cast<double>(inference_count_) / elapsed;
  const double control_on_big_pct =
      bigCoreHitPct(control_on_big_core_count_, control_count_);
  const double inference_on_big_pct =
      bigCoreHitPct(inference_on_big_core_count_, inference_count_);

  if (control_count_ > 0 && control_on_big_pct < 100.0 && !warned_control_off_big_core_) {
    RL_LOGW("Control thread ran off big cores: %.1f%% on CPU %d-%d (last cpu=%d)",
            control_on_big_pct, cpu::kRk3588BigCoreFirst, cpu::kRk3588BigCoreLast,
            control_last_cpu_);
    warned_control_off_big_core_ = true;
  }
  if (inference_count_ > 0 && inference_on_big_pct < 100.0 &&
      !warned_inference_off_big_core_) {
    RL_LOGW("Inference thread ran off big cores: %.1f%% on CPU %d-%d (last cpu=%d)",
            inference_on_big_pct, cpu::kRk3588BigCoreFirst, cpu::kRk3588BigCoreLast,
            inference_last_cpu_);
    warned_inference_off_big_core_ = true;
  }

  out_ << session_sec << ','
       << control_hz << ','
       << control_target_hz_ << ','
       << inference_hz << ','
       << inference_target_hz_ << ','
       << control_on_big_pct << ','
       << inference_on_big_pct << ','
       << control_last_cpu_ << ','
       << inference_last_cpu_ << '\n';
  out_.flush();
}

void RLFrequencyCsvLogger::maybeFlushRow() {
  const auto now = std::chrono::steady_clock::now();
  const double elapsed =
      std::chrono::duration<double>(now - window_start_).count();
  if (elapsed < record_interval_sec_) {
    return;
  }

  const double session_sec =
      std::chrono::duration<double>(now - session_start_).count();
  writeRow(session_sec, elapsed);

  control_count_ = 0;
  inference_count_ = 0;
  control_on_big_core_count_ = 0;
  inference_on_big_core_count_ = 0;
  window_start_ = now;
}

void RLFrequencyCsvLogger::stop() {
  if (!out_.is_open()) {
    return;
  }

  if (control_count_ > 0 || inference_count_ > 0) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
        std::chrono::duration<double>(now - window_start_).count();
    if (elapsed > 0.0) {
      const double session_sec =
          std::chrono::duration<double>(now - session_start_).count();
      writeRow(session_sec, elapsed);
    }
  }

  out_.close();
}

}  // namespace leju
