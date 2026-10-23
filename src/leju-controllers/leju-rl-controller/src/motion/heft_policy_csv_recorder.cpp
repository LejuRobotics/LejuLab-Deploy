#include "leju-rl-controller/motion/heft_policy_csv_recorder.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace {

std::string NowStamp() {
  using clock = std::chrono::system_clock;
  const auto t = clock::to_time_t(clock::now());
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

std::string CsvHeader() {
  std::ostringstream oss;
  oss << "body_pos_x\tbody_pos_y\tbody_pos_z\t"
         "body_quat_w\tbody_quat_x\tbody_quat_y\tbody_quat_z";
  for (int i = 0; i < 21; ++i) {
    oss << "\tjoint_pos" << std::setw(2) << std::setfill('0') << i;
  }
  for (int i = 0; i < 21; ++i) {
    oss << "\tjoint_vel_" << std::setw(2) << std::setfill('0') << i;
  }
  return oss.str();
}

}  // namespace

HeftPolicyCsvRecorder::HeftPolicyCsvRecorder(std::string output_dir,
                                             int min_frames)
    : output_dir_(std::move(output_dir)), min_frames_(min_frames) {
  std::error_code ec;
  std::filesystem::create_directories(output_dir_, ec);
}

HeftPolicyCsvRecorder::~HeftPolicyCsvRecorder() { joinWriter(); }

void HeftPolicyCsvRecorder::joinWriter() {
  if (writer_.joinable()) {
    writer_.join();
  }
}

bool HeftPolicyCsvRecorder::recording() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return recording_;
}

std::size_t HeftPolicyCsvRecorder::numFrames() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rows_.size();
}

void HeftPolicyCsvRecorder::start(const std::string& reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (recording_) {
    RL_LOGI("[CSV] already recording (%zu frames)", rows_.size());
    return;
  }
  rows_.clear();
  rows_.reserve(50 * 60 * 10);
  recording_ = true;
  if (reason.empty()) {
    RL_LOGI("[CSV] policy-ref recording started → %s", output_dir_.c_str());
  } else {
    RL_LOGI("[CSV] policy-ref recording started (%s) → %s", reason.c_str(),
            output_dir_.c_str());
  }
}

void HeftPolicyCsvRecorder::append(const HeftRefFrame& frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!recording_) {
    return;
  }
  if (frame.jpos.size() != 21) {
    return;
  }
  if (!frame.pos.allFinite() || !frame.quat.coeffs().allFinite() ||
      !frame.jpos.allFinite()) {
    return;
  }
  HeftRefFrame copy = frame;
  if (copy.jvel.size() != 21) {
    copy.jvel = array_t::Zero(21);
  } else if (!copy.jvel.allFinite()) {
    copy.jvel = array_t::Zero(21);
  }
  copy.quat.normalize();
  rows_.push_back(std::move(copy));
}

std::string HeftPolicyCsvRecorder::stopAndSave(const std::string& reason,
                                               bool async) {
  std::vector<HeftRefFrame> rows;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!recording_ && rows_.empty()) {
      return {};
    }
    recording_ = false;
    rows.swap(rows_);
  }

  const std::size_t n = rows.size();
  if (static_cast<int>(n) < min_frames_) {
    if (reason.empty()) {
      RL_LOGI("[CSV] discard take (%zu frames < min %d)", n, min_frames_);
    } else {
      RL_LOGI("[CSV] discard take (%zu frames < min %d) [%s]", n, min_frames_,
              reason.c_str());
    }
    return {};
  }

  ++take_idx_;
  std::ostringstream name;
  name << "pico_policy_" << NowStamp() << '_' << std::setw(3)
       << std::setfill('0') << take_idx_ << "_deploy.csv";
  const std::filesystem::path path =
      std::filesystem::path(output_dir_) / name.str();

  // Disk IO off the control thread: join any previous writer (already done
  // seconds ago in practice), then hand the buffer to a fresh one.
  joinWriter();
  auto write_job = [path_str = path.string(), rows = std::move(rows), reason,
                    n]() {
    if (!writeCsv(path_str, rows)) {
      RL_LOGW("[CSV] failed to write %s", path_str.c_str());
      return;
    }
    const double play_s = static_cast<double>(n) / 50.0;
    if (reason.empty()) {
      RL_LOGI(
          "\033[92m[CSV] saved %zu policy frames (play ≈%.1fs @50Hz) → "
          "%s\033[0m",
          n, play_s, path_str.c_str());
    } else {
      RL_LOGI(
          "\033[92m[CSV] saved %zu policy frames (play ≈%.1fs @50Hz) → "
          "%s\033[0m [%s]",
          n, play_s, path_str.c_str(), reason.c_str());
    }
  };
  if (async) {
    writer_ = std::thread(std::move(write_job));
  } else {
    write_job();
  }
  return path.string();
}

void HeftPolicyCsvRecorder::clearBuffers() {
  rows_.clear();
  recording_ = false;
}

bool HeftPolicyCsvRecorder::writeCsv(const std::string& path,
                                     const std::vector<HeftRefFrame>& rows) {
  std::error_code ec;
  std::filesystem::create_directories(
      std::filesystem::path(path).parent_path(), ec);
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << CsvHeader() << '\n';
  out << std::fixed << std::setprecision(8);
  for (const auto& f : rows) {
    out << f.pos.x() << '\t' << f.pos.y() << '\t' << f.pos.z() << '\t'
        << f.quat.w() << '\t' << f.quat.x() << '\t' << f.quat.y() << '\t'
        << f.quat.z();
    for (int i = 0; i < 21; ++i) {
      out << '\t' << f.jpos[i];
    }
    for (int i = 0; i < 21; ++i) {
      out << '\t' << f.jvel[i];
    }
    out << '\n';
  }
  return static_cast<bool>(out);
}

}  // namespace leju
