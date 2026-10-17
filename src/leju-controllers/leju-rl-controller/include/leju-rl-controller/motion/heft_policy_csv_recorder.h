#pragma once

#include <cstddef>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "leju-rl-controller/motion/heft_reference_source.h"

namespace leju {

class HeftPolicyCsvRecorder {
 public:
  explicit HeftPolicyCsvRecorder(std::string output_dir, int min_frames = 25);
  ~HeftPolicyCsvRecorder();

  void start(const std::string& reason = "");
  /// Stop recording. async=true (default): CSV is written on a background
  /// thread (safe to call from the control thread). async=false: blocks until
  /// written (use at shutdown). Returns target path or empty.
  std::string stopAndSave(const std::string& reason = "", bool async = true);
  void append(const HeftRefFrame& frame);

  bool recording() const;
  std::size_t numFrames() const;

 private:
  void clearBuffers();
  void joinWriter();
  static bool writeCsv(const std::string& path,
                       const std::vector<HeftRefFrame>& rows);

  std::string output_dir_;
  int min_frames_ = 25;
  int take_idx_ = 0;

  mutable std::mutex mutex_;
  bool recording_ = false;
  std::vector<HeftRefFrame> rows_;
  std::thread writer_;  ///< background CSV writer (joined before reuse)
};

}  // namespace leju
