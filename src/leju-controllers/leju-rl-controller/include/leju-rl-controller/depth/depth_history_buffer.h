#pragma once

#include <cstdint>
#include <mutex>
#include <initializer_list>
#include <vector>

namespace leju::depth {

struct DepthHistorySnapshot {
  bool ready = false;
  double timestamp_sec = 0.0;
  uint64_t sequence = 0;
  double valid_ratio = 0.0;
  std::vector<float> data;
};

class DepthHistoryBuffer {
 public:
  // The source MuJoCo path publishes directly to depth_history_array. At
  // 30 Hz it retains 22 frames and samples every third frame, producing the
  // chronological offsets [0, 3, 6, 9, 12, 15, 18, 21] over about 0.7 s.
  static constexpr size_t kSourceFrames = 22;
  static constexpr size_t kSelectedFrames = 8;
  static constexpr size_t kImageSize = 36 * 64;

  DepthHistoryBuffer();
  DepthHistoryBuffer(size_t source_frames,
                     std::initializer_list<size_t> selected_indices);

  void reset();
  bool push(const std::vector<float>& image, double timestamp_sec,
            uint64_t sequence, double valid_ratio);
  DepthHistorySnapshot snapshot() const;

 private:
  struct Entry {
    std::vector<float> image;
    double timestamp_sec = 0.0;
    uint64_t sequence = 0;
    double valid_ratio = 0.0;
  };
  mutable std::mutex mutex_;
  std::vector<Entry> entries_;
  std::vector<size_t> selected_indices_;
  size_t next_ = 0;
  bool ready_ = false;
};

}  // namespace leju::depth
