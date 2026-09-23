#include "leju-rl-controller/depth/depth_history_buffer.h"
#include <cstdlib>
#include <stdexcept>

namespace leju::depth {

DepthHistoryBuffer::DepthHistoryBuffer()
    : DepthHistoryBuffer(22, {0, 3, 6, 9, 12, 15, 18, 21}) {}

DepthHistoryBuffer::DepthHistoryBuffer(
    size_t source_frames, std::initializer_list<size_t> selected_indices)
    : entries_(source_frames), selected_indices_(selected_indices) {
  if (source_frames == 0 || selected_indices_.size() != kSelectedFrames) {
    throw std::invalid_argument("invalid depth history configuration");
  }
  for (size_t index : selected_indices_) {
    if (index >= source_frames) {
      throw std::invalid_argument("depth history index is out of range");
    }
  }
}

void DepthHistoryBuffer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.assign(entries_.size(), Entry{});
  next_ = 0;
  ready_ = false;
}

bool DepthHistoryBuffer::push(const std::vector<float>& image,
                              double timestamp_sec, uint64_t sequence,
                              double valid_ratio) {
  if (image.size() != kImageSize) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ready_) {
    for (auto& entry : entries_) entry.image = image;
    ready_ = true;
  } else {
    entries_[next_].image = image;
  }
  entries_[next_].timestamp_sec = timestamp_sec;
  entries_[next_].sequence = sequence;
  entries_[next_].valid_ratio = valid_ratio;
  next_ = (next_ + 1) % entries_.size();
  return true;
}

DepthHistorySnapshot DepthHistoryBuffer::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  DepthHistorySnapshot result;
  result.ready = ready_;
  if (!ready_) return result;
  result.data.reserve(kSelectedFrames * kImageSize);
  const size_t source_frames = entries_.size();
  const size_t newest = (next_ + source_frames - 1) % source_frames;
  const size_t oldest = (ready_ && next_ == 0) ? 0 : (next_ == 0 ? 0 : next_);
  (void)oldest;
  const bool newest_first = std::getenv("LEJU_DEPTH_NEWEST_FIRST") != nullptr;
  for (size_t id : selected_indices_) {
    const size_t age = newest_first ? (source_frames - 1 - id) : id;
    const size_t index = (next_ + age) % source_frames;
    const auto& entry = entries_[index];
    result.data.insert(result.data.end(), entry.image.begin(), entry.image.end());
  }
  const auto& latest = entries_[newest];
  result.timestamp_sec = latest.timestamp_sec;
  result.sequence = latest.sequence;
  result.valid_ratio = latest.valid_ratio;
  return result;
}

}  // namespace leju::depth
