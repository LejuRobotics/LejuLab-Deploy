#include "leju-rl-controller/depth/depth_observation_provider.h"

#include <cstdlib>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/Float64Types.hpp>
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>

namespace leju::depth {
namespace {
std::mutex g_input_config_mutex;
DepthDdsInputConfig g_input_config;

DepthDdsInputConfig CurrentInputConfig() {
  std::lock_guard<std::mutex> lock(g_input_config_mutex);
  return g_input_config;
}

double SteadySeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

class DepthDdsSubscription {
 public:
  DepthDdsSubscription()
      : input_config_(CurrentInputConfig()), participant_(0),
        subscriber_(participant_, input_config_.topic,
                    [this](const msgs::Float64Array& message) {
                      const auto& values = message.data();
                      DepthDdsInputConfig selected = CurrentInputConfig();
                      if (selected.topic.empty()) selected = input_config_;
                      bool high_resolution =
                          values.size() == static_cast<size_t>(
                                               selected.width * selected.height);
                      if (!high_resolution &&
                          values.size() == DepthHistoryBuffer::kImageSize) {
                        selected = DepthDdsInputConfig{};
                      }
                      if (!high_resolution &&
                          values.size() != DepthHistoryBuffer::kImageSize) {
                        if (rejected_frames_++ < 5) {
                          std::fprintf(stderr,
                                       "[depth DDS] reject %zu values; expected "
                                       "%dx%d or legacy 64x36\n",
                                       values.size(), input_config_.width,
                                       input_config_.height);
                        }
                        return;
                      }
                      const uint64_t received = ++received_frames_;
                      const int decimation = std::max(1, selected.frame_decimation);
                      if (high_resolution && (received - 1) % decimation != 0) return;
                      const uint64_t sequence = ++sequence_;
                      auto frame = BuildDepthFrameFromDdsData(
                          values, selected, SteadySeconds(), sequence);
                      if (!frame) return;
                      GlobalDepthObservationProvider()->submit(*frame);
                    }) {}

 private:
  DepthDdsInputConfig input_config_;
  dds::domain::DomainParticipant participant_;
  dds_common::TopicSubscriber<msgs::Float64Array> subscriber_;
  std::atomic<uint64_t> received_frames_{0};
  std::atomic<uint64_t> sequence_{0};
  std::atomic<uint64_t> rejected_frames_{0};
};
}  // namespace

std::optional<DepthFrame> BuildDepthFrameFromDdsData(
    const std::vector<double>& values, const DepthDdsInputConfig& config,
    double timestamp_sec, uint64_t sequence) {
  if (config.width <= 0 || config.height <= 0 ||
      values.size() != static_cast<size_t>(config.width * config.height)) {
    return std::nullopt;
  }
  DepthFrame frame;
  frame.width = config.width;
  frame.height = config.height;
  frame.unit = config.unit;
  frame.timestamp_sec = timestamp_sec;
  frame.sequence = sequence;
  frame.pixels.reserve(values.size());
  for (double value : values) frame.pixels.push_back(static_cast<float>(value));
  return frame;
}

void ConfigureDepthDdsInput(const DepthDdsInputConfig& config) {
  if (config.topic.empty() || config.width <= 0 || config.height <= 0 ||
      config.frame_decimation <= 0) {
    throw std::invalid_argument("invalid DDS depth input configuration");
  }
  std::lock_guard<std::mutex> lock(g_input_config_mutex);
  g_input_config = config;
}

DepthObservationProvider::DepthObservationProvider(DepthProcessorConfig config)
    : processor_(config),
      real_camera_processor_([config] {
        auto real_config = config;
        // depth_inpainter.launch enables a 3x3 Gaussian filter for the real
        // Gemini 330 stream. Legacy 64x36 MuJoCo frames are already processed
        // and continue to use processor_ without a second blur.
        real_config.gaussian_blur = true;
        return real_config;
      }()) {}

bool DepthObservationProvider::submit(const DepthFrame& frame) {
  const bool is_legacy_processed_frame =
      frame.width == 64 && frame.height == 36 &&
      frame.unit == DepthUnit::kMeters;
  const auto processed =
      (is_legacy_processed_frame ? processor_ : real_camera_processor_)
          .process(frame);
  if (!processed.valid) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  const ActiveSource source = is_legacy_processed_frame
                                  ? ActiveSource::kSimulation
                                  : ActiveSource::kRealCamera;
  if (active_source_ != source) {
    if (source == ActiveSource::kSimulation) {
      simulation_history_.reset();
    } else {
      real_camera_history_.reset();
    }
    active_source_ = source;
  }
  auto& history = source == ActiveSource::kSimulation
                      ? simulation_history_
                      : real_camera_history_;
  return history.push(processed.pixels, processed.timestamp_sec,
                      processed.sequence, processed.valid_ratio);
}

DepthHistorySnapshot DepthObservationProvider::snapshot(double now_sec,
                                                        double timeout_sec) const {
  std::lock_guard<std::mutex> lock(mutex_);
  DepthHistorySnapshot result;
  if (active_source_ == ActiveSource::kSimulation) {
    result = simulation_history_.snapshot();
  } else if (active_source_ == ActiveSource::kRealCamera) {
    result = real_camera_history_.snapshot();
  }
  if (std::getenv("LEJU_DEPTH_FREEZE") != nullptr && result.ready && result.data.size() == DepthHistoryBuffer::kSelectedFrames * DepthHistoryBuffer::kImageSize) {
    const std::vector<float> first(result.data.begin(), result.data.begin() + DepthHistoryBuffer::kImageSize);
    for (size_t frame = 1; frame < DepthHistoryBuffer::kSelectedFrames; ++frame) {
      std::copy(first.begin(), first.end(), result.data.begin() + frame * DepthHistoryBuffer::kImageSize);
    }
  }
  if (!result.ready || now_sec - result.timestamp_sec > timeout_sec) {
    result.ready = false;
  }
  return result;
}

void DepthObservationProvider::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  simulation_history_.reset();
  real_camera_history_.reset();
  active_source_ = ActiveSource::kNone;
}

std::shared_ptr<DepthObservationProvider> GlobalDepthObservationProvider() {
  static auto provider = [] {
    DepthProcessorConfig config;
    // The standalone MuJoCo DDS frame is already the processed 36x64 meter
    // image; applying the ROS launcher's blur a second time changes the
    // policy input distribution.  Keep blur available in the processor, but
    // disabled for this source.
    config.gaussian_blur = false;
    return std::make_shared<DepthObservationProvider>(config);
  }();
  return provider;
}

bool StartDepthDdsSubscription() {
  try {
    static auto subscription = std::make_unique<DepthDdsSubscription>();
    return subscription != nullptr;
  } catch (...) {
    return false;
  }
}

}  // namespace leju::depth
