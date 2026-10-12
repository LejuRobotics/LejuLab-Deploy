#include "leju-rl-controller/depth/depth_observation_provider.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/Float64Types.hpp>
#include <lejusdk-dds-idl/UInt16Types.hpp>
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

double SystemSeconds() {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

constexpr const char* kMujocoDepthTopic =
    "rt/depth_camera/frame_meters_36x64";

class DepthDdsSubscription {
 public:
  DepthDdsSubscription()
      : input_config_(CurrentInputConfig()), participant_(0) {
    if (input_config_.data_type == "uint16") {
      uint16_subscriber_ = std::make_unique<
          dds_common::TopicSubscriber<msgs::UInt16Array>>(
          participant_, input_config_.topic,
          [this](const msgs::UInt16Array& message) { HandleRaw(message, false); });
    } else {
      float64_subscriber_ = std::make_unique<
          dds_common::TopicSubscriber<msgs::Float64Array>>(
          participant_, input_config_.topic,
          [this](const msgs::Float64Array& message) {
            HandleFloat64(message, false);
          });
    }
    // MuJoCo always publishes the legacy Float64 topic. Keep it subscribed
    // when hardware uses a different topic, regardless of hardware encoding.
    if (ShouldSubscribeMujocoDepthFallback(input_config_)) {
      float64_legacy_subscriber_ = std::make_unique<
          dds_common::TopicSubscriber<msgs::Float64Array>>(
          participant_, kMujocoDepthTopic,
          [this](const msgs::Float64Array& message) {
            HandleFloat64(message, true);
          });
    }
  }

 private:
  void HandleFloat64(const msgs::Float64Array& message, bool force_legacy) {
    if (!force_legacy && input_config_.preprocessed) {
      const uint64_t received = ++received_frames_;
      const int decimation = std::max(1, input_config_.frame_decimation);
      if ((received - 1) % decimation != 0) return;

      const double receive_steady_sec = SteadySeconds();
      const double receive_system_sec = SystemSeconds();
      const uint64_t sequence = ++sequence_;
      auto processed = BuildPreprocessedDepth(
          message.data(), receive_steady_sec, sequence);
      ReportLatency(message.header_sec(), message.header_nanosec(),
                    receive_system_sec, receive_steady_sec);
      if (!processed) {
        if (rejected_frames_++ < 5) {
          std::fprintf(stderr,
                       "[depth DDS] reject invalid preprocessed Float64 frame\n");
        }
        return;
      }
      GlobalDepthObservationProvider()->submitPreprocessed(*processed);
      return;
    }
    HandleRaw(message, force_legacy);
  }

  template <typename Message>
  void HandleRaw(const Message& message, bool force_legacy) {
    const auto& values = message.data();
    DepthDdsInputConfig selected =
        force_legacy ? DepthDdsInputConfig{} : CurrentInputConfig();
    if (selected.topic.empty()) selected = input_config_;
    bool high_resolution =
        values.size() ==
        static_cast<size_t>(selected.width * selected.height);
    if (!high_resolution && values.size() == DepthHistoryBuffer::kImageSize) {
      selected = DepthDdsInputConfig{};
    }
    if (!high_resolution && values.size() != DepthHistoryBuffer::kImageSize) {
      if (rejected_frames_++ < 5) {
        std::fprintf(stderr,
                     "[depth DDS] reject %zu values; expected %dx%d or legacy "
                     "64x36\n",
                     values.size(), input_config_.width, input_config_.height);
      }
      return;
    }
    const uint64_t received = ++received_frames_;
    const int decimation = std::max(1, selected.frame_decimation);
    if (high_resolution && (received - 1) % decimation != 0) return;
    const uint64_t sequence = ++sequence_;
    auto frame = BuildDepthFrameFromDdsData(values, selected, SteadySeconds(),
                                             sequence);
    if (!frame) return;
    GlobalDepthObservationProvider()->submit(*frame);
  }

  void ReportLatency(int32_t sec, uint32_t nsec, double receive_system_sec,
                     double receive_steady_sec) {
    const auto latency =
        ComputeDepthLatencySec(sec, nsec, receive_system_sec);
    if (!latency) {
      const uint64_t count = ++timestamp_anomalies_;
      std::fprintf(stderr,
                   "[depth DDS] warning: invalid capture timestamp "
                   "sec=%d nsec=%u (anomaly=%llu)\n",
                   sec, nsec, static_cast<unsigned long long>(count));
      return;
    }
    if (*latency < -0.05 || *latency > 5.0) {
      const uint64_t count = ++timestamp_anomalies_;
      std::fprintf(stderr,
                   "[depth DDS] warning: capture-to-receive latency=%.3f s "
                   "(anomaly=%llu)\n",
                   *latency, static_cast<unsigned long long>(count));
      return;
    }
    const double last_log =
        last_latency_log_steady_sec_.load(std::memory_order_relaxed);
    if (receive_steady_sec - last_log >= 1.0) {
      std::fprintf(stderr,
                   "[depth DDS] capture-to-receive latency=%.3f s\n",
                   *latency);
      last_latency_log_steady_sec_.store(receive_steady_sec,
                                         std::memory_order_relaxed);
    }
  }

  DepthDdsInputConfig input_config_;
  dds::domain::DomainParticipant participant_;
  std::unique_ptr<dds_common::TopicSubscriber<msgs::Float64Array>>
      float64_subscriber_;
  std::unique_ptr<dds_common::TopicSubscriber<msgs::Float64Array>>
      float64_legacy_subscriber_;
  std::unique_ptr<dds_common::TopicSubscriber<msgs::UInt16Array>>
      uint16_subscriber_;
  std::atomic<uint64_t> received_frames_{0};
  std::atomic<uint64_t> sequence_{0};
  std::atomic<uint64_t> rejected_frames_{0};
  std::atomic<uint64_t> timestamp_anomalies_{0};
  std::atomic<double> last_latency_log_steady_sec_{
      -std::numeric_limits<double>::infinity()};
};
}  // namespace

bool ShouldSubscribeMujocoDepthFallback(const DepthDdsInputConfig& config) {
  // The simulator publishes the legacy Float64 64x36 topic even when the
  // hardware configuration consumes a preprocessed high-level topic. Keep
  // this fallback subscriber enabled so the simulator can feed the policy.
  return config.topic != kMujocoDepthTopic;
}

std::optional<ProcessedDepth> BuildPreprocessedDepth(
    const std::vector<double>& values, double receive_steady_sec,
    uint64_t sequence) {
  if (values.size() != DepthHistoryBuffer::kImageSize) return std::nullopt;

  ProcessedDepth depth;
  depth.width = 64;
  depth.height = 36;
  depth.timestamp_sec = receive_steady_sec;
  depth.sequence = sequence;
  depth.pixels.reserve(values.size());
  size_t positive_count = 0;
  for (const double value : values) {
    if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
      return std::nullopt;
    }
    if (value > 0.0) ++positive_count;
    depth.pixels.push_back(static_cast<float>(value));
  }
  depth.valid_ratio =
      static_cast<double>(positive_count) / static_cast<double>(values.size());
  depth.valid = true;
  return depth;
}

std::optional<double> ComputeDepthLatencySec(int32_t sec, uint32_t nsec,
                                             double receive_system_sec) {
  if ((sec == 0 && nsec == 0) || nsec >= 1000000000u ||
      !std::isfinite(receive_system_sec)) {
    return std::nullopt;
  }
  const double capture_system_sec =
      static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
  return receive_system_sec - capture_system_sec;
}

template <typename T>
std::optional<DepthFrame> BuildDepthFrameFromTypedDdsData(
    const std::vector<T>& values, const DepthDdsInputConfig& config,
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
  for (const T value : values) frame.pixels.push_back(static_cast<float>(value));
  return frame;
}

std::optional<DepthFrame> BuildDepthFrameFromDdsData(
    const std::vector<double>& values, const DepthDdsInputConfig& config,
    double timestamp_sec, uint64_t sequence) {
  return BuildDepthFrameFromTypedDdsData(values, config, timestamp_sec, sequence);
}

std::optional<DepthFrame> BuildDepthFrameFromDdsData(
    const std::vector<uint16_t>& values, const DepthDdsInputConfig& config,
    double timestamp_sec, uint64_t sequence) {
  return BuildDepthFrameFromTypedDdsData(values, config, timestamp_sec, sequence);
}

void ConfigureDepthDdsInput(const DepthDdsInputConfig& config) {
  if (config.topic.empty() || config.width <= 0 || config.height <= 0 ||
      config.frame_decimation <= 0 ||
      (config.data_type != "float64" && config.data_type != "uint16")) {
    throw std::invalid_argument("invalid DDS depth input configuration");
  }
  if (config.preprocessed &&
      (config.data_type != "float64" || config.width != 64 ||
       config.height != 36 || config.unit != DepthUnit::kMeters)) {
    throw std::invalid_argument(
        "preprocessed DDS depth input must be float64 64x36 meters");
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

bool DepthObservationProvider::submitPreprocessed(const ProcessedDepth& depth) {
  if (!depth.valid || depth.width != 64 || depth.height != 36 ||
      depth.pixels.size() != DepthHistoryBuffer::kImageSize ||
      !std::isfinite(depth.timestamp_sec) ||
      !std::isfinite(depth.valid_ratio) || depth.valid_ratio < 0.0 ||
      depth.valid_ratio > 1.0) {
    return false;
  }
  for (const float value : depth.pixels) {
    if (!std::isfinite(value) || value < 0.0f || value > 1.0f) return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (active_source_ != ActiveSource::kRealCamera) {
    real_camera_history_.reset();
    active_source_ = ActiveSource::kRealCamera;
  }
  return real_camera_history_.push(depth.pixels, depth.timestamp_sec,
                                   depth.sequence, depth.valid_ratio);
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
