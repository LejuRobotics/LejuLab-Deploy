#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "leju-rl-controller/depth/depth_history_buffer.h"
#include "leju-rl-controller/depth/depth_image_processor.h"

namespace leju::depth {

struct DepthDdsInputConfig {
  std::string topic = "rt/depth_camera/frame_meters_36x64";
  int width = 64;
  int height = 36;
  DepthUnit unit = DepthUnit::kMeters;
  int frame_decimation = 1;
  std::string data_type = "float64";
  bool preprocessed = false;
};

bool ShouldSubscribeMujocoDepthFallback(const DepthDdsInputConfig& config);
std::optional<ProcessedDepth> BuildPreprocessedDepth(
    const std::vector<double>& values, double receive_steady_sec,
    uint64_t sequence);
std::optional<double> ComputeDepthLatencySec(int32_t sec, uint32_t nsec,
                                             double receive_system_sec);

std::optional<DepthFrame> BuildDepthFrameFromDdsData(
    const std::vector<double>& values, const DepthDdsInputConfig& config,
    double timestamp_sec, uint64_t sequence);
std::optional<DepthFrame> BuildDepthFrameFromDdsData(
    const std::vector<uint16_t>& values, const DepthDdsInputConfig& config,
    double timestamp_sec, uint64_t sequence);

void ConfigureDepthDdsInput(const DepthDdsInputConfig& config);

class DepthObservationProvider {
 public:
  explicit DepthObservationProvider(DepthProcessorConfig config = {});
  bool submit(const DepthFrame& frame);
  bool submitPreprocessed(const ProcessedDepth& depth);
  DepthHistorySnapshot snapshot(double now_sec, double timeout_sec) const;
  void reset();

 private:
  DepthImageProcessor processor_;
  DepthImageProcessor real_camera_processor_;
  mutable std::mutex mutex_;
  DepthHistoryBuffer simulation_history_;
  DepthHistoryBuffer real_camera_history_{
      43, {0, 6, 12, 18, 24, 30, 36, 42}};
  enum class ActiveSource { kNone, kSimulation, kRealCamera };
  ActiveSource active_source_ = ActiveSource::kNone;
};

std::shared_ptr<DepthObservationProvider> GlobalDepthObservationProvider();
bool StartDepthDdsSubscription();

}  // namespace leju::depth
