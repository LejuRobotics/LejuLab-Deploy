// 麦克风采集 → DDS /rt/micphone_data 发布节点。
// 从 NX Python/ROS micphone_receiver_node.py 迁移到 RK3588 C++/CycloneDDS。
//
// 关键适配：
//  - pyaudio → ALSA plughw（设备名打开，非卡号）
//  - ROS topic → CycloneDDS IDL AudioReceiverData{sequence<octet> data}
//  - 48k→16k 降采样交给 ALSA plughw 插件自动完成
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/AudioReceiverData.hpp>
#include <lejusdk-topic-pubsub/topic_publisher.hpp>
#include <lejusdk-topic-pubsub/topic_names.h>

#include "mic_capture.h"

namespace {
constexpr int kDefaultDomainId = 0;
std::atomic<bool> g_running{true};

void signalHandler(int) { g_running = false; }

std::string envOr(const char* name, const std::string& def) {
  const char* v = std::getenv(name);
  return v && *v ? std::string(v) : def;
}
}  // namespace

using leju::audio::MicCapture;
using leju::audio::MicCaptureParams;
using leju::dds_common::TopicPublisher;
using leju::dds_topics::kAudioData;
using leju::msgs::AudioReceiverData;

int main() {
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  MicCaptureParams params;
  params.pcm_device = envOr("MIC_PCM_DEVICE", params.pcm_device);
  params.keyword = envOr("MIC_KEYWORD", params.keyword);
  params.sample_rate = std::stoul(envOr("MIC_SAMPLE_RATE", "16000"));
  params.channels = std::stoul(envOr("MIC_CHANNELS", "1"));
  params.period_frames = std::stoul(envOr("MIC_PERIOD_FRAMES", "512"));

  MicCapture cap(params);
  if (!cap.open()) {
    std::cerr << "[ERROR] failed to open microphone, exit" << std::endl;
    return 1;
  }

  // --- 设备唤醒：Jieli USB 设备 open 后可能持续返回全零数据，
  //     通过 close/reopen 循环可触发设备恢复正常 ---
  constexpr int kWakeupFramesPerCycle = 10;       // 每轮读几帧
  constexpr int kWakeupRetryIntervalMs = 500;     // 轮间间隔 (ms)
  constexpr int kWakeupMaxCycles = 120;           // 最多轮数 (120 * 0.5s = 60s)

  std::vector<uint8_t> buf;
  bool mic_ready = false;

  for (int cycle = 0; cycle < kWakeupMaxCycles && g_running; ++cycle) {
    bool saw_nonzero = false;
    for (int f = 0; f < kWakeupFramesPerCycle && g_running; ++f) {
      int n = cap.read(buf);
      if (n <= 0) continue;
      bool all_zero = std::all_of(buf.begin(), buf.begin() + n,
                                  [](uint8_t b) { return b == 0; });
      if (!all_zero) {
        saw_nonzero = true;
        break;
      }
    }

    if (saw_nonzero) {
      mic_ready = true;
      std::cout << "[INFO] mic wake-up ok (cycle " << cycle << ")" << std::endl;
      break;
    }

    std::cout << "[INFO] mic wake-up retry " << (cycle + 1) << "/"
              << kWakeupMaxCycles << " — close/reopen..." << std::endl;
    cap.close();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(kWakeupRetryIntervalMs));
    if (!cap.open()) {
      std::cerr << "[ERROR] mic reopen failed at cycle " << cycle << std::endl;
      return 1;
    }
  }

  if (!mic_ready) {
    if (g_running) {
      std::cerr << "[ERROR] mic wake-up timeout, proceeding anyway" << std::endl;
    } else {
      std::cout << "[INFO] mic wake-up interrupted by signal" << std::endl;
      return 0;
    }
  }

  // DDS
  dds::domain::DomainParticipant participant(kDefaultDomainId);
  TopicPublisher<AudioReceiverData> pub(participant, kAudioData);
  if (!pub.is_valid()) {
    std::cerr << "[ERROR] DDS publisher not valid for " << kAudioData << std::endl;
    return 1;
  }
  std::cout << "[INFO] DDS topic: " << kAudioData << ", publisher ready" << std::endl;
  std::cout << "[INFO] listening..." << std::endl;

  long publish_fail = 0;
  long frame_count = 0;
  while (g_running) {
    int n = cap.read(buf);
    if (n < 0) {
      std::cerr << "[ERROR] read failed, exiting" << std::endl;
      break;
    }
    if (n == 0) continue;  // overrun 恢复后空帧

    AudioReceiverData msg;
    msg.data(std::vector<uint8_t>(buf.begin(), buf.begin() + n));
    if (!pub.publish(msg)) {
      publish_fail++;
      if (frame_count % 100 == 0) {
        std::cerr << "[WARN] publish failed " << publish_fail << "/" << frame_count << std::endl;
      }
    }
    frame_count++;
  }

  std::cout << "[INFO] shutting down (frames=" << frame_count
            << ", publish_fail=" << publish_fail << ")" << std::endl;
  cap.close();
  return 0;
}
