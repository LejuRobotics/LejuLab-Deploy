// 诊断工具：向 /rt/audio_play 推合成正弦 PCM 流，可选向 /rt/audio_stop 发停止。
// 用于验证 audio_player_node 的 DDS 流输入 + stop 路径（无需音频文件）。
//
// 用法: audio_stream_probe [seconds] [--stop-after MS]
//   seconds      : 推流时长，默认 3
//   --stop-after : 推流 MS 毫秒后发一次 stop（验证打断）
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/AudioReceiverData.hpp>
#include <lejusdk-dds-idl/StringData.hpp>
#include <lejusdk-topic-pubsub/topic_names.h>
#include <lejusdk-topic-pubsub/topic_publisher.hpp>

using leju::dds_common::TopicPublisher;
using leju::dds_topics::kAudioPlay;
using leju::dds_topics::kAudioStop;
using leju::msgs::AudioReceiverData;
using leju::msgs::StringData;

int main(int argc, char** argv) {
  double seconds = argc >= 2 && argv[1][0] != '-' ? std::atof(argv[1]) : 3.0;
  int stop_after_ms = -1;
  std::string play_file;  // 非空则改为发按名播放信号 (/rt/audio_play_file)
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--stop-after" && i + 1 < argc)
      stop_after_ms = std::atoi(argv[++i]);
    else if (std::string(argv[i]) == "--play-file" && i + 1 < argc)
      play_file = argv[++i];
  }

  dds::domain::DomainParticipant participant(0);
  TopicPublisher<AudioReceiverData> pub(participant, kAudioPlay);
  TopicPublisher<StringData> stop_pub(participant, kAudioStop);
  TopicPublisher<StringData> play_file_pub(participant, leju::dds_topics::kAudioPlayFile);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 等匹配

  // 按名播放模式：发一次文件名到 /rt/audio_play_file 即退出（验证舞蹈音乐路径）
  if (!play_file.empty()) {
    StringData s;
    s.data(play_file);
    play_file_pub.publish(s);
    std::cout << "[probe] sent play-file '" << play_file << "' to "
              << leju::dds_topics::kAudioPlayFile << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return 0;
  }

  const int rate = 16000;
  const int chunk_samples = 1600;  // 0.1s/块
  const double freq = 520.0;
  long total_samples = static_cast<long>(seconds * rate);
  long sent = 0;
  double phase = 0.0;
  auto start = std::chrono::steady_clock::now();
  bool stopped = false;

  std::cout << "[probe] streaming " << seconds << "s tone to " << kAudioPlay << std::endl;
  while (sent < total_samples) {
    int n = std::min<long>(chunk_samples, total_samples - sent);
    std::vector<uint8_t> pcm(n * 2);
    for (int i = 0; i < n; ++i) {
      double v = std::sin(phase) * 12000.0;
      phase += 2.0 * M_PI * freq / rate;
      int16_t s = static_cast<int16_t>(v);
      std::memcpy(&pcm[i * 2], &s, 2);
    }
    AudioReceiverData msg;
    msg.data(pcm);
    pub.publish(msg);
    sent += n;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (stop_after_ms > 0 && !stopped) {
      auto el = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start).count();
      if (el >= stop_after_ms) {
        StringData s;
        s.data("");
        stop_pub.publish(s);
        std::cout << "[probe] sent STOP at " << el << "ms" << std::endl;
        stopped = true;
      }
    }
  }
  std::cout << "[probe] done (sent " << sent << " samples)" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  return 0;
}
