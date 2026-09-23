// 音频播放节点：单节点双输入。
//  输入1：RPC AudioPlayerService.play_file → ffmpeg 解码 → 入队
//  输入2：DDS /rt/audio_play (AudioReceiverData PCM) → 入队
//  停止 ：DDS /rt/audio_stop (StringData) → 清空队列
//  输出 ：播放线程从队列取块 → SpeakerOutput(ALSA plughw) → 扬声器
//
// 与采集侧 micphone_to_dds_node 对称。全链路约定 16k/mono/S16LE。
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <dds/dds.hpp>
#include <dds_rpc/replier.hpp>
#include <lejusdk-dds-idl/AudioReceiverData.hpp>
#include <lejusdk-dds-idl/StringData.hpp>
#include <lejusdk-topic-pubsub/topic_names.h>
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>

#include "audio_decoder.h"
#include "rpc_idl/AudioPlayerService.hpp"
#include "speaker_output.h"

namespace {
constexpr int kDefaultDomainId = 0;
constexpr char kAudioPlayerService[] = "AudioPlayerService";
constexpr size_t kEnqueueChunkBytes = 8192;  // 入队分块大小

std::atomic<bool> g_running{true};
void signalHandler(int) { g_running = false; }

std::string envOr(const char* name, const std::string& def) {
  const char* v = std::getenv(name);
  return v && *v ? std::string(v) : def;
}

// 把 "~/..." 展开成 $HOME/...
std::string expandHome(const std::string& p) {
  if (!p.empty() && p[0] == '~') {
    const char* home = std::getenv("HOME");
    if (home) return std::string(home) + p.substr(1);
  }
  return p;
}
}  // namespace

using leju::audio::DecodeParams;
using leju::audio::SpeakerOutput;
using leju::audio::SpeakerOutputParams;
using leju::dds_common::TopicSubscriber;
using leju::dds_topics::kAudioPlay;
using leju::dds_topics::kAudioPlayFile;
using leju::dds_topics::kAudioStop;
using leju::msgs::AudioReceiverData;
using leju::msgs::StringData;
using leju::srv::PlayAudioReply;
using leju::srv::PlayAudioRequest;

// 有界 PCM 队列：满则丢最旧块；支持唤醒退出与清空。
class PcmQueue {
 public:
  explicit PcmQueue(size_t max_blocks) : max_blocks_(max_blocks) {}

  void push(std::vector<uint8_t> block) {
    if (block.empty()) return;
    std::lock_guard<std::mutex> lk(mu_);
    if (queue_.size() >= max_blocks_) {
      queue_.pop_front();
      if (++drop_count_ % 50 == 1) {
        std::cerr << "[WARN] pcm queue full, dropping oldest (count=" << drop_count_
                  << ")" << std::endl;
      }
    }
    queue_.push_back(std::move(block));
    cv_.notify_one();
  }

  // 阻塞取一块；停止/清空唤醒时返回 false。
  bool pop(std::vector<uint8_t>& out) {
    std::unique_lock<std::mutex> lk(mu_);
    cv_.wait(lk, [this] { return !queue_.empty() || !running_; });
    if (!running_) return false;
    if (queue_.empty()) return false;
    out = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  void clear() {
    std::lock_guard<std::mutex> lk(mu_);
    queue_.clear();
  }

  void stop() {
    std::lock_guard<std::mutex> lk(mu_);
    running_ = false;
    cv_.notify_all();
  }

  size_t size() {
    std::lock_guard<std::mutex> lk(mu_);
    return queue_.size();
  }

 private:
  std::deque<std::vector<uint8_t>> queue_;
  std::mutex mu_;
  std::condition_variable cv_;
  size_t max_blocks_;
  bool running_ = true;
  long drop_count_ = 0;
};

int main() {
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // ── 配置 ──────────────────────────────────────────────────────────
  SpeakerOutputParams spk;
  spk.pcm_device = envOr("SPK_PCM_DEVICE", spk.pcm_device);
  spk.keyword = envOr("SPK_KEYWORD", spk.keyword);
  spk.sample_rate = std::stoul(envOr("SPK_SAMPLE_RATE", "16000"));
  spk.channels = std::stoul(envOr("SPK_CHANNELS", "1"));
  spk.period_frames = std::stoul(envOr("SPK_PERIOD_FRAMES", "512"));

  const std::string audio_dir = expandHome(
      envOr("AUDIO_FILE_DIR", "~/.config/lejuconfig/music"));
  const size_t queue_max = std::stoul(envOr("AUDIO_QUEUE_MAX", "500"));

  // ── 打开扬声器（失败即退出，不影响主控制链路）────────────────────
  SpeakerOutput speaker(spk);
  if (!speaker.open()) {
    std::cerr << "[ERROR] failed to open speaker, exit" << std::endl;
    return 1;
  }

  PcmQueue queue(queue_max);

  // ── 播放线程 ──────────────────────────────────────────────────────
  std::thread play_thread([&]() {
    std::vector<uint8_t> block;
    while (g_running) {
      if (!queue.pop(block)) {
        if (!g_running) break;
        continue;
      }
      speaker.write(block.data(), block.size());
    }
  });

  // 把整段 PCM 分块入队
  auto enqueue_pcm = [&](const uint8_t* data, size_t bytes) {
    for (size_t off = 0; off < bytes; off += kEnqueueChunkBytes) {
      size_t n = std::min(kEnqueueChunkBytes, bytes - off);
      queue.push(std::vector<uint8_t>(data + off, data + off + n));
    }
  };

  // 按文件名播放：先清队列 + drop 打断当前播放，再解码入队（RPC 与 topic 共用）。
  // 返回 {success, message}。
  auto play_file_by_name = [&](const std::string& name, int volume)
      -> std::pair<bool, std::string> {
    std::string file = name;
    if (!file.empty() && file[0] != '/') {
      file = audio_dir + "/" + file;
    }
    if (volume <= 0) volume = 100;
    std::cout << "[INFO] play_file: '" << file << "' volume=" << volume << std::endl;

    DecodeParams dp;
    dp.sample_rate = speaker.rate();
    dp.channels = speaker.channels();
    dp.volume = volume;

    std::vector<uint8_t> pcm;
    std::string err;
    if (!leju::audio::decodeFile(file, dp, pcm, err)) {
      std::cerr << "[ERROR] decode failed: " << err << std::endl;
      return {false, err};
    }
    // 打断当前播放：清队列 + 丢弃 ALSA 缓冲，让新音频立即替换上一首
    queue.clear();
    speaker.drop();
    enqueue_pcm(pcm.data(), pcm.size());
    return {true, "queued " + std::to_string(pcm.size()) + " bytes"};
  };

  // ── DDS ───────────────────────────────────────────────────────────
  dds::domain::DomainParticipant participant(kDefaultDomainId);

  // 输入2：PCM 流 topic（流式追加，不打断自己）
  TopicSubscriber<AudioReceiverData> play_sub(
      participant, kAudioPlay,
      [&](const AudioReceiverData& msg) {
        const auto& d = msg.data();
        if (!d.empty()) enqueue_pcm(d.data(), d.size());
      });

  // 输入3：按文件名播放 topic（fire-and-forget，data=文件名，volume 固定 100）
  TopicSubscriber<StringData> play_file_sub(
      participant, kAudioPlayFile,
      [&](const StringData& msg) {
        if (!msg.data().empty()) play_file_by_name(msg.data(), 100);
      });

  // 停止：stop topic
  TopicSubscriber<StringData> stop_sub(
      participant, kAudioStop,
      [&](const StringData&) {
        queue.clear();
        speaker.drop();
        std::cout << "[INFO] stop: queue cleared" << std::endl;
      });

  // 输入1：RPC play_file
  dds_entity_t participant_handle = participant->get_ddsc_entity();
  dds_rpc::Replier<PlayAudioRequest, PlayAudioReply> replier(participant_handle,
                                                             kAudioPlayerService);
  replier.set_handler([&](const PlayAudioRequest& req) {
    PlayAudioReply reply{};
    auto [ok, msg] = play_file_by_name(req.file_name(), req.volume());
    reply.success(ok);
    reply.message(msg);
    return reply;
  });

  std::cout << "[INFO] audio_player_node ready" << std::endl;
  std::cout << "[INFO]   speaker: " << spk.pcm_device << std::endl;
  std::cout << "[INFO]   rpc:     " << kAudioPlayerService << " (play_file)" << std::endl;
  std::cout << "[INFO]   sub:     " << kAudioPlay << " (PCM stream), " << kAudioPlayFile
            << " (play by name), " << kAudioStop << " (stop)" << std::endl;
  std::cout << "[INFO]   dir:     " << audio_dir << std::endl;

  // ── 主循环：处理 RPC ──────────────────────────────────────────────
  while (g_running) {
    replier.process_one(std::chrono::milliseconds(100));
  }

  std::cout << "[INFO] shutting down..." << std::endl;
  queue.stop();
  if (play_thread.joinable()) play_thread.join();
  speaker.close();
  return 0;
}
