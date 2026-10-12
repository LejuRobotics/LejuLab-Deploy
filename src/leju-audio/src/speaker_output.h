// 扬声器播放封装：ALSA plughw 打开设备名，声卡自动重采样到原生率。
// 对称于 mic_capture.h（采集侧）。
#ifndef LEJU_AUDIO_SPEAKER_OUTPUT_H_
#define LEJU_AUDIO_SPEAKER_OUTPUT_H_

#include <cstdint>
#include <string>
#include <vector>

#include <alsa/asoundlib.h>

namespace leju {
namespace audio {

struct SpeakerOutputParams {
  std::string pcm_device = "plughw:CARD=Device_1,DEV=0";  // ALSA 播放设备名（禁止卡号）
  std::string keyword = "USB Audio";                      // 设备名匹配关键词
  unsigned int sample_rate = 16000;
  unsigned int channels = 1;
  snd_pcm_uframes_t period_frames = 512;
  int open_retry = 3;            // 找不到设备时重试次数
  int open_retry_interval_s = 3; // 重试间隔
};

// 遍历 ALSA 卡，找 name 含 keyword 的 playback 卡，返回卡名（如 "Device_1"）。
// 找不到返回空串。
std::string findPlaybackCardName(const std::string& keyword);

class SpeakerOutput {
 public:
  explicit SpeakerOutput(SpeakerOutputParams params = {});
  ~SpeakerOutput();

  // 禁拷贝
  SpeakerOutput(const SpeakerOutput&) = delete;
  SpeakerOutput& operator=(const SpeakerOutput&) = delete;

  // 打开设备并协商参数。成功返回 true。
  // 找不到匹配设备时按 open_retry 退避重试；仍失败返回 false。
  bool open();

  // 阻塞写一段 S16LE PCM 字节。bytes 应为帧对齐（channels*2 的整数倍）。
  // 成功返回写入字节数；underrun 自动 recover 后重试；<0 表示不可恢复错误。
  int write(const uint8_t* pcm, size_t bytes);

  // 等待缓冲区内剩余样本播完（用于优雅结束）。
  void drain();
  // 丢弃缓冲区内未播样本（用于 stop）。
  void drop();

  void close();

  // 协商后的实际参数（open 成功后有效）
  unsigned int rate() const { return rate_; }
  unsigned int channels() const { return channels_; }
  snd_pcm_format_t format() const { return SND_PCM_FORMAT_S16_LE; }
  snd_pcm_uframes_t period() const { return period_; }
  const std::string& card_name() const { return card_name_; }
  bool is_open() const { return handle_ != nullptr; }

 private:
  SpeakerOutputParams params_;
  snd_pcm_t* handle_ = nullptr;
  unsigned int rate_ = 0;
  unsigned int channels_ = 0;
  snd_pcm_uframes_t period_ = 0;
  std::string card_name_;
  int underrun_count_ = 0;
};

}  // namespace audio
}  // namespace leju

#endif  // LEJU_AUDIO_SPEAKER_OUTPUT_H_
