// 麦克风采集封装：ALSA plughw 打开设备名，自动重采样到目标采样率。
#ifndef LEJU_AUDIO_MIC_CAPTURE_H_
#define LEJU_AUDIO_MIC_CAPTURE_H_

#include <cstdint>
#include <string>
#include <vector>

#include <alsa/asoundlib.h>

namespace leju {
namespace audio {

struct MicCaptureParams {
  std::string pcm_device = "plughw:CARD=Device,DEV=0";  // ALSA 设备名（禁止卡号）
  std::string keyword = "USB Composite Device";          // 设备名匹配关键词
  unsigned int sample_rate = 16000;
  unsigned int channels = 1;
  snd_pcm_uframes_t period_frames = 512;
  int open_retry = 3;            // 找不到设备时重试次数
  int open_retry_interval_s = 3; // 重试间隔
};

// 遍历 ALSA 卡，找 name 含 keyword 的 capture 卡，返回卡 ID（短名，如 "Device"）。
// 找不到返回空串。
std::string findMicCardName(const std::string& keyword);

class MicCapture {
 public:
  explicit MicCapture(MicCaptureParams params = {});
  ~MicCapture();

  // 禁拷贝
  MicCapture(const MicCapture&) = delete;
  MicCapture& operator=(const MicCapture&) = delete;

  // 打开设备并协商参数。成功返回 true。
  // 找不到匹配设备时按 open_retry 退避重试；仍失败返回 false。
  bool open();

  // 阻塞读一个 period 的 PCM 字节到 out，返回字节数。
  // <0 表示错误（EPIPE 已自动 recover 并返回该次读到 0，仅记 warn）。
  int read(std::vector<uint8_t>& out);

  void close();

  // 协商后的实际参数（open 成功后有效）
  unsigned int rate() const { return rate_; }
  unsigned int channels() const { return channels_; }
  snd_pcm_format_t format() const { return SND_PCM_FORMAT_S16_LE; }
  snd_pcm_uframes_t period() const { return period_; }
  const std::string& card_name() const { return card_name_; }

 private:
  MicCaptureParams params_;
  snd_pcm_t* handle_ = nullptr;
  unsigned int rate_ = 0;
  unsigned int channels_ = 0;
  snd_pcm_uframes_t period_ = 0;
  std::string card_name_;
  int overrun_count_ = 0;
};

}  // namespace audio
}  // namespace leju

#endif  // LEJU_AUDIO_MIC_CAPTURE_H_
