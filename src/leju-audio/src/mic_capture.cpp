#include "mic_capture.h"

#include <iostream>
#include <unistd.h>

namespace {

void quiet_error_handler(const char*, int, const char*, int, const char*, ...) {}

}  // namespace

namespace leju {
namespace audio {

std::string findMicCardName(const std::string& keyword) {
  int card = -1;
  while (snd_card_next(&card) >= 0 && card >= 0) {
    snd_ctl_t* ctl = nullptr;
    std::string ctlname = "hw:" + std::to_string(card);
    if (snd_ctl_open(&ctl, ctlname.c_str(), 0) < 0) continue;

    snd_ctl_card_info_t* info = nullptr;
    snd_ctl_card_info_alloca(&info);
    snd_ctl_card_info(ctl, info);
    std::string card_long_name = snd_ctl_card_info_get_name(info);
    std::string card_id = snd_ctl_card_info_get_id(info);

    int dev = -1;
    while (snd_ctl_pcm_next_device(ctl, &dev) >= 0 && dev >= 0) {
      snd_pcm_info_t* pcminfo = nullptr;
      snd_pcm_info_alloca(&pcminfo);
      snd_pcm_info_set_device(pcminfo, dev);
      snd_pcm_info_set_subdevice(pcminfo, 0);
      snd_pcm_info_set_stream(pcminfo, SND_PCM_STREAM_CAPTURE);
      if (snd_ctl_pcm_info(ctl, pcminfo) < 0) continue;
      // 关键词匹配：卡长名、卡ID、PCM 名，三者任一命中即匹配
      const char* pcm_name = snd_pcm_info_get_name(pcminfo);
      if (card_long_name.find(keyword) != std::string::npos ||
          card_id.find(keyword) != std::string::npos ||
          (pcm_name && std::string(pcm_name).find(keyword) != std::string::npos)) {
        // 返回卡 ID（短名），用于构造 plughw:CARD=<id>,DEV=0
        // ALSA 的 CARD= 参数按 ID 查找，不接受长描述名
        snd_ctl_close(ctl);
        return card_id;
      }
    }
    snd_ctl_close(ctl);
  }
  return "";
}

MicCapture::MicCapture(MicCaptureParams params) : params_(std::move(params)) {}

MicCapture::~MicCapture() { close(); }

bool MicCapture::open() {
  // 设备发现：按关键词找卡名
  int attempt = 0;
  while (card_name_.empty() && attempt < params_.open_retry) {
    card_name_ = findMicCardName(params_.keyword);
    if (card_name_.empty()) {
      attempt++;
      std::cerr << "[WARN] no mic matching '" << params_.keyword
                << "', retry " << attempt << "/" << params_.open_retry << std::endl;
      if (attempt < params_.open_retry) sleep(params_.open_retry_interval_s);
    }
  }
  if (!card_name_.empty()) {
    // 用实际发现的卡名构建 PCM 设备名，覆盖默认值
    params_.pcm_device = "plughw:CARD=" + card_name_ + ",DEV=0";
    std::cout << "[INFO] mic device: '" << params_.keyword
              << "' (card '" << card_name_ << "'), pcm='" << params_.pcm_device << "'"
              << std::endl;
  } else {
    std::cerr << "[ERROR] no mic matching '" << params_.keyword
              << "', retried " << params_.open_retry << "/" << params_.open_retry
              << std::endl;
    return false;
  }

  // 抑制 ALSA C 库 stderr 噪音
  snd_lib_error_set_handler(quiet_error_handler);

  int err = snd_pcm_open(&handle_, params_.pcm_device.c_str(),
                         SND_PCM_STREAM_CAPTURE, 0);
  if (err < 0) {
    std::cerr << "[ERROR] snd_pcm_open('" << params_.pcm_device
              << "'): " << snd_strerror(err) << std::endl;
    return false;
  }

  snd_pcm_hw_params_t* hw = nullptr;
  snd_pcm_hw_params_alloca(&hw);
  snd_pcm_hw_params_any(handle_, hw);

  snd_pcm_hw_params_set_access(handle_, hw, SND_PCM_ACCESS_RW_INTERLEAVED);
  snd_pcm_hw_params_set_format(handle_, hw, SND_PCM_FORMAT_S16_LE);
  rate_ = params_.sample_rate;
  snd_pcm_hw_params_set_rate_near(handle_, hw, &rate_, nullptr);
  channels_ = params_.channels;
  snd_pcm_hw_params_set_channels_near(handle_, hw, &channels_);
  period_ = params_.period_frames;
  snd_pcm_hw_params_set_period_size_near(handle_, hw, &period_, nullptr);
  snd_pcm_uframes_t buffer_size = period_ * 4;
  snd_pcm_hw_params_set_buffer_size_near(handle_, hw, &buffer_size);

  err = snd_pcm_hw_params(handle_, hw);
  if (err < 0) {
    std::cerr << "[ERROR] set params failed: rate=" << params_.sample_rate
              << " ch=" << params_.channels << " fmt=S16_LE: "
              << snd_strerror(err) << std::endl;
    snd_pcm_close(handle_);
    handle_ = nullptr;
    return false;
  }

  snd_pcm_prepare(handle_);
  double hz = static_cast<double>(rate_) / static_cast<double>(period_);
  std::cout << "[INFO] negotiated: rate=" << rate_ << " channels=" << channels_
            << " fmt=S16_LE period=" << period_ << " frames (" << hz
            << " Hz, " << period_ * (channels_ * 2) << " bytes/frame)" << std::endl;
  return true;
}

int MicCapture::read(std::vector<uint8_t>& out) {
  if (!handle_) return -1;
  size_t frame_bytes = period_ * channels_ * 2;  // S16 = 2 bytes
  out.resize(frame_bytes);
  snd_pcm_sframes_t n = snd_pcm_readi(handle_, out.data(), period_);
  if (n == -EPIPE) {
    overrun_count_++;
    std::cerr << "[WARN] overrun, recovered (count=" << overrun_count_ << ")" << std::endl;
    snd_pcm_prepare(handle_);
    return 0;
  } else if (n < 0) {
    std::cerr << "[ERROR] readi: " << snd_strerror(n) << std::endl;
    return static_cast<int>(n);
  }
  out.resize(static_cast<size_t>(n) * (channels_ * 2));
  return static_cast<int>(out.size());
}

void MicCapture::close() {
  if (handle_) {
    snd_pcm_drain(handle_);
    snd_pcm_close(handle_);
    handle_ = nullptr;
  }
}

}  // namespace audio
}  // namespace leju
