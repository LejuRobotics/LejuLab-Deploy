// 音频解码：subprocess 调 ffmpeg 把任意格式解码成 16k/mono/S16LE PCM。
#ifndef LEJU_AUDIO_AUDIO_DECODER_H_
#define LEJU_AUDIO_AUDIO_DECODER_H_

#include <cstdint>
#include <string>
#include <vector>

namespace leju {
namespace audio {

struct DecodeParams {
  unsigned int sample_rate = 16000;
  unsigned int channels = 1;
  int volume = 100;  // 音量百分比，100 = 原音量
};

// 用 ffmpeg 把 path 指向的音频文件解码成 S16LE PCM 字节，写入 out。
// 成功返回 true（out 非空）；文件不存在/ffmpeg 失败/空输出返回 false。
// 通过 fork+execvp 直接执行 ffmpeg，不经 shell（避免路径注入）。
bool decodeFile(const std::string& path, const DecodeParams& params,
                std::vector<uint8_t>& out, std::string& error_msg);

}  // namespace audio
}  // namespace leju

#endif  // LEJU_AUDIO_AUDIO_DECODER_H_
