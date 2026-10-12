// 轻量单测：SpeakerOutputParams 默认值 + findPlaybackCardName 空/不存在关键词行为。
// 不打开真实设备，可在任意机器编译运行。
#include <cassert>
#include <iostream>
#include "speaker_output.h"

using leju::audio::SpeakerOutput;
using leju::audio::SpeakerOutputParams;
using leju::audio::findPlaybackCardName;

int main() {
  // 默认参数
  SpeakerOutputParams p;
  assert(p.pcm_device == "plughw:CARD=Device_1,DEV=0");
  assert(p.sample_rate == 16000);
  assert(p.channels == 1);
  assert(p.period_frames == 512);
  std::cout << "[OK] default params" << std::endl;

  // 不存在的关键词应返回空串（不崩）
  std::string none = findPlaybackCardName("ZZZ_NOT_A_REAL_DEVICE_ZZZ");
  assert(none.empty());
  std::cout << "[OK] non-existent keyword -> empty" << std::endl;

  // 真实播放设备关键词在板上应能找到（其它机器跳过）
  std::string card = findPlaybackCardName("USB Audio");
  if (card.empty()) {
    std::cout << "[SKIP] no USB Audio playback device on this host" << std::endl;
  } else {
    std::cout << "[OK] found card: " << card << std::endl;
  }
  return 0;
}
