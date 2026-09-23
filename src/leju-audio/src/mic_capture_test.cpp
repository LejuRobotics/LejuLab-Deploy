// 轻量单测：MicCaptureParams 默认值 + findMicCardName 空关键词/不存在关键词行为。
// 不打开真实设备，可在任意机器编译运行。
#include <cassert>
#include <iostream>
#include "mic_capture.h"

using leju::audio::MicCapture;
using leju::audio::MicCaptureParams;
using leju::audio::findMicCardName;

int main() {
  // 默认参数
  MicCaptureParams p;
  assert(p.pcm_device == "plughw:CARD=Device,DEV=0");
  assert(p.sample_rate == 16000);
  assert(p.channels == 1);
  assert(p.period_frames == 512);
  std::cout << "[OK] default params" << std::endl;

  // 不存在的关键词应返回空串（不崩）
  std::string none = findMicCardName("ZZZ_NOT_A_REAL_DEVICE_ZZZ");
  assert(none.empty());
  std::cout << "[OK] non-existent keyword -> empty" << std::endl;

  // 真实设备关键词（USB Composite Device）在 3588 上应能找到
  std::string card = findMicCardName("USB Composite Device");
  if (card.empty()) {
    std::cout << "[SKIP] no USB Composite Device on this host" << std::endl;
  } else {
    std::cout << "[OK] found card: " << card << std::endl;
  }
  return 0;
}
