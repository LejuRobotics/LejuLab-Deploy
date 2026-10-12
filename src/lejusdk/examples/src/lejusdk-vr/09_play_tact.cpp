/**
 * @file 09_play_tact.cpp
 * @brief tact 动作播放测试（C++ 客户端，仿真单测用）
 *
 * 直接调 VR API 的 playTact → DDS-RPC TactPlayerService/PlayTact，
 * 触发 RK3588 lejulab 的 TactPlayer 解析并播放 .tact 动作。
 *
 * 用法：
 *   # 1. 启动仿真（含 TactPlayer），按遥控器 start
 *   # 2. 发 PlayTact（动作名不含 .tact 后缀）
 *   CYCLONEDDS_URI=file://.../cyclonedds.xml \
 *   LD_LIBRARY_PATH=...cyclonedds-0.10.2/lib:...cyclonedds-cxx-0.10.2/lib \
 *   ./vr_09_play_tact 抱拳
 */

#include <lejusdk-vr/lejusdk_vr.h>

#include <iostream>
#include <string>

using namespace leju::vr;

int main(int argc, char** argv) {
  std::string name = (argc >= 2) ? argv[1] : "抱拳";

  std::cout << "=== tact 播放测试: '" << name << "' ===" << std::endl;

  KuavoVRAPI vr;
  if (!vr.initialize()) {
    std::cerr << "VR API 初始化失败" << std::endl;
    return 1;
  }
  std::cout << "初始化成功，发送 playTact..." << std::endl;

  std::string message;
  bool ok = vr.playTact(name, &message, 5000);

  std::cout << "结果: success=" << (ok ? "true" : "false")
            << "  message=\"" << message << "\"" << std::endl;

  vr.shutdown();
  return ok ? 0 : 1;
}
