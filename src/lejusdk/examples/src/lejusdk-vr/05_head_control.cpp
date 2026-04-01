/**
 * @file 05_head_control.cpp
 * @brief 头部关节控制示例
 *
 * 本示例演示如何使用 VR API 控制机器人头部：
 * - 头部控制无需切换模式，指令直接透传
 * - 发布头部关节指令（俯仰 + 偏航）
 * - 支持 Ctrl+C 优雅退出
 *
 * 头部关节: 2 DOF
 * - q[0]: pitch (俯仰角，正值低头)
 * - q[1]: yaw (偏航角，正值左转)
 */

#include <lejusdk-vr/lejusdk_vr.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace leju::vr;

// 全局变量用于信号处理
volatile bool g_running = true;

void signalHandler(int signum) {
  std::cout << "\n收到信号 " << signum << ", 正在停止..." << std::endl;
  g_running = false;
}

int main() {
  // 注册信号处理
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  std::cout << "=== 头部关节控制示例 ===" << std::endl;

  // 1. 创建并初始化 API
  KuavoVRAPI vr;
  if (!vr.initialize()) {
    std::cerr << "初始化失败" << std::endl;
    return 1;
  }
  std::cout << "初始化成功" << std::endl;

  // 2. 头部控制不需要设置模式，直接发送指令即可
  std::cout << "头部控制就绪（无需切换模式）" << std::endl;

  // 3. 准备头部指令
  JointTrajectoryPoint head_cmd;
  head_cmd.q = {0.0, 0.0};   // pitch, yaw
  head_cmd.v = {0.0, 0.0};
  head_cmd.acc = {0.0, 0.0};

  // 4. 运动参数
  constexpr double FREQUENCY = 100.0;  // 控制频率 100 Hz
  constexpr double DT = 1.0 / FREQUENCY;
  constexpr double PERIOD = 4.0;       // 周期 4 秒
  constexpr double PITCH_AMP = 20.0 * M_PI / 180.0;  // 俯仰振幅 +-20度
  constexpr double YAW_AMP = 30.0 * M_PI / 180.0;    // 偏航振幅 +-30度

  double t = 0.0;
  int loop_count = 0;

  std::cout << "\n开始控制循环，频率: " << FREQUENCY << " Hz，周期: " << PERIOD << " s" << std::endl;
  std::cout << "按 Ctrl+C 停止\n" << std::endl;

  auto last_time = std::chrono::steady_clock::now();

  // 5. 控制循环
  while (g_running) {
    // 计算正弦波相位
    double phase = 2.0 * M_PI * t / PERIOD;

    // 点头 + 摇头
    head_cmd.q[0] = PITCH_AMP * std::sin(phase);        // pitch: 点头
    head_cmd.q[1] = YAW_AMP * std::sin(phase * 0.5);    // yaw: 摇头 (较慢)

    // 发布指令
    vr.publishHeadJointCmd(head_cmd);

    // 每秒打印一次状态
    if (loop_count % static_cast<int>(FREQUENCY) == 0) {
      std::cout << "t=" << std::fixed << std::setprecision(1) << t << "s"
                << "  pitch=" << std::setprecision(1) << head_cmd.q[0] * 180.0 / M_PI << "°"
                << "  yaw=" << head_cmd.q[1] * 180.0 / M_PI << "°" << std::endl;
    }

    // 更新时间
    t += DT;
    loop_count++;

    // 精确定时
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time);
    auto sleep_time = std::chrono::microseconds(static_cast<int>(DT * 1e6)) - elapsed;
    if (sleep_time.count() > 0) {
      std::this_thread::sleep_for(sleep_time);
    }
    last_time = std::chrono::steady_clock::now();
  }

  // 6. 回到零位
  std::cout << "\n回到零位..." << std::endl;
  head_cmd.q = {0.0, 0.0};
  vr.publishHeadJointCmd(head_cmd);

  // 7. 关闭
  vr.shutdown();
  std::cout << "已关闭" << std::endl;

  return 0;
}
