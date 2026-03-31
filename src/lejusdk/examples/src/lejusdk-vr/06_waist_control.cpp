/**
 * @file 06_waist_control.cpp
 * @brief 腰部关节控制示例
 *
 * 本示例演示如何使用 VR API 控制机器人腰部：
 * - 切换腰部到外部控制模式
 * - 发布腰部关节指令
 * - 支持 Ctrl+C 优雅退出
 *
 * 腰部关节: 1 DOF
 * - q[0]: yaw (偏航角，正值左转)
 *
 * 支持机器人：Kuavo5, Roban2（Kuavo4 无腰部关节）
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

  std::cout << "=== 腰部关节控制示例 ===" << std::endl;

  // 1. 创建并初始化 API
  KuavoVRAPI vr;
  if (!vr.initialize()) {
    std::cerr << "初始化失败" << std::endl;
    return 1;
  }
  std::cout << "初始化成功" << std::endl;

  // 2. 设置腰部为外部控制模式
  std::cout << "切换腰部到外部控制模式..." << std::endl;
  if (!vr.setWaistMode(ControlMode::kExternal)) {
    std::cerr << "切换模式失败" << std::endl;
    return 1;
  }
  std::cout << "腰部已切换到外部控制模式" << std::endl;

  // 3. 准备腰部指令
  JointTrajectoryPoint waist_cmd;
  waist_cmd.q = {0.0};
  waist_cmd.v = {0.0};
  waist_cmd.acc = {0.0};

  // 4. 运动参数
  constexpr double FREQUENCY = 100.0;  // 控制频率 100 Hz
  constexpr double DT = 1.0 / FREQUENCY;
  constexpr double PERIOD = 4.0;       // 周期 4 秒
  constexpr double AMPLITUDE = 30.0 * M_PI / 180.0;  // 振幅 +-30 度

  double t = 0.0;
  int loop_count = 0;

  std::cout << "\n开始控制循环，频率: " << FREQUENCY << " Hz，周期: " << PERIOD << " s" << std::endl;
  std::cout << "按 Ctrl+C 停止\n" << std::endl;

  auto last_time = std::chrono::steady_clock::now();

  // 5. 控制循环
  while (g_running) {
    // 计算正弦波相位
    double phase = 2.0 * M_PI * t / PERIOD;

    // 腰部左右转动
    waist_cmd.q[0] = AMPLITUDE * std::sin(phase);

    // 发布指令
    vr.publishWaistJointCmd(waist_cmd);

    // 每秒打印一次状态
    if (loop_count % static_cast<int>(FREQUENCY) == 0) {
      std::cout << "t=" << std::fixed << std::setprecision(1) << t << "s"
                << "  waist=" << std::setprecision(1) << waist_cmd.q[0] * 180.0 / M_PI << "°" << std::endl;
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

  // 6. 恢复自动模式
  std::cout << "\n恢复腰部到自动模式..." << std::endl;
  vr.setWaistMode(ControlMode::kAuto);

  // 7. 关闭
  vr.shutdown();
  std::cout << "已关闭" << std::endl;

  return 0;
}
