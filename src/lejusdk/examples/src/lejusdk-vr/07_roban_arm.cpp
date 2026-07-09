/**
 * @file 07_roban_arm.cpp
 * @brief Roban 手臂关节控制示例
 *
 * 本示例演示如何使用 RobanVRAPI 控制 Roban 机器人手臂：
 * - 使用 RobanVRAPI 而非 KuavoVRAPI
 * - Roban 手臂为 8 DOF (左臂 4 + 右臂 4)
 * - 支持 Ctrl+C 优雅退出
 *
 * Roban 手臂关节定义 (8 DOF = 左臂 4 + 右臂 4):
 *   左臂 [0-3]:
 *     [0] zarm_l1 = shoulder_pitch (肩部前屈/后伸)
 *     [1] zarm_l2 = shoulder_roll  (肩部外展/内收)
 *     [2] zarm_l3 = wrist          (手腕旋转)
 *     [3] zarm_l4 = elbow          (肘部弯曲)
 *   右臂 [4-7]:
 *     [4-7] zarm_r1 ~ zarm_r4 (与左臂对应)
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

  std::cout << "=== Roban 手臂控制示例 ===" << std::endl;

  // 1. 创建 Roban VR API（注意使用 RobanVRAPI）
  RobanVRAPI vr;
  if (!vr.initialize()) {
    std::cerr << "初始化失败" << std::endl;
    return 1;
  }
  std::cout << "初始化成功 (Roban)" << std::endl;

  // 2. 设置手臂为外部控制模式
  std::cout << "切换手臂到外部控制模式..." << std::endl;
  if (!vr.setArmMode(ControlMode::kExternal)) {
    std::cerr << "切换模式失败" << std::endl;
    return 1;
  }
  std::cout << "手臂已切换到外部控制模式" << std::endl;

  // 3. 准备手臂指令 (Roban: 8 DOF)
  constexpr int ARM_DOF = 8;
  JointTrajectoryPoint cmd;
  cmd.q.resize(ARM_DOF, 0.0);
  cmd.v.resize(ARM_DOF, 0.0);
  cmd.acc.resize(ARM_DOF, 0.0);

  // 4. 运动参数
  constexpr double FREQUENCY = 100.0;  // 控制频率 100 Hz
  constexpr double DT = 1.0 / FREQUENCY;
  constexpr double PERIOD = 4.0;  // 周期 4 秒

  // 关节运动幅度 (弧度)
  constexpr double SHOULDER_PITCH_MAX = 45.0 * M_PI / 180.0;  // 肩部前抬 45 度
  constexpr double SHOULDER_ROLL_MAX = 30.0 * M_PI / 180.0;   // 肩部外展 30 度
  constexpr double ELBOW_MAX = 90.0 * M_PI / 180.0;           // 肘部弯曲 90 度

  double t = 0.0;
  int loop_count = 0;

  std::cout << "\n开始控制循环，频率: " << FREQUENCY << " Hz，周期: " << PERIOD << " s" << std::endl;
  std::cout << "Roban 手臂: 8 DOF (左臂 4 + 右臂 4)" << std::endl;
  std::cout << "按 Ctrl+C 停止\n" << std::endl;

  auto last_time = std::chrono::steady_clock::now();

  // 5. 控制循环
  while (g_running) {
    // 计算正弦波相位
    double phase = 2.0 * M_PI * t / PERIOD;

    // 左臂运动: 抬起手臂并弯曲肘部
    cmd.q[0] = -SHOULDER_PITCH_MAX * 0.5 * (1.0 - std::cos(phase));  // shoulder_pitch: 向前抬起
    cmd.q[1] = SHOULDER_ROLL_MAX * 0.5 * (1.0 - std::cos(phase));    // shoulder_roll: 向外展开
    cmd.q[2] = 0.0;                                                   // wrist: 保持不动
    cmd.q[3] = -ELBOW_MAX * 0.5 * (1.0 - std::cos(phase));           // elbow: 弯曲

    // 右臂镜像运动
    cmd.q[4] = -SHOULDER_PITCH_MAX * 0.5 * (1.0 - std::cos(phase));  // shoulder_pitch: 向前抬起
    cmd.q[5] = -SHOULDER_ROLL_MAX * 0.5 * (1.0 - std::cos(phase));   // shoulder_roll: 向外展开 (镜像)
    cmd.q[6] = 0.0;                                                   // wrist: 保持不动
    cmd.q[7] = -ELBOW_MAX * 0.5 * (1.0 - std::cos(phase));           // elbow: 弯曲

    // 发布指令
    if (!vr.publishArmJointCmd(cmd)) {
      std::cerr << "发布指令失败" << std::endl;
    }

    // 每秒打印一次状态
    if (loop_count % static_cast<int>(FREQUENCY) == 0) {
      std::cout << "t=" << std::fixed << std::setprecision(1) << t << "s"
                << "  shoulder_pitch=" << std::setprecision(1) << cmd.q[0] * 180.0 / M_PI << "°"
                << "  elbow=" << cmd.q[3] * 180.0 / M_PI << "°" << std::endl;
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

  // 6. 恢复手臂到自动模式
  std::cout << "\n恢复手臂到自动模式..." << std::endl;
  vr.setArmMode(ControlMode::kAuto);

  // 7. 关闭
  vr.shutdown();
  std::cout << "已关闭" << std::endl;

  return 0;
}
