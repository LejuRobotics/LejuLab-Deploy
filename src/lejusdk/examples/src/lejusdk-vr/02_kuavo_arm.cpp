/**
 * @file 02_kuavo_arm.cpp
 * @brief Kuavo 手臂关节控制示例
 *
 * 本示例演示如何使用 VR API 控制 Kuavo 机器人手臂：
 * - 切换手臂到外部控制模式
 * - 使用正弦波生成平滑的周期运动
 * - 支持 Ctrl+C 优雅退出
 *
 * Kuavo 手臂关节定义 (14 DOF = 左臂 7 + 右臂 7):
 *   左臂 [0-6]:
 *     [0] zarm_l1 = shoulder_pitch (肩部前屈/后伸)
 *     [1] zarm_l2 = shoulder_roll  (肩部外展/内收)
 *     [2] zarm_l3 = shoulder_yaw   (肩部旋转)
 *     [3] zarm_l4 = elbow          (肘部弯曲)
 *     [4] zarm_l5 = wrist_roll     (手腕旋转)
 *     [5] zarm_l6 = wrist_pitch    (手腕俯仰)
 *     [6] zarm_l7 = wrist_yaw      (手腕偏航)
 *   右臂 [7-13]:
 *     [7-13] zarm_r1 ~ zarm_r7 (与左臂对应)
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

  std::cout << "=== Kuavo 手臂关节控制示例 ===" << std::endl;

  // 1. 创建并初始化 API
  KuavoVRAPI vr;
  if (!vr.initialize()) {
    std::cerr << "初始化失败" << std::endl;
    return 1;
  }
  std::cout << "初始化成功" << std::endl;

  // 2. 设置手臂为外部控制模式
  std::cout << "切换手臂到外部控制模式..." << std::endl;
  if (!vr.setArmMode(ControlMode::kExternal)) {
    std::cerr << "切换模式失败" << std::endl;
    return 1;
  }
  std::cout << "手臂已切换到外部控制模式" << std::endl;

  // 3. 准备手臂指令
  // Kuavo 手臂: 14 DOF (左臂 7 + 右臂 7)
  constexpr int ARM_DOF = 14;
  JointTrajectoryPoint cmd;
  cmd.q.resize(ARM_DOF, 0.0);
  cmd.v.resize(ARM_DOF, 0.0);
  cmd.acc.resize(ARM_DOF, 0.0);

  // 4. 运动参数
  constexpr double FREQUENCY = 100.0;  // 控制频率 100 Hz
  constexpr double DT = 1.0 / FREQUENCY;
  constexpr double PERIOD = 4.0;  // 周期 4 秒

  // 关节运动幅度 (弧度)
  constexpr double SHOULDER_PITCH_MAX = 60.0 * M_PI / 180.0;  // 肩部前抬 60 度
  constexpr double SHOULDER_ROLL_MAX = 30.0 * M_PI / 180.0;   // 肩部外展 30 度
  constexpr double ELBOW_MAX = 90.0 * M_PI / 180.0;           // 肘部弯曲 90 度

  double t = 0.0;
  int loop_count = 0;

  std::cout << "\n开始控制循环，频率: " << FREQUENCY << " Hz，周期: " << PERIOD << " s" << std::endl;
  std::cout << "按 Ctrl+C 停止\n" << std::endl;

  auto last_time = std::chrono::steady_clock::now();

  // 5. 控制循环
  while (g_running) {
    // 计算正弦波相位
    double phase = 2.0 * M_PI * t / PERIOD;

    // 左臂运动: 抬起手臂并弯曲肘部
    cmd.q[0] = -SHOULDER_PITCH_MAX * 0.5 * (1.0 - std::cos(phase));  // shoulder_pitch: 向前抬起
    cmd.q[1] = SHOULDER_ROLL_MAX * 0.5 * (1.0 - std::cos(phase));    // shoulder_roll: 向外展开
    cmd.q[2] = 0.0;                                                   // shoulder_yaw: 保持不动
    cmd.q[3] = -ELBOW_MAX * 0.5 * (1.0 - std::cos(phase));           // elbow: 弯曲

    // 右臂镜像运动
    cmd.q[7] = -SHOULDER_PITCH_MAX * 0.5 * (1.0 - std::cos(phase));  // shoulder_pitch: 向前抬起
    cmd.q[8] = -SHOULDER_ROLL_MAX * 0.5 * (1.0 - std::cos(phase));   // shoulder_roll: 向外展开 (镜像)
    cmd.q[9] = 0.0;                                                   // shoulder_yaw: 保持不动
    cmd.q[10] = -ELBOW_MAX * 0.5 * (1.0 - std::cos(phase));          // elbow: 弯曲

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
