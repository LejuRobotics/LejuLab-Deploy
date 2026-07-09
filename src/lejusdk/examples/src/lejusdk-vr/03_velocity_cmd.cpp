/**
 * @file 03_velocity_cmd.cpp
 * @brief VR 速度输入发布示例
 *
 * 本示例演示如何使用 VR API 发布 VR 速度输入：
 * - 构建 VR 速度输入
 * - 发布速度输入控制机器人移动
 * - 发送零速度停止机器人
 */

#include <lejusdk-vr/lejusdk_vr.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace leju::vr;

int main() {
  std::cout << "=== 速度指令发布示例 ===" << std::endl;

  // 1. 创建并初始化 API
  KuavoVRAPI vr;
  if (!vr.initialize()) {
    std::cerr << "初始化失败" << std::endl;
    return 1;
  }
  std::cout << "初始化成功" << std::endl;

  // 2. 构建 VR 速度输入
  //    坐标系：X 正向前进，Y 正向左，Z 正为逆时针
  VrVelocityCmd cmd;
  cmd.linear_x = 0.3;   // 前进 0.3 m/s
  cmd.linear_y = 0.0;   // 无侧移
  cmd.angular_z = 0.1;  // 逆时针旋转 0.1 rad/s

  // 3. 发布速度指令
  std::cout << "开始发布速度指令..." << std::endl;
  for (int i = 0; i < 10; ++i) {
    if (vr.publishVrVelocityCmd(cmd)) {
      std::cout << "速度指令已发布 #" << i + 1 << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 4. 停止机器人（发送零速度）
  std::cout << "停止机器人..." << std::endl;
  cmd.linear_x = 0.0;
  cmd.linear_y = 0.0;
  cmd.angular_z = 0.0;
  vr.publishVrVelocityCmd(cmd);
  std::cout << "机器人已停止" << std::endl;

  // 5. 关闭
  vr.shutdown();
  std::cout << "已关闭" << std::endl;

  return 0;
}
