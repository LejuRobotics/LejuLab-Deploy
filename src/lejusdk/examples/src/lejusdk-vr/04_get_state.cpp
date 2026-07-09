/**
 * @file 04_get_state.cpp
 * @brief 查询控制器状态示例
 *
 * 本示例演示如何查询当前控制器状态：
 * - 获取当前控制器名称
 * - 获取各部位控制模式
 * - 获取可用控制器列表
 */

#include <lejusdk-vr/lejusdk_vr.h>

#include <iostream>
#include <string>

using namespace leju::vr;

// 将控制模式转换为可读字符串
std::string modeToString(ControlMode mode) {
  switch (mode) {
    case ControlMode::kKeepPose:
      return "KeepPose";
    case ControlMode::kAuto:
      return "Auto";
    case ControlMode::kExternal:
      return "External";
    default:
      return "Unknown";
  }
}

int main() {
  std::cout << "=== 查询控制器状态示例 ===" << std::endl;

  // 1. 创建并初始化 API
  KuavoVRAPI vr;
  if (!vr.initialize()) {
    std::cerr << "初始化失败" << std::endl;
    return 1;
  }
  std::cout << "初始化成功" << std::endl;

  // 2. 查询控制器状态
  ControllerState state;
  if (vr.getControllerState(state)) {
    std::cout << "\n--- 控制器状态 ---" << std::endl;
    std::cout << "当前控制器: " << state.current_controller << std::endl;
    std::cout << "手臂模式: " << modeToString(state.arm_mode) << std::endl;
    std::cout << "头部模式: " << modeToString(state.head_mode) << std::endl;
    std::cout << "腰部模式: " << modeToString(state.waist_mode) << std::endl;

    std::cout << "可用控制器: ";
    for (const auto& ctrl : state.available_controllers) {
      std::cout << ctrl << " ";
    }
    std::cout << std::endl;
  } else {
    std::cerr << "获取控制器状态失败" << std::endl;
  }

  // 3. 关闭
  vr.shutdown();
  std::cout << "\n已关闭" << std::endl;

  return 0;
}
