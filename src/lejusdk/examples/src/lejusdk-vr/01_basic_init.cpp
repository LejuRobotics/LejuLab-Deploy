/**
 * @file 01_basic_init.cpp
 * @brief VR API 初始化与关闭示例
 *
 * 本示例演示 VR API 的基本使用流程：
 * - 创建 API 实例
 * - 初始化 API
 * - 关闭 API 并释放资源
 */

#include <lejusdk-vr/lejusdk_vr.h>

#include <iostream>

using namespace leju::vr;

int main() {
  std::cout << "=== VR API 初始化示例 ===" << std::endl;

  // 1. 创建 API 实例（默认使用 Kuavo 机器人）
  KuavoVRAPI vr;

  // 2. 初始化 API
  if (!vr.initialize()) {
    std::cerr << "初始化失败" << std::endl;
    return 1;
  }
  std::cout << "初始化成功" << std::endl;

  // 3. 检查初始化状态
  if (vr.isInitialized()) {
    std::cout << "API 已初始化，可以正常使用" << std::endl;
  }

  // ... 在此添加业务逻辑 ...

  // 4. 关闭 API，释放资源
  vr.shutdown();
  std::cout << "已关闭" << std::endl;

  return 0;
}
