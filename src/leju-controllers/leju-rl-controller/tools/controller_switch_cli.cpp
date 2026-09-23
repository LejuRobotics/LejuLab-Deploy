#include <lejusdk-vr/lejusdk_vr.h>

#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "用法: " << argv[0] << " <controller_name>\n"
              << "示例: " << argv[0] << " depth_walk\n";
    return 2;
  }

  leju::vr::KuavoVRAPI api;
  if (!api.initialize()) {
    std::cerr << "DDS ControllerService 初始化失败\n";
    return 1;
  }

  std::string message;
  const bool ok = api.switchController(argv[1], &message);
  api.shutdown();
  if (!ok) {
    std::cerr << "控制器切换失败: " << message << "\n";
    return 1;
  }
  std::cout << "控制器切换请求已发送: " << argv[1] << "\n";
  if (!message.empty()) std::cout << message << "\n";
  return 0;
}
