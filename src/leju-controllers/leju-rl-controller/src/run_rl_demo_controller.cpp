#include "leju-rl-controller/controllers/rl_demo_controller.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/time_utils.hpp"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  // Check if config file argument is provided
  std::string config_file;
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <config_file>" << std::endl;
    return 1;
  }
  config_file = argv[1];

  try {
    leju::rl_demo::RLDemoController controller(leju::RobotVersion::from_env());
    if (!controller.initialize(config_file)) {
      std::cerr << "❌ Failed to initialize Kuavo robot controller"
                << std::endl;
      return 1;
    }
    controller.start();
    controller.mainLoop();
  } catch (const std::exception& e) {
    std::cerr << "RL controller with exception: " << e.what()
              << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "RL controller with unknown exception" << std::endl;
    return 1;
  }
  std::cout << "exit controller." << std::endl;

  return 0;
}
